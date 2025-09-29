/**
 * @file zedf9p.c
 * @brief ZEDF9P GNSS Module Driver Implementation - Enhanced UBX Protocol Support
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-09-10
 */

#include "zedf9p.h"
#include <stdio.h>
#include <string.h>

// Internal constants
//static const uint16_t UBX_HEADER_LENGTH = 6U;
//static const uint16_t UBX_CHECKSUM_LENGTH = 2U;
//static const uint32_t DEFAULT_TIMEOUT_MS = 1000U;
static const uint32_t UBX_POLL_TIMEOUT_MS = 5000U;
static const uint8_t UBX_SYNC_CHAR_1 = 0xB5U;
static const uint8_t UBX_SYNC_CHAR_2 = 0x62U;
//static const uint16_t UBX_I2C_BYTES_AVAILABLE_REG = 0xFD;
static const uint8_t UBX_MAX_RETRIES = 3U;

// Internal function prototypes
static zedf9p_status_t zedf9p_write_data(const zedf9p_t *dev, const uint8_t *data, uint16_t length);
//static zedf9p_status_t zedf9p_read_data(const zedf9p_t *dev, uint8_t *data, uint16_t length);
static zedf9p_status_t zedf9p_read_available_data(const zedf9p_t *dev, uint8_t *data, uint16_t max_length, uint16_t *bytes_read);
static zedf9p_status_t zedf9p_parse_ubx_byte(zedf9p_t *dev, uint8_t byte);
static zedf9p_status_t zedf9p_handle_message(zedf9p_t *dev, const ubx_message_t *message);
static zedf9p_status_t zedf9p_wait_for_message_response(zedf9p_t *dev, uint8_t msg_class, uint8_t msg_id,
                                                       uint32_t timeout_ms, bool wait_for_ack, ubx_message_t *response);
static zedf9p_status_t zedf9p_flush_input_buffer(const zedf9p_t *dev);
//static zedf9p_status_t zedf9p_check_i2c_bytes_available(const zedf9p_t *dev, uint16_t *bytes_available);

// Enhanced status error string lookup table
static const char* const STATUS_STRINGS[] = {
    [ZEDF9P_OK] = "OK",
    [-ZEDF9P_ERR_I2C] = "I2C communication failed",
    [-ZEDF9P_ERR_UART] = "UART communication failed",
    [-ZEDF9P_ERR_TIMEOUT] = "Timeout occurred",
    [-ZEDF9P_ERR_NULL] = "Null pointer",
    [-ZEDF9P_ERR_CRC] = "CRC check failed",
    [-ZEDF9P_ERR_INVALID_ARG] = "Invalid argument",
    [-ZEDF9P_ERR_BUFFER_FULL] = "Buffer full",
    [-ZEDF9P_ERR_NO_DATA] = "No data available",
    [-ZEDF9P_ERR_NACK] = "Negative acknowledgment received",
    [-ZEDF9P_ERR_INVALID_MESSAGE] = "Invalid message format",
    [-ZEDF9P_ERR_PARSE_ERROR] = "Parse error"
};

const char* zedf9p_status_error(const zedf9p_status_t status) {
    const int idx = (status <= 0) ? -status : 0;
    if (idx < (int)(sizeof(STATUS_STRINGS) / sizeof(STATUS_STRINGS[0])) && STATUS_STRINGS[idx] != NULL) {
        return STATUS_STRINGS[idx];
    }
    return "Unknown error";
}

static void zedf9p_verify_checksum_manual(const zedf9p_t *dev, const ubx_message_t *message) {
    if (!dev->io.debug_print) return;

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    // Start with class and ID
    ck_a += message->msg_class;
    ck_b += ck_a;
    ck_a += message->msg_id;
    ck_b += ck_a;

    // Add length bytes
    ck_a += (uint8_t)(message->length & 0xFF);
    ck_b += ck_a;
    ck_a += (uint8_t)((message->length >> 8) & 0xFF);
    ck_b += ck_a;

    // Add payload
    for (uint16_t i = 0; i < message->length; i++) {
        ck_a += message->payload[i];
        ck_b += ck_a;
    }

    char debug_buf[128];
    sprintf(debug_buf, "Manual checksum: CK_A=0x%02X CK_B=0x%02X vs Calculated: CK_A=0x%02X CK_B=0x%02X\n",
           ck_a, ck_b, dev->calculated_ck_a, dev->calculated_ck_b);
    dev->io.debug_print(debug_buf);
}

zedf9p_status_t zedf9p_init(zedf9p_t *dev, const zedf9p_interface_type_t interface_type,
                           const uint8_t address, const zedf9p_interface_t io) {
    if (dev == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // Validate interface functions based on interface type
    if (interface_type == ZEDF9P_INTERFACE_I2C) {
        if (io.i2c_write == NULL || io.i2c_read == NULL) {
            return ZEDF9P_ERR_NULL;
        }
    } else if (interface_type == ZEDF9P_INTERFACE_UART) {
        if (io.uart_write == NULL || io.uart_read == NULL) {
            return ZEDF9P_ERR_NULL;
        }
    } else {
        return ZEDF9P_ERR_INVALID_ARG;
    }

    if (io.delay_ms == NULL || io.get_millis == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // Initialize device structure
    memset(dev, 0, sizeof(zedf9p_t));
    dev->interface_type = interface_type;
    dev->i2c_address = (address != 0U) ? address : ZEDF9P_I2C_ADDR;
    dev->io = io;
    dev->measurement_rate_ms = 1000U;
    dev->navigation_rate = 1U;
    dev->dynamic_model = DYN_MODEL_STATIONARY;

    // Initialize parsing state
    dev->rx_buffer_idx = 0U;
    dev->current_message.valid = false;
    dev->parse_state = UBX_STATE_SYNC1;
    dev->ubx_7f_check_disabled = false;

    // Initialize data validity flags
    dev->nav_pvt_valid = false;
    dev->nav_hpposllh_valid = false;
    dev->rawx_valid = false;
    dev->mon_ver_valid = false;
    dev->nav_clock_valid = false;
    dev->nav_hpposecef_valid = false;
    dev->nav_timeutc_valid = false;
    dev->sfrbx_valid = false;

    // Initialize pending message tracking
    dev->pending_msg.msg_class = 0;
    dev->pending_msg.msg_id = 0;
    dev->pending_msg.waiting_for_ack = false;
    dev->pending_msg.waiting_for_response = false;

    dev->initialized = true;

    // Flush any existing data in the input buffer
    zedf9p_flush_input_buffer(dev);

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_soft_reset(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    const uint8_t payload[] = {0x00, 0x00, 0x02, 0x00};  // Software reset, controlled
    return zedf9p_send_ubx_message(dev, UBX_CLASS_CFG, UBX_CFG_RST,
                                           payload, sizeof(payload));
}

zedf9p_status_t zedf9p_hard_reset(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    const uint8_t payload[] = {0x00, 0x00, 0x01, 0x00};  // Hardware reset, controlled
    return zedf9p_send_ubx_message(dev, UBX_CLASS_CFG, UBX_CFG_RST,
                                           payload, sizeof(payload));
}

zedf9p_status_t zedf9p_set_measurement_rate(zedf9p_t *dev, const uint8_t layer_mask, const uint16_t meas_rate_ms, const uint16_t nav_rate) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    if (meas_rate_ms == 0U || nav_rate == 0U) {
        return ZEDF9P_ERR_INVALID_ARG;
    }

    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_RATE_MEAS, meas_rate_ms, 2U);
    if (status != ZEDF9P_OK) {
        return status;
    }

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_RATE_NAV, nav_rate, 2U);
    if (status == ZEDF9P_OK) {
        dev->measurement_rate_ms = meas_rate_ms;
        dev->navigation_rate = nav_rate;
    }

    return status;
}

zedf9p_status_t zedf9p_set_dynamic_model(zedf9p_t *dev, const uint8_t layer_mask, const uint8_t model) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    if (model > DYN_MODEL_BIKE) {
        return ZEDF9P_ERR_INVALID_ARG;
    }

    const zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_NAVSPG_DYNMODEL, model, 1U);
    if (status == ZEDF9P_OK) {
        dev->dynamic_model = model;
    }

    return status;
}

zedf9p_status_t zedf9p_set_message_rate(const zedf9p_t *dev, const uint8_t msg_class, const uint8_t msg_id, const uint8_t rate) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Map UBX message class/ID to CFG-VALSET key
    uint32_t config_key = 0;

    if (msg_class == UBX_CLASS_NAV) {
        switch (msg_id) {
            case UBX_NAV_CLOCK:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_I2C : UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_UART1; // I2C : UART1
                break;
            case UBX_NAV_HPPOSECEF:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_I2C : UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1; // I2C : UART1
                break;
            case UBX_NAV_HPPOSLLH:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_I2C : UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1;
                break;
            case UBX_NAV_PVT:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_NAV_PVT_I2C : UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1;
                break;
            case UBX_NAV_RELPOSNED:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_I2C : UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1;
                break;
            case UBX_NAV_SAT:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_NAV_SAT_I2C : UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART1;
                break;
            case UBX_NAV_TIMEUTC:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_I2C : UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1; // I2C : UART1
                break;
            default:
                return ZEDF9P_ERR_INVALID_ARG; // Unsupported NAV message
        }
    } else if (msg_class == UBX_CLASS_RXM) {
        switch (msg_id) {
            case UBX_RXM_RAWX:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_I2C : UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_UART1;
                break;
            case UBX_RXM_MEASX:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_I2C : UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_UART1;
                break;
            case UBX_RXM_SFRBX:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_I2C : UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_UART1;
                break;
            default:
                return ZEDF9P_ERR_INVALID_ARG; // Unsupported RXM message
        }
    } else if (msg_class == UBX_CLASS_MON) {
        switch (msg_id) {
            case UBX_MON_RF:
                config_key = (dev->interface_type == ZEDF9P_INTERFACE_I2C) ? UBLOX_CFG_MSGOUT_UBX_MON_RF_I2C : UBLOX_CFG_MSGOUT_UBX_MON_RF_UART1;
                break;
            default:
                return ZEDF9P_ERR_INVALID_ARG; // Unsupported MON message
        }
    } else {
        return ZEDF9P_ERR_INVALID_ARG; // Unsupported message class
    }

    // If we couldn't find a mapping, return error
    if (config_key == 0) {
        return ZEDF9P_ERR_INVALID_ARG;
    }

    // Use CFG-VALSET to configure the message rate
    return zedf9p_config_set_val(dev, UBLOX_CFG_LAYER_RAM, config_key, rate, 1U);
}

zedf9p_status_t zedf9p_config_set_val(const zedf9p_t *dev, const uint8_t layer_mask, const uint32_t key_id, const uint64_t value, const uint8_t size) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    if (size != 1U && size != 2U && size != 4U && size != 8U) {
        return ZEDF9P_ERR_INVALID_ARG;
    }

    uint8_t payload[12];  // Version(1) + Layer(1) + Reserved(2) + KeyID(4) + Value(up to 8)

    payload[0] = 0x00;          // Version
    payload[1] = layer_mask;    // Layer: RAM
    payload[2] = 0x00;          // Reserved
    payload[3] = 0x00;          // Reserved

    // Key ID (little endian)
    payload[4] = (uint8_t)(key_id & 0xFFU);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFFU);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFFU);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFFU);

    // Value (little endian)
    for (uint8_t i = 0; i < size; i++) {
        payload[8 + i] = (uint8_t)((value >> (i * 8)) & 0xFFU);
    }

    return zedf9p_send_ubx_message(dev, UBX_CLASS_CFG, UBX_CFG_VALSET,
                                           payload, 8U + size);
}

zedf9p_status_t zedf9p_config_get_val(zedf9p_t *dev, const uint8_t layer_mask, const uint32_t key_id, uint64_t *value, const uint8_t size) {
    if (dev == NULL || !dev->initialized || value == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    if (size != 1U && size != 2U && size != 4U && size != 8U) {
        return ZEDF9P_ERR_INVALID_ARG;
    }

    uint8_t payload[8];  // Version(1) + Layer(1) + Position(2) + KeyID(4)

    payload[0] = 0x00;          // Version
    payload[1] = layer_mask;    // Layer: RAM
    payload[2] = 0x00;          // Position
    payload[3] = 0x00;          // Position

    // Key ID (little endian)
    payload[4] = (uint8_t)(key_id & 0xFFU);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFFU);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFFU);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFFU);

    // Send CFG-VALGET request and wait for response
    ubx_message_t response;
    const zedf9p_status_t status = zedf9p_send_ubx_message_with_response(dev, UBX_CLASS_CFG, UBX_CFG_VALGET,
                                                                  payload, sizeof(payload),
                                                                  UBX_CLASS_CFG, UBX_CFG_VALGET,
                                                                  UBX_POLL_TIMEOUT_MS, &response);

    if (status != ZEDF9P_OK) {
        return status;
    }

    // Parse the response to extract the value
    if (response.length < (12U + size)) {  // Header + KeyID + Value
        return ZEDF9P_ERR_INVALID_MESSAGE;
    }

    // Verify the key ID in the response matches our request
    const uint32_t response_key = (uint32_t)response.payload[8] |
                           ((uint32_t)response.payload[9] << 8) |
                           ((uint32_t)response.payload[10] << 16) |
                           ((uint32_t)response.payload[11] << 24);

    if (response_key != key_id) {
        return ZEDF9P_ERR_INVALID_MESSAGE;
    }

    // Extract the value (little endian)
    *value = 0;
    for (uint8_t i = 0; i < size; i++) {
        *value |= ((uint64_t)response.payload[12 + i] << (i * 8));
    }

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_process_data(zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    uint8_t buffer[64];  // Process data in chunks
    uint16_t bytes_read = 0;

    // Read available data
    zedf9p_status_t status = zedf9p_read_available_data(dev, buffer, sizeof(buffer), &bytes_read);
    if (status != ZEDF9P_OK) {
        return (status == ZEDF9P_ERR_NO_DATA) ? ZEDF9P_OK : status;  // No data is not an error
    }

    // Process each byte
    for (uint16_t i = 0; i < bytes_read; i++) {
        status = zedf9p_parse_ubx_byte(dev, buffer[i]);
        if (status != ZEDF9P_OK && status != ZEDF9P_ERR_NO_DATA) {
            return status;
        }
    }

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_get_pvt(zedf9p_t *dev, zedf9p_nav_pvt_t *pvt) {
    if (dev == NULL || !dev->initialized || pvt == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    if (!dev->nav_pvt_valid) {
        return ZEDF9P_ERR_NO_DATA;
    }

    *pvt = dev->nav_pvt;
    dev->nav_pvt_valid = false;  // Mark as consumed

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_get_hpposllh(zedf9p_t *dev, zedf9p_nav_hpposllh_t *hppos) {
    if (dev == NULL || !dev->initialized || hppos == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    if (!dev->nav_hpposllh_valid) {
        return ZEDF9P_ERR_NO_DATA;
    }

    *hppos = dev->nav_hpposllh;
    dev->nav_hpposllh_valid = false;  // Mark as consumed

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_get_rawx(zedf9p_t *dev, zedf9p_rawx_t *rawx) {
    if (dev == NULL || !dev->initialized || rawx == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    if (!dev->rawx_valid) {
        return ZEDF9P_ERR_NO_DATA;
    }

    *rawx = dev->rawx;
    dev->rawx_valid = false;  // Mark as consumed

    return ZEDF9P_OK;
}

bool zedf9p_is_pvt_available(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    return dev->nav_pvt_valid;
}

bool zedf9p_is_hpposllh_available(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    return dev->nav_hpposllh_valid;
}

bool zedf9p_is_rawx_available(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    return dev->rawx_valid;
}

zedf9p_status_t zedf9p_register_pvt_callback(zedf9p_t *dev, const zedf9p_message_callback_t callback, void *user_data) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    dev->nav_pvt_callback = callback;
    dev->callback_user_data = user_data;

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_register_hpposllh_callback(zedf9p_t *dev, const zedf9p_message_callback_t callback, void *user_data) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    dev->nav_hpposllh_callback = callback;
    dev->callback_user_data = user_data;

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_register_rawx_callback(zedf9p_t *dev, const zedf9p_message_callback_t callback, void *user_data) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    dev->rawx_callback = callback;
    dev->callback_user_data = user_data;

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_register_generic_callback(zedf9p_t *dev, const zedf9p_message_callback_t callback, void *user_data) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    dev->generic_callback = callback;
    dev->callback_user_data = user_data;

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_send_ubx_message(const zedf9p_t *dev, const uint8_t msg_class, const uint8_t msg_id,
                                       const uint8_t *payload, const uint16_t payload_len) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    if (payload_len > UBX_MAX_PAYLOAD_SIZE) {
        return ZEDF9P_ERR_INVALID_ARG;
    }

    uint8_t message[UBX_MAX_PAYLOAD_SIZE + UBX_HEADER_SIZE + UBX_CHECKSUM_SIZE];
    uint16_t msg_idx = 0;

    // UBX header
    message[msg_idx++] = UBX_SYNC_CHAR_1;
    message[msg_idx++] = UBX_SYNC_CHAR_2;
    message[msg_idx++] = msg_class;
    message[msg_idx++] = msg_id;
    message[msg_idx++] = (uint8_t)(payload_len & 0xFFU);
    message[msg_idx++] = (uint8_t)((payload_len >> 8) & 0xFFU);

    // Payload
    if (payload != NULL && payload_len > 0U) {
        memcpy(&message[msg_idx], payload, payload_len);
        msg_idx += payload_len;
    }

    // Calculate checksum
    uint8_t ck_a, ck_b;
    zedf9p_calculate_checksum(&message[2], msg_idx - 2U, &ck_a, &ck_b);

    message[msg_idx++] = ck_a;
    message[msg_idx++] = ck_b;

    return zedf9p_write_data(dev, message, msg_idx);
}

zedf9p_status_t zedf9p_send_ubx_message_with_ack(zedf9p_t *dev, const uint8_t msg_class, const uint8_t msg_id,
                                                const uint8_t *payload, const uint16_t payload_len, const uint32_t timeout_ms) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Send message and wait for ACK/NAK
    ubx_message_t response;
    return zedf9p_send_ubx_message_with_response(dev, msg_class, msg_id, payload, payload_len,
                                               UBX_CLASS_ACK, UBX_ACK_ACK, timeout_ms, &response);
}

zedf9p_status_t zedf9p_send_ubx_message_with_response(zedf9p_t *dev, const uint8_t send_class, const uint8_t send_id,
                                                     const uint8_t *payload, const uint16_t payload_len,
                                                     const uint8_t expected_class, const uint8_t expected_id,
                                                     const uint32_t timeout_ms, ubx_message_t *response) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    uint8_t retry_count = 0;

    while (retry_count < UBX_MAX_RETRIES) {
        // Clear any existing data
        zedf9p_flush_input_buffer(dev);

        // Send the message
        zedf9p_status_t status = zedf9p_send_ubx_message(dev, send_class, send_id, payload, payload_len);
        if (status != ZEDF9P_OK) {
            retry_count++;
            dev->io.delay_ms(10);  // Short delay before retry
            continue;
        }

        // Wait for response
        const bool wait_for_ack = (expected_class == UBX_CLASS_ACK);
        status = zedf9p_wait_for_message_response(dev, expected_class, expected_id,
                                                timeout_ms, wait_for_ack, response);

        if (status == ZEDF9P_OK) {
            return ZEDF9P_OK;
        }

        if (status != ZEDF9P_ERR_TIMEOUT) {
            return status;  // Non-timeout errors are not retried
        }

        retry_count++;
        dev->io.delay_ms(50);  // Delay before retry
    }

    return ZEDF9P_ERR_TIMEOUT;
}

zedf9p_status_t zedf9p_poll_ubx_message(zedf9p_t *dev, const uint8_t msg_class, const uint8_t msg_id,
                                        const uint32_t timeout_ms, ubx_message_t *response) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Send poll request and wait for response
    return zedf9p_send_ubx_message_with_response(dev, msg_class, msg_id, NULL, 0,
                                               msg_class, msg_id, timeout_ms, response);
}

void zedf9p_calculate_checksum(const uint8_t *data, const uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    if (data == NULL || ck_a == NULL || ck_b == NULL) {
        return;
    }

    *ck_a = 0U;
    *ck_b = 0U;

    for (uint16_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

bool zedf9p_validate_checksum(const uint8_t *data, const uint16_t len, const uint8_t ck_a, const uint8_t ck_b) {
    if (data == NULL) {
        return false;
    }

    uint8_t calc_ck_a, calc_ck_b;
    zedf9p_calculate_checksum(data, len, &calc_ck_a, &calc_ck_b);

    return (calc_ck_a == ck_a) && (calc_ck_b == ck_b);
}

// Internal function implementations

static zedf9p_status_t zedf9p_write_data(const zedf9p_t *dev, const uint8_t *data, const uint16_t length) {
    if (dev == NULL || data == NULL || length == 0U) {
        return ZEDF9P_ERR_NULL;
    }

    int result;
    if (dev->interface_type == ZEDF9P_INTERFACE_I2C) {
        result = dev->io.i2c_write(dev->i2c_address, data, length);
        return (result == 0) ? ZEDF9P_OK : ZEDF9P_ERR_I2C;
    } else if (dev->interface_type == ZEDF9P_INTERFACE_UART) {
        result = dev->io.uart_write(data, length);
        return (result == 0) ? ZEDF9P_OK : ZEDF9P_ERR_UART;
    }

    return ZEDF9P_ERR_INVALID_ARG;
}

/*static zedf9p_status_t zedf9p_read_data(const zedf9p_t *dev, uint8_t *data, const uint16_t length) {
    if (dev == NULL || data == NULL || length == 0U) {
        return ZEDF9P_ERR_NULL;
    }

    int result;
    if (dev->interface_type == ZEDF9P_INTERFACE_I2C) {
        result = dev->io.i2c_read(dev->i2c_address, data, length);
        if (result == 0) {
            return ZEDF9P_OK;
        }
        if (result == -1) {
            return ZEDF9P_ERR_NO_DATA;  // No data available
        }
        return ZEDF9P_ERR_I2C;
    } else if (dev->interface_type == ZEDF9P_INTERFACE_UART) {
        result = dev->io.uart_read(data, length);
        if (result >= 0) {
            return (result > 0) ? ZEDF9P_OK : ZEDF9P_ERR_NO_DATA;
        }
        return ZEDF9P_ERR_UART;
    }

    return ZEDF9P_ERR_INVALID_ARG;
}*/

static zedf9p_status_t zedf9p_read_available_data(const zedf9p_t *dev, uint8_t *data, uint16_t max_length, uint16_t *bytes_read) {
    if (dev == NULL || data == NULL || bytes_read == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    *bytes_read = 0;

    if (dev->interface_type == ZEDF9P_INTERFACE_I2C) {
        // For I2C, just try to read a small amount first
        const int result = dev->io.i2c_read(dev->i2c_address, data, 8); // Read only 8 bytes at a time
        if (result >= 0) {
            *bytes_read = result;
            return (result > 0) ? ZEDF9P_OK : ZEDF9P_ERR_NO_DATA;
        } else {
            return ZEDF9P_ERR_I2C;
        }
    } else if (dev->interface_type == ZEDF9P_INTERFACE_UART) {
        const int result = dev->io.uart_read(data, max_length);
        if (result >= 0) {
            *bytes_read = result;
            return (result > 0) ? ZEDF9P_OK : ZEDF9P_ERR_NO_DATA;
        } else {
            return ZEDF9P_ERR_UART;
        }
    }

    return ZEDF9P_ERR_INVALID_ARG;
}

/*static zedf9p_status_t zedf9p_check_i2c_bytes_available(const zedf9p_t *dev, uint16_t *bytes_available) {
    if (dev == NULL || bytes_available == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // The register address should be sent as a single 16-bit value, not two bytes
    const uint8_t reg_addr_bytes[2] = {0xFD, 0x00}; // Try this order instead
    uint8_t response[2] = {0};

    // Write register address
    int result = dev->io.i2c_write(dev->i2c_address, reg_addr_bytes, 2);
    if (result != 0) {
        return ZEDF9P_ERR_I2C;
    }

    // Small delay for register read
    dev->io.delay_ms(1);

    // Read bytes available count
    result = dev->io.i2c_read(dev->i2c_address, response, 2);
    if (result != 0) {
        return ZEDF9P_ERR_I2C;
    }

    *bytes_available = ((uint16_t)response[0] << 8) | response[1];
    return ZEDF9P_OK;
}*/

static zedf9p_status_t zedf9p_flush_input_buffer(const zedf9p_t *dev) {
    if (dev == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // For I2C, just try a few small reads and give up
    if (dev->interface_type == ZEDF9P_INTERFACE_I2C) {
        uint8_t dummy_buffer[8];
        for (int attempt = 0; attempt < 5; attempt++) {  // Only try 5 times
            const int result = dev->io.i2c_read(dev->i2c_address, dummy_buffer, sizeof(dummy_buffer));
            if (result <= 0) {
                break;  // No more data or error
            }
            dev->io.delay_ms(10);
        }
        return ZEDF9P_OK;
    }

    // UART version remains the same
    uint8_t dummy_buffer[64];
    uint16_t bytes_read;
    zedf9p_status_t status;
    int attempts = 0;

    do {
        status = zedf9p_read_available_data(dev, dummy_buffer, sizeof(dummy_buffer), &bytes_read);
        if (status == ZEDF9P_ERR_NO_DATA) {
            break;
        }
        dev->io.delay_ms(1);
        attempts++;
    } while (status == ZEDF9P_OK && bytes_read > 0 && attempts < 10);  // Limit attempts

    return ZEDF9P_OK;
}

static zedf9p_status_t zedf9p_parse_ubx_byte(zedf9p_t *dev, const uint8_t byte) {
    static uint32_t byte_count = 0;
    byte_count++;

    switch (dev->parse_state) {
        case UBX_STATE_SYNC1:
            if (byte == UBX_SYNC_CHAR_1) {
                dev->parse_state = UBX_STATE_SYNC2;
                dev->rx_buffer_idx = 0U;
                if (dev->io.debug_print) {
                    char debug_buf[64];
                    sprintf(debug_buf, "SYNC1->SYNC2 at byte %lu\n", byte_count);
                    dev->io.debug_print(debug_buf);
                }
            }
            break;

        case UBX_STATE_SYNC2:
            if (byte == UBX_SYNC_CHAR_2) {
                dev->parse_state = UBX_STATE_CLASS;
                if (dev->io.debug_print) {
                    char debug_buf[64];
                    sprintf(debug_buf, "SYNC2->CLASS at byte %lu\n", byte_count);
                    dev->io.debug_print(debug_buf);
                }
            } else {
                dev->parse_state = UBX_STATE_SYNC1;
                if (dev->io.debug_print) {
                    char debug_buf[64];
                    sprintf(debug_buf, "SYNC2 failed (0x%02X), back to SYNC1 at byte %lu\n", byte, byte_count);
                    dev->io.debug_print(debug_buf);
                }
            }
            break;

        case UBX_STATE_CLASS:
            dev->current_message.msg_class = byte;
            dev->calculated_ck_a = byte;
            dev->calculated_ck_b = byte;
            dev->parse_state = UBX_STATE_ID;
            if (dev->io.debug_print) {
                char debug_buf[64];
                sprintf(debug_buf, "Class: 0x%02X\n", byte);
                dev->io.debug_print(debug_buf);
            }
            break;

        case UBX_STATE_ID:
            dev->current_message.msg_id = byte;
            dev->calculated_ck_a += byte;
            dev->calculated_ck_b += dev->calculated_ck_a;
            dev->parse_state = UBX_STATE_LENGTH1;
            if (dev->io.debug_print) {
                char debug_buf[64];
                sprintf(debug_buf, "ID: 0x%02X\n", byte);
                dev->io.debug_print(debug_buf);
            }
            break;

        case UBX_STATE_LENGTH1:
            dev->expected_length = byte;
            dev->calculated_ck_a += byte;
            dev->calculated_ck_b += dev->calculated_ck_a;
            dev->parse_state = UBX_STATE_LENGTH2;
            if (dev->io.debug_print) {
                char debug_buf[64];
                sprintf(debug_buf, "Length LSB: 0x%02X\n", byte);
                dev->io.debug_print(debug_buf);
            }
            break;

        case UBX_STATE_LENGTH2:
            dev->expected_length |= ((uint16_t)byte << 8);
            dev->current_message.length = dev->expected_length;
            dev->calculated_ck_a += byte;
            dev->calculated_ck_b += dev->calculated_ck_a;

            if (dev->io.debug_print) {
                char debug_buf[128];
                sprintf(debug_buf, "Message: Class=0x%02X ID=0x%02X Length=%d (LSB=0x%02X MSB=0x%02X)\n",
                       dev->current_message.msg_class, dev->current_message.msg_id,
                       dev->expected_length, (uint8_t)(dev->expected_length & 0xFF), byte);
                dev->io.debug_print(debug_buf);
            }

            if (dev->expected_length == 0U) {
                dev->parse_state = UBX_STATE_CHECKSUM1;
                if (dev->io.debug_print) {
                    dev->io.debug_print("Zero length message, going to checksum\n");
                }
            } else if (dev->expected_length <= UBX_MAX_PAYLOAD_SIZE) {
                dev->bytes_remaining = dev->expected_length;
                dev->parse_state = UBX_STATE_PAYLOAD;
                if (dev->io.debug_print) {
                    char debug_buf[64];
                    sprintf(debug_buf, "Starting payload read: %d bytes remaining\n", dev->bytes_remaining);
                    dev->io.debug_print(debug_buf);
                }
            } else {
                if (dev->io.debug_print) {
                    char debug_buf[128];
                    sprintf(debug_buf, "ERROR: Invalid message length %d > MAX %d (Class=0x%02X ID=0x%02X)\n",
                           dev->expected_length, UBX_MAX_PAYLOAD_SIZE,
                           dev->current_message.msg_class, dev->current_message.msg_id);
                    dev->io.debug_print(debug_buf);
                }
                dev->parse_state = UBX_STATE_SYNC1;
                return ZEDF9P_ERR_INVALID_MESSAGE;
            }
            break;

        case UBX_STATE_PAYLOAD:
            if (dev->rx_buffer_idx < UBX_MAX_PAYLOAD_SIZE) {
                dev->current_message.payload[dev->rx_buffer_idx++] = byte;

                // Debug checksum calculation for first/last few bytes
                if (dev->rx_buffer_idx <= 5 || dev->bytes_remaining <= 5) {
                    const uint8_t old_ck_a = dev->calculated_ck_a;
                    const uint8_t old_ck_b = dev->calculated_ck_b;
                    dev->calculated_ck_a += byte;
                    dev->calculated_ck_b += dev->calculated_ck_a;

                    if (dev->io.debug_print) {
                        char debug_buf[128];
                        sprintf(debug_buf, "Byte[%d]=0x%02X: CK_A 0x%02X->0x%02X, CK_B 0x%02X->0x%02X\n",
                               dev->rx_buffer_idx-1, byte, old_ck_a, dev->calculated_ck_a,
                               old_ck_b, dev->calculated_ck_b);
                        dev->io.debug_print(debug_buf);
                    }
                } else {
                    dev->calculated_ck_a += byte;
                    dev->calculated_ck_b += dev->calculated_ck_a;
                }

                dev->bytes_remaining--;

                // Debug every 100 bytes or when payload is complete
                if (dev->io.debug_print && (dev->rx_buffer_idx % 100 == 0 || dev->bytes_remaining == 0)) {
                    char debug_buf[64];
                    sprintf(debug_buf, "Payload: %d/%d bytes (remaining: %d)\n",
                           dev->rx_buffer_idx, dev->expected_length, dev->bytes_remaining);
                    dev->io.debug_print(debug_buf);
                }

                if (dev->bytes_remaining == 0U) {
                    dev->parse_state = UBX_STATE_CHECKSUM1;
                    if (dev->io.debug_print) {
                        char debug_buf[64];
                        sprintf(debug_buf, "Payload complete. CK_A=0x%02X CK_B=0x%02X\n",
                               dev->calculated_ck_a, dev->calculated_ck_b);
                        dev->io.debug_print(debug_buf);
                    }
                }
            } else {
                if (dev->io.debug_print) {
                    char debug_buf[64];
                    sprintf(debug_buf, "ERROR: Buffer overflow at index %d\n", dev->rx_buffer_idx);
                    dev->io.debug_print(debug_buf);
                }
                dev->parse_state = UBX_STATE_SYNC1;
                return ZEDF9P_ERR_BUFFER_FULL;
            }
            break;

        case UBX_STATE_CHECKSUM1:
            zedf9p_verify_checksum_manual(dev, &dev->current_message);
            if (dev->io.debug_print) {
                char debug_buf[64];
                sprintf(debug_buf, "CK_A check: Expected=0x%02X Got=0x%02X\n", dev->calculated_ck_a, byte);
                dev->io.debug_print(debug_buf);
            }

            if (byte == dev->calculated_ck_a) {
                dev->parse_state = UBX_STATE_CHECKSUM2;
            } else {
                if (dev->io.debug_print) {
                    dev->io.debug_print("ERROR: CK_A mismatch\n");
                }
                dev->parse_state = UBX_STATE_SYNC1;
                return ZEDF9P_ERR_CRC;
            }
            break;

        case UBX_STATE_CHECKSUM2:
            if (dev->io.debug_print) {
                char debug_buf[64];
                sprintf(debug_buf, "CK_B check: Expected=0x%02X Got=0x%02X\n", dev->calculated_ck_b, byte);
                dev->io.debug_print(debug_buf);
            }

            dev->parse_state = UBX_STATE_SYNC1;

            if (byte == dev->calculated_ck_b) {
                dev->current_message.valid = true;
                dev->current_message.timestamp_ms = dev->io.get_millis();
                if (dev->io.debug_print) {
                    char debug_buf[128];
                    sprintf(debug_buf, "Message VALID: Class=0x%02X ID=0x%02X Length=%d - Processing...\n",
                           dev->current_message.msg_class, dev->current_message.msg_id, dev->current_message.length);
                    dev->io.debug_print(debug_buf);
                }
                return zedf9p_handle_message(dev, &dev->current_message);
            } else {
                if (dev->io.debug_print) {
                    dev->io.debug_print("ERROR: CK_B mismatch\n");
                }
                return ZEDF9P_ERR_CRC;
            }

        default:
            if (dev->io.debug_print) {
                char debug_buf[64];
                sprintf(debug_buf, "ERROR: Unknown parse state %d\n", dev->parse_state);
                dev->io.debug_print(debug_buf);
            }
            dev->parse_state = UBX_STATE_SYNC1;
            break;
    }

    return ZEDF9P_OK;
}

static zedf9p_status_t zedf9p_handle_message(zedf9p_t *dev, const ubx_message_t *message) {
    if (dev == NULL || message == NULL || !message->valid) {
        return ZEDF9P_ERR_NULL;
    }

    // Handle specific message types
    if (message->msg_class == UBX_CLASS_NAV) {
        if (message->msg_id == UBX_NAV_PVT && message->length >= sizeof(zedf9p_nav_pvt_t)) {
            memcpy(&dev->nav_pvt, message->payload, sizeof(zedf9p_nav_pvt_t));
            dev->nav_pvt_valid = true;

            if (dev->nav_pvt_callback != NULL) {
                dev->nav_pvt_callback(message, dev->callback_user_data);
            }
        } else if (message->msg_id == UBX_NAV_HPPOSLLH && message->length >= sizeof(zedf9p_nav_hpposllh_t)) {
            memcpy(&dev->nav_hpposllh, message->payload, sizeof(zedf9p_nav_hpposllh_t));
            dev->nav_hpposllh_valid = true;

            if (dev->nav_hpposllh_callback != NULL) {
                dev->nav_hpposllh_callback(message, dev->callback_user_data);
            }
        } else if (message->msg_id == UBX_NAV_CLOCK && message->length >= sizeof(zedf9p_nav_clock_t)) {
            memcpy(&dev->nav_clock, message->payload, sizeof(zedf9p_nav_clock_t));
            dev->nav_clock_valid = true;
            if (dev->nav_clock_callback != NULL) {
                dev->nav_clock_callback(message, dev->callback_user_data);
            }
        } else if (message->msg_id == UBX_NAV_HPPOSECEF && message->length >= sizeof(zedf9p_nav_hpposecef_t)) {
            memcpy(&dev->nav_hpposecef, message->payload, sizeof(zedf9p_nav_hpposecef_t));
            dev->nav_hpposecef_valid = true;
            if (dev->nav_hpposecef_callback != NULL) {
                dev->nav_hpposecef_callback(message, dev->callback_user_data);
            }
        } else if (message->msg_id == UBX_NAV_TIMEUTC && message->length >= sizeof(zedf9p_nav_timeutc_t)) {
            memcpy(&dev->nav_timeutc, message->payload, sizeof(zedf9p_nav_timeutc_t));
            dev->nav_timeutc_valid = true;
            if (dev->nav_timeutc_callback != NULL) {
                dev->nav_timeutc_callback(message, dev->callback_user_data);
            }
        }
    } else if (message->msg_class == UBX_CLASS_RXM) {
        if (message->msg_id == UBX_RXM_RAWX) {
            // RAWX handling code
            if (message->length >= 16U) {
                memcpy(&dev->rawx, message->payload, 16U);
                const uint8_t num_meas = dev->rawx.num_meas;
                const uint16_t expected_length = 16U + (num_meas * 32U);

                if (message->length == expected_length && num_meas <= 32U) {
                    if (num_meas > 0) {
                        memcpy(dev->rawx.meas, &message->payload[16], num_meas * 32U);
                    }
                    dev->rawx_valid = true;

                    // THIS IS THE MISSING PIECE:
                    if (dev->rawx_callback != NULL) {
                        dev->rawx_callback(message, dev->callback_user_data);
                    }
                }
            }
        } else if (message->msg_id == UBX_RXM_SFRBX && message->length >= sizeof(zedf9p_sfrbx_t)) {
            memcpy(&dev->sfrbx, message->payload, sizeof(zedf9p_sfrbx_t));
            dev->sfrbx_valid = true;
            if (dev->sfrbx_callback != NULL) {
                dev->sfrbx_callback(message, dev->callback_user_data);
            }
        }
    } else if (message->msg_class == UBX_CLASS_MON && message->msg_id == UBX_MON_VER) {
        // MON-VER message parsing
        if (message->length >= 40U) {  // Minimum MON-VER size (swVersion + hwVersion)
            // Clear the structure
            memset(&dev->mon_ver, 0, sizeof(zedf9p_mon_ver_t));

            // Extract software version (30 bytes, null-terminated string)
            strncpy(dev->mon_ver.sw_version, (char *)&message->payload[0], 29);
            dev->mon_ver.sw_version[29] = '\0';  // Ensure null termination

            // Extract hardware version (10 bytes, null-terminated string)
            strncpy(dev->mon_ver.hw_version, (char *)&message->payload[30], 9);
            dev->mon_ver.hw_version[9] = '\0';  // Ensure null termination

            // Parse extensions if present (each extension is 30 bytes)
            if (message->length > 40U) {
                const uint16_t extensions_len = message->length - 40U;
                const uint8_t num_extensions = (uint8_t)(extensions_len / 30U);  // Each extension is 30 bytes

                dev->mon_ver.num_extensions = (num_extensions > 30U) ? 30U : num_extensions;

                for (uint8_t i = 0; i < dev->mon_ver.num_extensions; i++) {
                    const uint16_t ext_offset = 40U + (i * 30U);
                    strncpy(dev->mon_ver.extensions[i], (char *)&message->payload[ext_offset], 29);
                    dev->mon_ver.extensions[i][29] = '\0';  // Ensure null termination

                    // Check if this extension contains ROM version info
                    if (strncmp(dev->mon_ver.extensions[i], "ROM BASE", 8) == 0) {
                        strncpy(dev->mon_ver.rom_version, dev->mon_ver.extensions[i], 29);
                        dev->mon_ver.rom_version[29] = '\0';
                    }
                }
            }

            dev->mon_ver.valid = true;
            dev->mon_ver_valid = true;
        }
    }

    // Check if this message matches a pending request
    bool is_expected_response = false;

    if (dev->pending_msg.waiting_for_response) {
        if (dev->pending_msg.expected_class == UBX_CLASS_ACK) {
            // For ACK messages, check the payload to see what command was ACKed
            if (message->msg_class == UBX_CLASS_ACK && message->length >= 2) {
                const uint8_t ack_class = message->payload[0];
                const uint8_t ack_id = message->payload[1];
                if (ack_class == dev->pending_msg.msg_class && ack_id == dev->pending_msg.msg_id) {
                    is_expected_response = true;
                }
            }
        } else {
            // For non-ACK messages, use the original logic
            if (message->msg_class == dev->pending_msg.expected_class &&
                (dev->pending_msg.expected_id == 0xFF || message->msg_id == dev->pending_msg.expected_id)) {
                is_expected_response = true;
                }
        }
    }

    if (is_expected_response) {
        // Copy the message to the response buffer
        dev->pending_response = *message;
        dev->pending_msg.waiting_for_response = false;
        dev->pending_msg.response_received = true;
    }

    // Handle ACK/NAK messages
    if (message->msg_class == UBX_CLASS_ACK && message->length >= 2U && dev->pending_msg.waiting_for_ack) {
        const uint8_t ack_class = message->payload[0];
        const uint8_t ack_id = message->payload[1];

        if (ack_class == dev->pending_msg.msg_class && ack_id == dev->pending_msg.msg_id) {
            dev->pending_msg.waiting_for_ack = false;
            dev->pending_msg.ack_received = true;
            dev->pending_msg.is_ack = (message->msg_id == UBX_ACK_ACK);
        }
    }

    // Call generic callback for all messages
    if (dev->generic_callback != NULL) {
        dev->generic_callback(message, dev->callback_user_data);
    }

    return ZEDF9P_OK;
}

static zedf9p_status_t zedf9p_wait_for_message_response(zedf9p_t *dev, const uint8_t msg_class,
                                                       const uint8_t msg_id, const uint32_t timeout_ms,
                                                       const bool wait_for_ack, ubx_message_t *response) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Set up pending message tracking
    dev->pending_msg.expected_class = msg_class;
    dev->pending_msg.expected_id = msg_id;
    dev->pending_msg.timestamp_ms = dev->io.get_millis();
    dev->pending_msg.timeout_ms = timeout_ms;
    dev->pending_msg.waiting_for_ack = wait_for_ack;
    dev->pending_msg.waiting_for_response = !wait_for_ack;
    dev->pending_msg.ack_received = false;
    dev->pending_msg.response_received = false;
    dev->pending_msg.is_ack = false;

    const uint32_t start_time = dev->io.get_millis();

    while ((dev->io.get_millis() - start_time) < timeout_ms) {
        // Process incoming data
        const zedf9p_status_t status = zedf9p_process_data(dev);
        if (status != ZEDF9P_OK && status != ZEDF9P_ERR_NO_DATA) {
            return status;
        }

        // Check if we received what we were waiting for
        if (wait_for_ack && dev->pending_msg.ack_received) {
            if (response != NULL) {
                // For ACK/NAK, create a simple response message
                response->msg_class = UBX_CLASS_ACK;
                response->msg_id = dev->pending_msg.is_ack ? UBX_ACK_ACK : UBX_ACK_NAK;
                response->length = 2;
                response->payload[0] = dev->pending_msg.msg_class;
                response->payload[1] = dev->pending_msg.msg_id;
                response->valid = true;
                response->timestamp_ms = dev->io.get_millis();
            }
            return dev->pending_msg.is_ack ? ZEDF9P_OK : ZEDF9P_ERR_NACK;
        }

        if (!wait_for_ack && dev->pending_msg.response_received) {
            if (response != NULL) {
                *response = dev->pending_response;
            }
            return ZEDF9P_OK;
        }

        dev->io.delay_ms(1);  // Small delay to prevent busy waiting
    }

    // Reset pending message state
    dev->pending_msg.waiting_for_ack = false;
    dev->pending_msg.waiting_for_response = false;

    return ZEDF9P_ERR_TIMEOUT;
}

// Hardware Configuration Function Implementations

zedf9p_status_t zedf9p_config_uart(const zedf9p_t *dev, const uint8_t layer_mask, const uint8_t port, const zedf9p_uart_config_t *config) {
    if (dev == NULL || !dev->initialized || config == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    if (port != 1U && port != 2U) {
        return ZEDF9P_ERR_INVALID_ARG;
    }

    const uint32_t base_key = (port == 1U) ? 0x40520000U : 0x40530000U;

    // Configure baudrate
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, base_key + 1U, config->baudrate, 4U);
    if (status != ZEDF9P_OK) return status;

    // Configure stop bits
    status = zedf9p_config_set_val(dev, layer_mask, base_key + 2U, config->stopbits, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure data bits
    status = zedf9p_config_set_val(dev, layer_mask, base_key + 3U, config->databits, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure parity
    status = zedf9p_config_set_val(dev, layer_mask, base_key + 4U, config->parity, 1U);
    if (status != ZEDF9P_OK) return status;

    // Enable/disable UART
    status = zedf9p_config_set_val(dev, layer_mask, base_key + 5U, config->enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure remap
    return zedf9p_config_set_val(dev, layer_mask, base_key + 6U, config->remap ? 1U : 0U, 1U);
}

zedf9p_status_t zedf9p_config_i2c(const zedf9p_t *dev, const uint8_t layer_mask, const zedf9p_i2c_config_t *config) {
    if (dev == NULL || !dev->initialized || config == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // Set I2C address
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_I2C_ADDRESS, config->address, 1U);
    if (status != ZEDF9P_OK) return status;

    // Set extended timeout
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_I2C_EXTENDEDTIMEOUT, config->extended_timeout ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Enable/disable I2C
    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_I2C_ENABLED, config->enabled ? 1U : 0U, 1U);
}

zedf9p_status_t zedf9p_config_gnss_signals(const zedf9p_t *dev, const uint8_t layer_mask, const zedf9p_gnss_config_t *config) {
    if (dev == NULL || !dev->initialized || config == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // GPS configuration
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_GPS_ENA, config->gps_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_GPS_L1CA_ENA, config->gps_l1ca_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_GPS_L2C_ENA, config->gps_l2c_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // SBAS configuration
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_SBAS_ENA, config->sbas_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_SBAS_L1CA_ENA, config->sbas_l1ca_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Galileo configuration
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_GAL_ENA, config->galileo_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_GAL_E1_ENA, config->galileo_e1_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_GAL_E5B_ENA, config->galileo_e5b_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // BeiDou configuration
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_BDS_ENA, config->beidou_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_BDS_B1_ENA, config->beidou_b1_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_BDS_B2_ENA, config->beidou_b2_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // QZSS configuration
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_QZSS_ENA, config->qzss_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_QZSS_L1CA_ENA, config->qzss_l1ca_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_QZSS_L1S_ENA, config->qzss_l1s_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_QZSS_L2C_ENA, config->qzss_l2c_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // GLONASS configuration
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_GLO_ENA, config->glonass_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_GLO_L1_ENA, config->glonass_l1_enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SIGNAL_GLO_L2_ENA, config->glonass_l2_enabled ? 1U : 0U, 1U);
}

zedf9p_status_t zedf9p_config_sbas(const zedf9p_t *dev, const uint8_t layer_mask, const zedf9p_sbas_config_t *config) {
    if (dev == NULL || !dev->initialized || config == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // Configure SBAS test mode
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SBAS_USE_TESTMODE, config->use_testmode ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure SBAS ranging
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SBAS_USE_RANGING, config->use_ranging ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure SBAS differential corrections
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SBAS_USE_DIFFCORR, config->use_diffcorr ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure SBAS integrity
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SBAS_USE_INTEGRITY, config->use_integrity ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure PRN scan mask
    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SBAS_PRNSCANMASK, config->prn_scan_mask, 8U);
}

zedf9p_status_t zedf9p_config_time_mode(const zedf9p_t *dev, const uint8_t layer_mask, const zedf9p_tmode_config_t *config) {
    if (dev == NULL || !dev->initialized || config == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // Set time mode
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_MODE, config->mode, 1U);
    if (status != ZEDF9P_OK) return status;

    // Set position type
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_POS_TYPE, config->pos_type, 1U);
    if (status != ZEDF9P_OK) return status;

    if (config->pos_type == POS_TYPE_ECEF) {
        // Configure ECEF coordinates
        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_X, (uint64_t)config->ecef_x, 4U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_Y, (uint64_t)config->ecef_y, 4U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_Z, (uint64_t)config->ecef_z, 4U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_X_HP, config->ecef_x_hp, 1U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_Y_HP, config->ecef_y_hp, 1U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_Z_HP, config->ecef_z_hp, 1U);
        if (status != ZEDF9P_OK) return status;
    } else {
        // Configure LLH coordinates
        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_LAT, (uint64_t)config->lat, 4U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_LON, (uint64_t)config->lon, 4U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_HEIGHT, (uint64_t)config->height, 4U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_LAT_HP, config->lat_hp, 1U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_LON_HP, config->lon_hp, 1U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_HEIGHT_HP, config->height_hp, 1U);
        if (status != ZEDF9P_OK) return status;
    }

    // Configure accuracy and survey-in parameters
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_FIXED_POS_ACC, config->fixed_pos_acc, 4U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_SVIN_MIN_DUR, config->svin_min_dur, 4U);
    if (status != ZEDF9P_OK) return status;

    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_SVIN_ACC_LIMIT, config->svin_acc_limit, 4U);
}

zedf9p_status_t zedf9p_config_time_pulse(const zedf9p_t *dev, const uint8_t layer_mask, const zedf9p_timepulse_config_t *config) {
    if (dev == NULL || !dev->initialized || config == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // Configure antenna cable delay
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_ANT_CABLEDELAY, (uint64_t)config->ant_cable_delay_ns, 2U);
    if (status != ZEDF9P_OK) return status;

    // Configure period/frequency
    if (config->period_us > 0U) {
        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_PERIOD_TP1, config->period_us, 4U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_PERIOD_LOCK_TP1, config->period_lock_us, 4U);
        if (status != ZEDF9P_OK) return status;
    }

    if (config->freq_hz > 0U) {
        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_FREQ_TP1, config->freq_hz, 4U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_FREQ_LOCK_TP1, config->freq_lock_hz, 4U);
        if (status != ZEDF9P_OK) return status;
    }

    // Configure pulse length
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_LEN_TP1, config->length_us, 4U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_LEN_LOCK_TP1, config->length_lock_us, 4U);
    if (status != ZEDF9P_OK) return status;

    // Configure duty cycle (convert to fixed point)
    const uint64_t duty_fixed = (uint64_t)(config->duty_percent * 4294967296.0 / 100.0);
    const uint64_t duty_lock_fixed = (uint64_t)(config->duty_lock_percent * 4294967296.0 / 100.0);

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_DUTY_TP1, duty_fixed, 8U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_DUTY_LOCK_TP1, duty_lock_fixed, 8U);
    if (status != ZEDF9P_OK) return status;

    // Configure user delay
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_USER_DELAY_TP1, (uint64_t)config->user_delay_ns, 4U);
    if (status != ZEDF9P_OK) return status;

    // Configure flags
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_TP1_ENA, config->enabled ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_SYNC_GNSS_TP1, config->sync_gnss ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_USE_LOCKED_TP1, config->use_locked ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_ALIGN_TO_TOW_TP1, config->align_to_tow ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_POL_TP1, config->polarity, 1U);
    if (status != ZEDF9P_OK) return status;

    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TP_TIMEGRID_TP1, config->time_grid, 1U);
}

zedf9p_status_t zedf9p_config_power_management(const zedf9p_t *dev, const uint8_t layer_mask, const zedf9p_power_config_t *config) {
    if (dev == NULL || !dev->initialized || config == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // Configure operate mode
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_PM_OPERATEMODE, config->operate_mode, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure position update period
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_PM_POSUPDATEPERIOD, config->pos_update_period_ms, 4U);
    if (status != ZEDF9P_OK) return status;

    // Configure acquisition period
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_PM_ACQPERIOD, config->acq_period_ms, 4U);
    if (status != ZEDF9P_OK) return status;

    // Configure grid offset
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_PM_GRIDOFFSET, config->grid_offset_ms, 4U);
    if (status != ZEDF9P_OK) return status;

    // Configure on time
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_PM_ONTIME, config->on_time_ms, 4U);
    if (status != ZEDF9P_OK) return status;

    // Configure minimum acquisition time
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_PM_MINACQTIME, config->min_acq_time_ms, 4U);
    if (status != ZEDF9P_OK) return status;

    // Configure maximum acquisition time
    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_PM_MAXACQTIME, config->max_acq_time_ms, 4U);
}

zedf9p_status_t zedf9p_config_antenna(const zedf9p_t *dev, const uint8_t layer_mask, const zedf9p_antenna_config_t *config) {
    if (dev == NULL || !dev->initialized || config == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    // Configure voltage control
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_CFG_VOLTCTRL, config->voltage_ctrl ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure short detection
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_CFG_SHORTDET, config->short_det ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_CFG_SHORTDET_POL, config->short_det_pol ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure open detection
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_CFG_OPENDET, config->open_det ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_CFG_OPENDET_POL, config->open_det_pol ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure power down
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_CFG_PWRDOWN, config->power_down ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_CFG_PWRDOWN_POL, config->power_down_pol ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure recovery
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_CFG_RECOVER, config->recover ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure switch pin
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_SUP_SWITCH_PIN, config->switch_pin, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure thresholds
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_SUP_SHORT_THR, config->short_thr, 1U);
    if (status != ZEDF9P_OK) return status;

    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_HW_ANT_SUP_OPEN_THR, config->open_thr, 1U);
}

zedf9p_status_t zedf9p_config_interference_mitigation(const zedf9p_t *dev, const uint8_t layer_mask, const bool enable, const uint8_t ant_setting, const bool enable_aux) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Enable/disable interference mitigation
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_ITFM_ENABLE, enable ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    // Configure antenna setting
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_ITFM_ANTSETTING, ant_setting, 1U);
    if (status != ZEDF9P_OK) return status;

    // Enable/disable auxiliary interference mitigation
    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_ITFM_ENABLE_AUX, enable_aux ? 1U : 0U, 1U);
}

zedf9p_status_t zedf9p_config_jamming_monitor(const zedf9p_t *dev, const uint8_t layer_mask, const bool enable) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_JAMMING_MONITOR, enable ? 1U : 0U, 1U);
}

zedf9p_status_t zedf9p_enable_rtcm_message(const zedf9p_t *dev, const uint8_t layer_mask, const uint16_t message_type, const uint8_t rate, const uint8_t interface_mask) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    zedf9p_status_t status = ZEDF9P_OK;

    // Map message type to configuration keys
    uint32_t i2c_key = 0U, uart1_key = 0U, uart2_key = 0U;

    switch (message_type) {
        case 1005U:
            i2c_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C;
            uart1_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1;
            uart2_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART2;
            break;
        case 1077U:
            i2c_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_I2C;
            uart1_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1;
            uart2_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART2;
            break;
        case 1087U:
            i2c_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_I2C;
            uart1_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1;
            uart2_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART2;
            break;
        case 1097U:
            i2c_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_I2C;
            uart1_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART1;
            uart2_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART2;
            break;
        case 1127U:
            i2c_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_I2C;
            uart1_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART1;
            uart2_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART2;
            break;
        case 1230U:
            i2c_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C;
            uart1_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1;
            uart2_key = UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART2;
            break;
        default:
            return ZEDF9P_ERR_INVALID_ARG;
    }

    // Configure message rate for selected interfaces
    if ((interface_mask & 0x01U) != 0U) {  // I2C
        status = zedf9p_config_set_val(dev, layer_mask, i2c_key, rate, 1U);
        if (status != ZEDF9P_OK) return status;
    }

    if ((interface_mask & 0x02U) != 0U) {  // UART1
        status = zedf9p_config_set_val(dev, layer_mask, uart1_key, rate, 1U);
        if (status != ZEDF9P_OK) return status;
    }

    if ((interface_mask & 0x04U) != 0U) {  // UART2
        status = zedf9p_config_set_val(dev, layer_mask, uart2_key, rate, 1U);
        if (status != ZEDF9P_OK) return status;
    }

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_config_rtcm_base_station(const zedf9p_t *dev, const uint8_t layer_mask, const bool enable_1005, const bool enable_1077,
                                               const bool enable_1087, const bool enable_1097,
                                               const bool enable_1127, const bool enable_1230) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    zedf9p_status_t status;
    const uint8_t rate = 1U;  // 1Hz for all RTCM messages
    const uint8_t all_interfaces = 0x07U;  // I2C + UART1 + UART2

    // Configure each RTCM message type
    if (enable_1005) {
        status = zedf9p_enable_rtcm_message(dev, layer_mask, 1005U, rate, all_interfaces);
        if (status != ZEDF9P_OK) return status;
    }

    if (enable_1077) {
        status = zedf9p_enable_rtcm_message(dev, layer_mask, 1077U, rate, all_interfaces);
        if (status != ZEDF9P_OK) return status;
    }

    if (enable_1087) {
        status = zedf9p_enable_rtcm_message(dev, layer_mask, 1087U, rate, all_interfaces);
        if (status != ZEDF9P_OK) return status;
    }

    if (enable_1097) {
        status = zedf9p_enable_rtcm_message(dev, layer_mask, 1097U, rate, all_interfaces);
        if (status != ZEDF9P_OK) return status;
    }

    if (enable_1127) {
        status = zedf9p_enable_rtcm_message(dev, layer_mask, 1127U, rate, all_interfaces);
        if (status != ZEDF9P_OK) return status;
    }

    if (enable_1230) {
        status = zedf9p_enable_rtcm_message(dev, layer_mask, 1230U, rate, all_interfaces);
        if (status != ZEDF9P_OK) return status;
    }

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_config_spartn(const zedf9p_t *dev, const uint8_t layer_mask, const bool enable, const bool use_source) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Configure SPARTN as correction source
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_SPARTN_USE_SOURCE, use_source ? 1U : 0U, 1U);
    if (status != ZEDF9P_OK) return status;

    if (enable) {
        // Enable SPARTN message output on all interfaces
        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_MSGOUT_SPARTN_I2C, 1U, 1U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_MSGOUT_SPARTN_UART1, 1U, 1U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_MSGOUT_SPARTN_UART2, 1U, 1U);
        if (status != ZEDF9P_OK) return status;
    } else {
        // Disable SPARTN message output
        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_MSGOUT_SPARTN_I2C, 0U, 1U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_MSGOUT_SPARTN_UART1, 0U, 1U);
        if (status != ZEDF9P_OK) return status;

        status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_MSGOUT_SPARTN_UART2, 0U, 1U);
        if (status != ZEDF9P_OK) return status;
    }

    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_start_survey_in(const zedf9p_t *dev, const uint8_t layer_mask, const uint32_t min_duration_s, const uint32_t accuracy_limit_mm) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Configure survey-in parameters
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_SVIN_MIN_DUR, min_duration_s, 4U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_SVIN_ACC_LIMIT, accuracy_limit_mm, 4U);
    if (status != ZEDF9P_OK) return status;

    // Set time mode to survey-in
    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_MODE, TMODE_SURVEY_IN, 1U);
}

zedf9p_status_t zedf9p_stop_survey_in(const zedf9p_t *dev, const uint8_t layer_mask) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Disable time mode
    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_MODE, TMODE_DISABLED, 1U);
}

zedf9p_status_t zedf9p_set_fixed_base_position(const zedf9p_t *dev, const uint8_t layer_mask, const double lat_deg, const double lon_deg,
                                              const double height_m, const uint32_t accuracy_mm) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Convert coordinates to u-blox format
    const int32_t lat_scaled = (int32_t)(lat_deg * 1e7);
    const int32_t lon_scaled = (int32_t)(lon_deg * 1e7);
    const int32_t height_scaled = (int32_t)(height_m * 100.0);  // cm

    // Extract high precision parts
    const double lat_hp_deg = lat_deg - ((double)lat_scaled / 1e7);
    const double lon_hp_deg = lon_deg - ((double)lon_scaled / 1e7);
    const double height_hp_m = height_m - ((double)height_scaled / 100.0);

    const int8_t lat_hp = (int8_t)(lat_hp_deg * 1e9);
    const int8_t lon_hp = (int8_t)(lon_hp_deg * 1e9);
    const int8_t height_hp = (int8_t)(height_hp_m * 10.0);  // 0.1 mm

    // Set position type to LLH
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_POS_TYPE, POS_TYPE_LLH, 1U);
    if (status != ZEDF9P_OK) return status;

    // Set coordinates
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_LAT, (uint64_t)lat_scaled, 4U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_LON, (uint64_t)lon_scaled, 4U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_HEIGHT, (uint64_t)height_scaled, 4U);
    if (status != ZEDF9P_OK) return status;

    // Set high precision parts
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_LAT_HP, lat_hp, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_LON_HP, lon_hp, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_HEIGHT_HP, height_hp, 1U);
    if (status != ZEDF9P_OK) return status;

    // Set accuracy
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_FIXED_POS_ACC, accuracy_mm, 4U);
    if (status != ZEDF9P_OK) return status;

    // Set time mode to fixed
    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_MODE, TMODE_FIXED_MODE, 1U);
}

zedf9p_status_t zedf9p_set_fixed_base_position_ecef(const zedf9p_t *dev, const uint8_t layer_mask, const double x_m, const double y_m,
                                                   const double z_m, const uint32_t accuracy_mm) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    // Convert coordinates to u-blox format
    const int32_t x_scaled = (int32_t)(x_m * 100.0);  // cm
    const int32_t y_scaled = (int32_t)(y_m * 100.0);  // cm
    const int32_t z_scaled = (int32_t)(z_m * 100.0);  // cm

    // Extract high precision parts
    const double x_hp_m = x_m - ((double)x_scaled / 100.0);
    const double y_hp_m = y_m - ((double)y_scaled / 100.0);
    const double z_hp_m = z_m - ((double)z_scaled / 100.0);

    const int8_t x_hp = (int8_t)(x_hp_m * 10.0);  // 0.1 mm
    const int8_t y_hp = (int8_t)(y_hp_m * 10.0);  // 0.1 mm
    const int8_t z_hp = (int8_t)(z_hp_m * 10.0);  // 0.1 mm

    // Set position type to ECEF
    zedf9p_status_t status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_POS_TYPE, POS_TYPE_ECEF, 1U);
    if (status != ZEDF9P_OK) return status;

    // Set coordinates
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_X, (uint64_t)x_scaled, 4U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_Y, (uint64_t)y_scaled, 4U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_Z, (uint64_t)z_scaled, 4U);
    if (status != ZEDF9P_OK) return status;

    // Set high precision parts
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_X_HP, x_hp, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_Y_HP, y_hp, 1U);
    if (status != ZEDF9P_OK) return status;

    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_ECEF_Z_HP, z_hp, 1U);
    if (status != ZEDF9P_OK) return status;

    // Set accuracy
    status = zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_FIXED_POS_ACC, accuracy_mm, 4U);
    if (status != ZEDF9P_OK) return status;

    // Set time mode to fixed
    return zedf9p_config_set_val(dev, layer_mask, UBLOX_CFG_TMODE_MODE, TMODE_FIXED_MODE, 1U);
}

zedf9p_status_t zedf9p_get_mon_ver(zedf9p_t *dev, zedf9p_mon_ver_t *mon_ver) {
    if (dev == NULL || !dev->initialized || mon_ver == NULL) {
        return ZEDF9P_ERR_NULL;
    }

    if (!dev->mon_ver_valid) {
        return ZEDF9P_ERR_NO_DATA;
    }

    *mon_ver = dev->mon_ver;
    dev->mon_ver_valid = false;  // Mark as consumed

    return ZEDF9P_OK;
}

bool zedf9p_is_mon_ver_available(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    return dev->mon_ver_valid;
}

zedf9p_status_t zedf9p_poll_mon_ver(zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    ubx_message_t response;
    return zedf9p_poll_ubx_message(dev, UBX_CLASS_MON, UBX_MON_VER,
                                  UBX_POLL_TIMEOUT_MS, &response);
}

zedf9p_status_t zedf9p_disable_7f_check(zedf9p_t *dev, const bool disabled) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }

    dev->ubx_7f_check_disabled = disabled;
    return ZEDF9P_OK;
}

// Callback registration functions
zedf9p_status_t zedf9p_register_clock_callback(zedf9p_t *dev, const zedf9p_message_callback_t callback, void *user_data) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }
    dev->nav_clock_callback = callback;
    dev->callback_user_data = user_data;
    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_register_hpposecef_callback(zedf9p_t *dev, const zedf9p_message_callback_t callback, void *user_data) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }
    dev->nav_hpposecef_callback = callback;
    dev->callback_user_data = user_data;
    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_register_timeutc_callback(zedf9p_t *dev, const zedf9p_message_callback_t callback, void *user_data) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }
    dev->nav_timeutc_callback = callback;
    dev->callback_user_data = user_data;
    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_register_sfrbx_callback(zedf9p_t *dev, const zedf9p_message_callback_t callback, void *user_data) {
    if (dev == NULL || !dev->initialized) {
        return ZEDF9P_ERR_NULL;
    }
    dev->sfrbx_callback = callback;
    dev->callback_user_data = user_data;
    return ZEDF9P_OK;
}

// Data getter functions
zedf9p_status_t zedf9p_get_clock(zedf9p_t *dev, zedf9p_nav_clock_t *clock) {
    if (dev == NULL || !dev->initialized || clock == NULL) {
        return ZEDF9P_ERR_NULL;
    }
    if (!dev->nav_clock_valid) {
        return ZEDF9P_ERR_NO_DATA;
    }
    *clock = dev->nav_clock;
    dev->nav_clock_valid = false;  // Mark as consumed
    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_get_hpposecef(zedf9p_t *dev, zedf9p_nav_hpposecef_t *hpposecef) {
    if (dev == NULL || !dev->initialized || hpposecef == NULL) {
        return ZEDF9P_ERR_NULL;
    }
    if (!dev->nav_hpposecef_valid) {
        return ZEDF9P_ERR_NO_DATA;
    }
    *hpposecef = dev->nav_hpposecef;
    dev->nav_hpposecef_valid = false;  // Mark as consumed
    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_get_timeutc(zedf9p_t *dev, zedf9p_nav_timeutc_t *timeutc) {
    if (dev == NULL || !dev->initialized || timeutc == NULL) {
        return ZEDF9P_ERR_NULL;
    }
    if (!dev->nav_timeutc_valid) {
        return ZEDF9P_ERR_NO_DATA;
    }
    *timeutc = dev->nav_timeutc;
    dev->nav_timeutc_valid = false;  // Mark as consumed
    return ZEDF9P_OK;
}

zedf9p_status_t zedf9p_get_sfrbx(zedf9p_t *dev, zedf9p_sfrbx_t *sfrbx) {
    if (dev == NULL || !dev->initialized || sfrbx == NULL) {
        return ZEDF9P_ERR_NULL;
    }
    if (!dev->sfrbx_valid) {
        return ZEDF9P_ERR_NO_DATA;
    }
    *sfrbx = dev->sfrbx;
    dev->sfrbx_valid = false;  // Mark as consumed
    return ZEDF9P_OK;
}

// Availability check functions
bool zedf9p_is_clock_available(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    return dev->nav_clock_valid;
}

bool zedf9p_is_hpposecef_available(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    return dev->nav_hpposecef_valid;
}

bool zedf9p_is_timeutc_available(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    return dev->nav_timeutc_valid;
}

bool zedf9p_is_sfrbx_available(const zedf9p_t *dev) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }
    return dev->sfrbx_valid;
}