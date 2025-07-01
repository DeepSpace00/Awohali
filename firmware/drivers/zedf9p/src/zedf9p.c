/**
 * @file zedf9p.c
 * @brief Implementation of ZED-F9P GNSS module driver
 * @author Generated Driver
 * @version 1.0
 * 
 * Comprehensive driver implementation for u-blox ZED-F9P GNSS module
 * Supports both I2C and UART communication protocols
 */

#include "zedf9p.h"
#include <string.h>

/* Private function prototypes */
static zedf9p_error_t zedf9p_write_data(zedf9p_handle_t *handle, const uint8_t *data, size_t len);
static zedf9p_error_t zedf9p_read_data(zedf9p_handle_t *handle, uint8_t *data, size_t len, uint32_t timeout_ms);
static zedf9p_error_t zedf9p_wait_for_ack_nak(zedf9p_handle_t *handle, uint8_t msg_class, uint8_t msg_id);
static bool zedf9p_parse_ubx_message(zedf9p_handle_t *handle, ubx_message_t *message);
static zedf9p_error_t zedf9p_i2c_read_available_bytes(zedf9p_handle_t *handle, uint16_t *available_bytes);
static zedf9p_error_t zedf9p_parse_rawx_message(const ubx_message_t *message, zedf9p_rawx_data_t *rawx_data);

/**
 * @brief Initialize the ZED-F9P driver
 */
zedf9p_error_t zedf9p_init(zedf9p_handle_t *handle, const zedf9p_config_t *config)
{
    if (!handle || !config) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }

    /* Copy configuration */
    memcpy(&handle->config, config, sizeof(zedf9p_config_t));
    
    /* Initialize buffers */
    memset(handle->rx_buffer, 0, sizeof(handle->rx_buffer));
    memset(handle->tx_buffer, 0, sizeof(handle->tx_buffer));
    handle->rx_buffer_len = 0;
    
    /* Initialize current message structure */
    memset(&handle->current_message, 0, sizeof(ubx_message_t));
    
    /* Initialize communication interface */
    bool init_success = false;
    
    if (config->interface == ZEDF9P_INTERFACE_I2C) {
        init_success = zedf9p_platform_i2c_init(config->i2c_address);
    } else if (config->interface == ZEDF9P_INTERFACE_UART) {
        init_success = zedf9p_platform_uart_init(config->uart_baudrate);
    }
    
    if (!init_success) {
        return ZEDF9P_ERROR_COMMUNICATION;
    }
    
    handle->initialized = true;
    
    /* Wait for module to be ready */
    zedf9p_platform_delay(100);
    
    return ZEDF9P_OK;
}

/**
 * @brief Deinitialize the ZED-F9P driver
 */
zedf9p_error_t zedf9p_deinit(zedf9p_handle_t *handle)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }
    
    bool deinit_success = false;
    
    if (handle->config.interface == ZEDF9P_INTERFACE_I2C) {
        deinit_success = zedf9p_platform_i2c_deinit();
    } else if (handle->config.interface == ZEDF9P_INTERFACE_UART) {
        deinit_success = zedf9p_platform_uart_deinit();
    }
    
    handle->initialized = false;
    
    return deinit_success ? ZEDF9P_OK : ZEDF9P_ERROR_COMMUNICATION;
}

/**
 * @brief Send UBX message to the module
 */
zedf9p_error_t zedf9p_send_ubx_message(zedf9p_handle_t *handle, uint8_t msg_class, 
                                       uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }
    
    if (payload_len > ZEDF9P_MAX_PAYLOAD_SIZE) {
        return ZEDF9P_ERROR_BUFFER_OVERFLOW;
    }
    
    /* Build UBX message */
    uint8_t *buf = handle->tx_buffer;
    size_t idx = 0;
    
    /* Sync characters */
    buf[idx++] = UBX_SYNC_CHAR_1;
    buf[idx++] = UBX_SYNC_CHAR_2;
    
    /* Message class and ID */
    buf[idx++] = msg_class;
    buf[idx++] = msg_id;
    
    /* Payload length (little endian) */
    buf[idx++] = payload_len & 0xFF;
    buf[idx++] = (payload_len >> 8) & 0xFF;
    
    /* Payload */
    if (payload && payload_len > 0) {
        memcpy(&buf[idx], payload, payload_len);
        idx += payload_len;
    }
    
    /* Calculate checksum */
    uint8_t ck_a, ck_b;
    zedf9p_calculate_ubx_checksum(&buf[2], 4 + payload_len, &ck_a, &ck_b);
    buf[idx++] = ck_a;
    buf[idx++] = ck_b;
    
    /* Send message */
    zedf9p_error_t result = zedf9p_write_data(handle, buf, idx);
    
    if (handle->config.debug_enabled) {
        zedf9p_platform_debug_print("Sent UBX message: Class=0x%02X, ID=0x%02X, Len=%d\n", 
                                    msg_class, msg_id, payload_len);
    }
    
    return result;
}

/**
 * @brief Receive UBX message from the module
 */
zedf9p_error_t zedf9p_receive_ubx_message(zedf9p_handle_t *handle, ubx_message_t *message, uint32_t timeout_ms)
{
    if (!handle || !handle->initialized || !message) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }
    
    uint32_t start_time = zedf9p_platform_get_tick();
    uint8_t state = 0;  /* 0=sync1, 1=sync2, 2=class, 3=id, 4=len1, 5=len2, 6=payload, 7=ck_a, 8=ck_b */
    uint16_t payload_len = 0;
    uint16_t payload_idx = 0;
    uint8_t rx_byte;
    
    message->valid = false;
    
    while ((zedf9p_platform_get_tick() - start_time) < timeout_ms) {
        zedf9p_error_t read_result = zedf9p_read_data(handle, &rx_byte, 1, 10);
        
        if (read_result != ZEDF9P_OK) {
            continue;
        }
        
        switch (state) {
            case 0: /* Sync char 1 */
                if (rx_byte == UBX_SYNC_CHAR_1) {
                    state = 1;
                }
                break;
                
            case 1: /* Sync char 2 */
                if (rx_byte == UBX_SYNC_CHAR_2) {
                    state = 2;
                } else {
                    state = 0;
                }
                break;
                
            case 2: /* Message class */
                message->msg_class = rx_byte;
                state = 3;
                break;
                
            case 3: /* Message ID */
                message->msg_id = rx_byte;
                state = 4;
                break;
                
            case 4: /* Length LSB */
                payload_len = rx_byte;
                state = 5;
                break;
                
            case 5: /* Length MSB */
                payload_len |= (rx_byte << 8);
                message->length = payload_len;
                payload_idx = 0;
                
                if (payload_len > ZEDF9P_MAX_PAYLOAD_SIZE) {
                    state = 0;  /* Reset state machine */
                    continue;
                }
                
                if (payload_len == 0) {
                    state = 7;  /* No payload, go to checksum */
                } else {
                    message->payload = &handle->rx_buffer[UBX_HEADER_SIZE];
                    state = 6;
                }
                break;
                
            case 6: /* Payload */
                message->payload[payload_idx++] = rx_byte;
                if (payload_idx >= payload_len) {
                    state = 7;
                }
                break;
                
            case 7: /* Checksum A */
                message->ck_a = rx_byte;
                state = 8;
                break;
                
            case 8: /* Checksum B */
                message->ck_b = rx_byte;
                
                /* Verify checksum */
                uint8_t calc_ck_a, calc_ck_b;
                uint8_t header[4] = {message->msg_class, message->msg_id, 
                                   payload_len & 0xFF, (payload_len >> 8) & 0xFF};
                
                /* Calculate checksum for header */
                calc_ck_a = 0;
                calc_ck_b = 0;
                for (int i = 0; i < 4; i++) {
                    calc_ck_a += header[i];
                    calc_ck_b += calc_ck_a;
                }
                
                /* Add payload to checksum */
                for (int i = 0; i < payload_len; i++) {
                    calc_ck_a += message->payload[i];
                    calc_ck_b += calc_ck_a;
                }
                
                if (calc_ck_a == message->ck_a && calc_ck_b == message->ck_b) {
                    message->valid = true;
                    
                    if (handle->config.debug_enabled) {
                        zedf9p_platform_debug_print("Received UBX message: Class=0x%02X, ID=0x%02X, Len=%d\n", 
                                                    message->msg_class, message->msg_id, message->length);
                    }
                    
                    return ZEDF9P_OK;
                } else {
                    if (handle->config.debug_enabled) {
                        zedf9p_platform_debug_print("UBX checksum error: expected A=0x%02X B=0x%02X, got A=0x%02X B=0x%02X\n", 
                                                    calc_ck_a, calc_ck_b, message->ck_a, message->ck_b);
                    }
                    state = 0;  /* Reset and try again */
                }
                break;
                
            default:
                state = 0;
                break;
        }
    }
    
    return ZEDF9P_ERROR_TIMEOUT;
}

/**
 * @brief Send UBX command and wait for ACK/NAK
 */
zedf9p_error_t zedf9p_send_ubx_command(zedf9p_handle_t *handle, uint8_t msg_class, 
                                       uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }
    
    /* Send the command */
    zedf9p_error_t result = zedf9p_send_ubx_message(handle, msg_class, msg_id, payload, payload_len);
    if (result != ZEDF9P_OK) {
        return result;
    }
    
    /* Wait for ACK/NAK */
    return zedf9p_wait_for_ack_nak(handle, msg_class, msg_id);
}

/**
 * @brief Poll for specific UBX message
 */
zedf9p_error_t zedf9p_poll_ubx_message(zedf9p_handle_t *handle, uint8_t msg_class, 
                                       uint8_t msg_id, ubx_message_t *response)
{
    if (!handle || !handle->initialized || !response) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }
    
    /* Send poll request (empty message) */
    zedf9p_error_t result = zedf9p_send_ubx_message(handle, msg_class, msg_id, NULL, 0);
    if (result != ZEDF9P_OK) {
        return result;
    }
    
    /* Wait for response */
    uint32_t start_time = zedf9p_platform_get_tick();
    
    while ((zedf9p_platform_get_tick() - start_time) < handle->config.timeout_ms) {
        result = zedf9p_receive_ubx_message(handle, response, 100);
        
        if (result == ZEDF9P_OK && response->valid) {
            if (response->msg_class == msg_class && response->msg_id == msg_id) {
                return ZEDF9P_OK;
            }
        }
    }
    
    return ZEDF9P_ERROR_TIMEOUT;
}

/**
 * @brief Get Navigation PVT data
 */
zedf9p_error_t zedf9p_get_nav_pvt(zedf9p_handle_t *handle, zedf9p_nav_pvt_t *pvt_data)
{
    if (!handle || !handle->initialized || !pvt_data) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }
    
    ubx_message_t response;
    zedf9p_error_t result = zedf9p_poll_ubx_message(handle, UBX_CLASS_NAV, UBX_NAV_PVT, &response);
    
    if (result != ZEDF9P_OK || !response.valid) {
        return result;
    }
    
    if (response.length < 84) {  /* Minimum PVT message size */
        return ZEDF9P_ERROR_INVALID_MESSAGE;
    }
    
    /* Parse PVT data */
    uint8_t *payload = response.payload;
    
    pvt_data->iTOW = *(uint32_t*)&payload[0];
    pvt_data->year = *(uint16_t*)&payload[4];
    pvt_data->month = payload[6];
    pvt_data->day = payload[7];
    pvt_data->hour = payload[8];
    pvt_data->min = payload[9];
    pvt_data->sec = payload[10];
    pvt_data->valid = payload[11];
    pvt_data->tAcc = *(uint32_t*)&payload[12];
    pvt_data->nano = *(int32_t*)&payload[16];
    pvt_data->fixType = payload[20];
    pvt_data->flags = payload[21];
    pvt_data->flags2 = payload[22];
    pvt_data->numSV = payload[23];
    pvt_data->lon = *(int32_t*)&payload[24];
    pvt_data->lat = *(int32_t*)&payload[28];
    pvt_data->height = *(int32_t*)&payload[32];
    pvt_data->hMSL = *(int32_t*)&payload[36];
    pvt_data->hAcc = *(uint32_t*)&payload[40];
    pvt_data->vAcc = *(uint32_t*)&payload[44];
    pvt_data->velN = *(int32_t*)&payload[48];
    pvt_data->velE = *(int32_t*)&payload[52];
    pvt_data->velD = *(int32_t*)&payload[56];
    pvt_data->gSpeed = *(int32_t*)&payload[60];
    pvt_data->headMot = *(int32_t*)&payload[64];
    pvt_data->sAcc = *(uint32_t*)&payload[68];
    pvt_data->headAcc = *(uint32_t*)&payload[72];
    pvt_data->pDOP = *(uint16_t*)&payload[76];
    
    if (response.length >= 92) {  /* Extended PVT message */
        pvt_data->headVeh = *(int32_t*)&payload[84];
        pvt_data->magDec = *(int16_t*)&payload[88];
        pvt_data->magAcc = *(uint16_t*)&payload[90];
    }
    
    return ZEDF9P_OK;
}

/**
 * @brief Get High Precision Position data
 */
zedf9p_error_t zedf9p_get_nav_hpposllh(zedf9p_handle_t *handle, zedf9p_nav_hpposllh_t *hppos_data)
{
    if (!handle || !handle->initialized || !hppos_data) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }
    
    ubx_message_t response;
    zedf9p_error_t result = zedf9p_poll_ubx_message(handle, UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, &response);
    
    if (result != ZEDF9P_OK || !response.valid) {
        return result;
    }
    
    if (response.length < 36) {  /* HPPOSLLH message size */
        return ZEDF9P_ERROR_INVALID_MESSAGE;
    }
    
    /* Parse HPPOSLLH data */
    uint8_t *payload = response.payload;
    
    hppos_data->version = payload[0];
    hppos_data->flags = payload[3];
    hppos_data->iTOW = *(uint32_t*)&payload[4];
    hppos_data->lon = *(int32_t*)&payload[8];
    hppos_data->lat = *(int32_t*)&payload[12];
    hppos_data->height = *(int32_t*)&payload[16];
    hppos_data->hMSL = *(int32_t*)&payload[20];
    hppos_data->lonHp = (int8_t)payload[24];
    hppos_data->latHp = (int8_t)payload[25];
    hppos_data->heightHp = (int8_t)payload[26];
    hppos_data->hMSLHp = (int8_t)payload[27];
    hppos_data->hAcc = *(uint32_t*)&payload[28];
    hppos_data->vAcc = *(uint32_t*)&payload[32];
    
    return ZEDF9P_OK;
}

/**
 * @brief Set measurement rate
 */
zedf9p_error_t zedf9p_set_measurement_rate(zedf9p_handle_t *handle, uint16_t rate_ms, 
                                          uint16_t nav_rate, uint16_t time_ref)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }
    
    uint8_t payload[6];
    
    /* Build CFG-RATE payload */
    *(uint16_t*)&payload[0] = rate_ms;    /* Measurement rate */
    *(uint16_t*)&payload[2] = nav_rate;   /* Navigation rate */
    *(uint16_t*)&payload[4] = time_ref;   /* Time reference */
    
    return zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_RATE, payload, sizeof(payload));
}

/**
 * @brief Configure message output rate
 */
zedf9p_error_t zedf9p_configure_message_rate(zedf9p_handle_t *handle, uint8_t msg_class, 
                                            uint8_t msg_id, uint8_t rate)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }
    
    uint8_t payload[3];
    
    /* Build CFG-MSG payload */
    payload[0] = msg_class;
    payload[1] = msg_id;
    payload[2] = rate;
    
    return zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_MSG, payload, sizeof(payload));
}

/**
 * @brief Reset the module
 */
zedf9p_error_t zedf9p_reset(zedf9p_handle_t *handle, uint8_t reset_type)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }
    
    uint8_t payload[4];
    
    /* Build CFG-RST payload */
    *(uint16_t*)&payload[0] = 0xFFFF;     /* Clear all */
    payload[2] = reset_type;              /* Reset type */
    payload[3] = 0x00;                    /* Reserved */
    
    /* Send reset command (don't wait for ACK as module will reset) */
    return zedf9p_send_ubx_message(handle, UBX_CLASS_CFG, UBX_CFG_RST, payload, sizeof(payload));
}

/**
 * @brief Save current configuration to non-volatile memory
 */
zedf9p_error_t zedf9p_save_config(zedf9p_handle_t *handle)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }
    
    uint8_t payload[12];
    
    /* Build CFG-CFG payload */
    *(uint32_t*)&payload[0] = 0x00000000;  /* Clear mask */
    *(uint32_t*)&payload[4] = 0xFFFFFFFF;  /* Save mask (all) */
    *(uint32_t*)&payload[8] = 0x00000000;  /* Load mask */
    
    return zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_CFG, payload, sizeof(payload));
}

/**
 * @brief Load configuration from non-volatile memory
 */
zedf9p_error_t zedf9p_load_config(zedf9p_handle_t *handle)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }
    
    uint8_t payload[12];
    
    /* Build CFG-CFG payload */
    *(uint32_t*)&payload[0] = 0x00000000;  /* Clear mask */
    *(uint32_t*)&payload[4] = 0x00000000;  /* Save mask */
    *(uint32_t*)&payload[8] = 0xFFFFFFFF;  /* Load mask (all) */
    
    return zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_CFG, payload, sizeof(payload));
}

/**
 * @brief Get module version information
 */
zedf9p_error_t zedf9p_get_version(zedf9p_handle_t *handle, char *version_info, size_t buffer_size)
{
    if (!handle || !handle->initialized || !version_info || buffer_size == 0) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }
    
    ubx_message_t response;
    zedf9p_error_t result = zedf9p_poll_ubx_message(handle, UBX_CLASS_MON, UBX_MON_VER, &response);
    
    if (result != ZEDF9P_OK || !response.valid) {
        return result;
    }
    
    if (response.length < 40) {
        return ZEDF9P_ERROR_INVALID_MESSAGE;
    }
    
    /* Extract software version (first 30 characters) */
    size_t copy_len = (buffer_size - 1 < 30) ? buffer_size - 1 : 30;
    memcpy(version_info, response.payload, copy_len);
    version_info[copy_len] = '\0';
    
    return ZEDF9P_OK;
}

/**
 * @brief Enable/disable RAWX message output
 */
zedf9p_error_t zedf9p_enable_rawx(zedf9p_handle_t *handle, bool enable, uint8_t output_rate)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }
    
    uint8_t rate = enable ? output_rate : 0;
    
    /* Configure RXM-RAWX message rate */
    zedf9p_error_t result = zedf9p_configure_message_rate(handle, UBX_CLASS_RXM, UBX_RXM_RAWX, rate);
    
    if (handle->config.debug_enabled) {
        zedf9p_platform_debug_print("RAWX output %s with rate %d\n", 
                                    enable ? "enabled" : "disabled", rate);
    }
    
    return result;
}

/**
 * @brief Get RAWX raw measurement data
 */
zedf9p_error_t zedf9p_get_rawx_data(zedf9p_handle_t *handle, zedf9p_rawx_data_t *rawx_data, uint32_t timeout_ms)
{
    if (!handle || !handle->initialized || !rawx_data) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }
    
    ubx_message_t response;
    uint32_t start_time = zedf9p_platform_get_tick();
    
    rawx_data->valid = false;
    
    /* Wait for RAWX message */
    while ((zedf9p_platform_get_tick() - start_time) < timeout_ms) {
        zedf9p_error_t result = zedf9p_receive_ubx_message(handle, &response, 100);
        
        if (result == ZEDF9P_OK && response.valid) {
            if (response.msg_class == UBX_CLASS_RXM && response.msg_id == UBX_RXM_RAWX) {
                return zedf9p_parse_rawx_message(&response, rawx_data);
            }
        }
    }
    
    return ZEDF9P_ERROR_TIMEOUT;
}

/**
 * @brief Poll for RAWX data (request single measurement)
 */
zedf9p_error_t zedf9p_poll_rawx_data(zedf9p_handle_t *handle, zedf9p_rawx_data_t *rawx_data)
{
    if (!handle || !handle->initialized || !rawx_data) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }
    
    ubx_message_t response;
    zedf9p_error_t result = zedf9p_poll_ubx_message(handle, UBX_CLASS_RXM, UBX_RXM_RAWX, &response);
    
    if (result != ZEDF9P_OK || !response.valid) {
        return result;
    }
    
    return zedf9p_parse_rawx_message(&response, rawx_data);
}

/**
 * @brief Parse RAWX message data
 */
static zedf9p_error_t zedf9p_parse_rawx_message(const ubx_message_t *message, zedf9p_rawx_data_t *rawx_data)
{
    if (!message || !rawx_data || message->length < 16) {
        return ZEDF9P_ERROR_INVALID_MESSAGE;
    }
    
    uint8_t *payload = message->payload;
    size_t offset = 0;
    
    /* Parse header */
    rawx_data->header.rcvTow = *(double*)&payload[offset];
    offset += 8;
    rawx_data->header.week = *(uint16_t*)&payload[offset];
    offset += 2;
    rawx_data->header.leapS = (int8_t)payload[offset++];
    rawx_data->header.numMeas = payload[offset++];
    rawx_data->header.recStat = payload[offset++];
    rawx_data->header.version = payload[offset++];
    offset += 2; /* Skip reserved bytes */
    
    /* Validate number of measurements */
    if (rawx_data->header.numMeas > 64) {
        return ZEDF9P_ERROR_INVALID_MESSAGE;
    }
    
    /* Check if message length matches expected size */
    size_t expected_length = 16 + (rawx_data->header.numMeas * 32);
    if (message->length < expected_length) {
        return ZEDF9P_ERROR_INVALID_MESSAGE;
    }
    
    rawx_data->num_measurements = rawx_data->header.numMeas;
    
    /* Parse individual measurements */
    for (uint8_t i = 0; i < rawx_data->header.numMeas && i < 64; i++) {
        zedf9p_rawx_meas_t *meas = &rawx_data->measurements[i];
        
        meas->prMes = *(double*)&payload[offset];
        offset += 8;
        meas->cpMes = *(double*)&payload[offset];
        offset += 8;
        meas->doMes = *(float*)&payload[offset];
        offset += 4;
        meas->gnssId = payload[offset++];
        meas->svId = payload[offset++];
        meas->sigId = payload[offset++];
        meas->freqId = payload[offset++];
        meas->locktime = *(uint16_t*)&payload[offset];
        offset += 2;
        meas->cno = payload[offset++];
        meas->prStdev = payload[offset++];
        meas->cpStdev = payload[offset++];
        meas->doStdev = payload[offset++];
        meas->trkStat = payload[offset++];
        offset++; /* Skip reserved byte */
    }
    
    rawx_data->valid = true;
    
    return ZEDF9P_OK;
}

/**
 * @brief Set frequency band configuration for GNSS signals
 */
zedf9p_error_t zedf9p_set_frequency_band(zedf9p_handle_t *handle, zedf9p_freq_band_t freq_band)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }

    zedf9p_error_t result = ZEDF9P_OK;

    switch (freq_band) {
        case ZEDF9P_FREQ_L1_ONLY:
            // Enable L1 bands, disable L2 bands
            result = zedf9p_configure_gps_bands(handle, true, false);
            if (result != ZEDF9P_OK) return result;
            
            result = zedf9p_configure_galileo_bands(handle, true, false);
            if (result != ZEDF9P_OK) return result;
            
            result = zedf9p_configure_beidou_bands(handle, true, false);
            if (result != ZEDF9P_OK) return result;
            
            result = zedf9p_configure_glonass_bands(handle, true, false);
            break;

        case ZEDF9P_FREQ_L2_ONLY:
            // Enable L2 bands, disable L1 bands
            result = zedf9p_configure_gps_bands(handle, false, true);
            if (result != ZEDF9P_OK) return result;
            
            result = zedf9p_configure_galileo_bands(handle, false, true);
            if (result != ZEDF9P_OK) return result;
            
            result = zedf9p_configure_beidou_bands(handle, false, true);
            if (result != ZEDF9P_OK) return result;
            
            result = zedf9p_configure_glonass_bands(handle, false, true);
            break;

        case ZEDF9P_FREQ_L1_L2_DUAL:
            // Enable both L1 and L2 bands
            result = zedf9p_configure_gps_bands(handle, true, true);
            if (result != ZEDF9P_OK) return result;
            
            result = zedf9p_configure_galileo_bands(handle, true, true);
            if (result != ZEDF9P_OK) return result;
            
            result = zedf9p_configure_beidou_bands(handle, true, true);
            if (result != ZEDF9P_OK) return result;
            
            result = zedf9p_configure_glonass_bands(handle, true, true);
            break;

        default:
            return ZEDF9P_ERROR_INVALID_PARAM;
    }

    return result;
}

/**
 * @brief Configure GPS L1/L2 signal reception
 */
zedf9p_error_t zedf9p_configure_gps_bands(zedf9p_handle_t *handle, bool enable_l1, bool enable_l2)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }

    zedf9p_error_t result;
    uint8_t payload[8];
    uint32_t key_id;
    uint8_t value;

    // Configure GPS L1 C/A
    key_id = CFG_SIGNAL_GPS_L1CA_ENA;
    value = enable_l1 ? 1 : 0;
    
    payload[0] = 0x00;  // Version
    payload[1] = 0x01;  // Layer: RAM
    payload[2] = 0x00;  // Reserved
    payload[3] = 0x00;  // Reserved
    
    // Key ID (little endian)
    payload[4] = (uint8_t)(key_id & 0xFF);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFF);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFF);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFF);
    
    // Value
    payload[8] = value;

    result = zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
    if (result != ZEDF9P_OK) {
        return result;
    }

    // Configure GPS L2C
    key_id = CFG_SIGNAL_GPS_L2C_ENA;
    value = enable_l2 ? 1 : 0;
    
    payload[4] = (uint8_t)(key_id & 0xFF);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFF);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFF);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFF);
    payload[8] = value;

    result = zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
    
    return result;
}

/**
 * @brief Configure Galileo E1/E5b signal reception
 */
zedf9p_error_t zedf9p_configure_galileo_bands(zedf9p_handle_t *handle, bool enable_e1, bool enable_e5b)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }

    zedf9p_error_t result;
    uint8_t payload[9];
    uint32_t key_id;
    uint8_t value;

    // Configure Galileo E1
    key_id = CFG_SIGNAL_GAL_E1_ENA;
    value = enable_e1 ? 1 : 0;
    
    payload[0] = 0x00;  // Version
    payload[1] = 0x01;  // Layer: RAM
    payload[2] = 0x00;  // Reserved
    payload[3] = 0x00;  // Reserved
    
    // Key ID (little endian)
    payload[4] = (uint8_t)(key_id & 0xFF);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFF);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFF);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFF);
    
    // Value
    payload[8] = value;

    result = zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
    if (result != ZEDF9P_OK) {
        return result;
    }

    // Configure Galileo E5b
    key_id = CFG_SIGNAL_GAL_E5B_ENA;
    value = enable_e5b ? 1 : 0;
    
    payload[4] = (uint8_t)(key_id & 0xFF);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFF);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFF);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFF);
    payload[8] = value;

    result = zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
    
    return result;
}

/**
 * @brief Configure BeiDou B1/B2 signal reception
 */
zedf9p_error_t zedf9p_configure_beidou_bands(zedf9p_handle_t *handle, bool enable_b1, bool enable_b2)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }

    zedf9p_error_t result;
    uint8_t payload[9];
    uint32_t key_id;
    uint8_t value;

    // Configure BeiDou B1I
    key_id = CFG_SIGNAL_BDS_B1_ENA;
    value = enable_b1 ? 1 : 0;
    
    payload[0] = 0x00;  // Version
    payload[1] = 0x01;  // Layer: RAM
    payload[2] = 0x00;  // Reserved
    payload[3] = 0x00;  // Reserved
    
    // Key ID (little endian)
    payload[4] = (uint8_t)(key_id & 0xFF);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFF);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFF);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFF);
    
    // Value
    payload[8] = value;

    result = zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
    if (result != ZEDF9P_OK) {
        return result;
    }

    // Configure BeiDou B2I
    key_id = CFG_SIGNAL_BDS_B2_ENA;
    value = enable_b2 ? 1 : 0;
    
    payload[4] = (uint8_t)(key_id & 0xFF);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFF);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFF);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFF);
    payload[8] = value;

    result = zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
    
    return result;
}

/**
 * @brief Configure GLONASS L1/L2 signal reception
 */
zedf9p_error_t zedf9p_configure_glonass_bands(zedf9p_handle_t *handle, bool enable_l1, bool enable_l2)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }

    zedf9p_error_t result;
    uint8_t payload[9];
    uint32_t key_id;
    uint8_t value;

    // Configure GLONASS L1 OF
    key_id = CFG_SIGNAL_GLO_L1_ENA;
    value = enable_l1 ? 1 : 0;
    
    payload[0] = 0x00;  // Version
    payload[1] = 0x01;  // Layer: RAM
    payload[2] = 0x00;  // Reserved
    payload[3] = 0x00;  // Reserved
    
    // Key ID (little endian)
    payload[4] = (uint8_t)(key_id & 0xFF);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFF);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFF);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFF);
    
    // Value
    payload[8] = value;

    result = zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
    if (result != ZEDF9P_OK) {
        return result;
    }

    // Configure GLONASS L2 OF
    key_id = CFG_SIGNAL_GLO_L2_ENA;
    value = enable_l2 ? 1 : 0;
    
    payload[4] = (uint8_t)(key_id & 0xFF);
    payload[5] = (uint8_t)((key_id >> 8) & 0xFF);
    payload[6] = (uint8_t)((key_id >> 16) & 0xFF);
    payload[7] = (uint8_t)((key_id >> 24) & 0xFF);
    payload[8] = value;

    result = zedf9p_send_ubx_command(handle, UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 9);
    
    return result;
}

/**
 * @brief Get current frequency band configuration
 */
zedf9p_error_t zedf9p_get_frequency_config(zedf9p_handle_t *handle, 
                                          bool *gps_l1_enabled, bool *gps_l2_enabled,
                                          bool *gal_e1_enabled, bool *gal_e5b_enabled)
{
    if (!handle || !handle->initialized) {
        return ZEDF9P_ERROR_NOT_INITIALIZED;
    }

    if (!gps_l1_enabled || !gps_l2_enabled || !gal_e1_enabled || !gal_e5b_enabled) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }

    zedf9p_error_t result;
    uint8_t payload[8];
    ubx_message_t response;
    uint32_t key_ids[] = {
        CFG_SIGNAL_GPS_L1CA_ENA,
        CFG_SIGNAL_GPS_L2C_ENA,
        CFG_SIGNAL_GAL_E1_ENA,
        CFG_SIGNAL_GAL_E5B_ENA
    };
    bool *outputs[] = {gps_l1_enabled, gps_l2_enabled, gal_e1_enabled, gal_e5b_enabled};

    for (int i = 0; i < 4; i++) {
        // Prepare VALGET request
        payload[0] = 0x00;  // Version
        payload[1] = 0x00;  // Layer: RAM
        payload[2] = 0x00;  // Reserved
        payload[3] = 0x00;  // Reserved
        
        // Key ID (little endian)
        payload[4] = (uint8_t)(key_ids[i] & 0xFF);
        payload[5] = (uint8_t)((key_ids[i] >> 8) & 0xFF);
        payload[6] = (uint8_t)((key_ids[i] >> 16) & 0xFF);
        payload[7] = (uint8_t)((key_ids[i] >> 24) & 0xFF);

        result = zedf9p_send_ubx_message(handle, UBX_CLASS_CFG, UBX_CFG_VALGET, payload, 8);
        if (result != ZEDF9P_OK) {
            return result;
        }

        // Wait for response
        result = zedf9p_receive_ubx_message(handle, &response, handle->config.timeout_ms);
        if (result != ZEDF9P_OK) {
            return result;
        }

        // Check if we got the expected response
        if (response.msg_class != UBX_CLASS_CFG || response.msg_id != UBX_CFG_VALGET) {
            return ZEDF9P_ERROR_INVALID_MESSAGE;
        }

        // Extract the configuration value (should be at payload offset 8)
        if (response.length >= 9) {
            *outputs[i] = (response.payload[8] != 0);
        } else {
            return ZEDF9P_ERROR_INVALID_MESSAGE;
        }
    }

    return ZEDF9P_OK;
}

/**
 * @brief Filter RAWX measurements by frequency band
 */
zedf9p_error_t zedf9p_filter_rawx_by_frequency(zedf9p_rawx_data_t *rawx_data, 
                                              zedf9p_freq_band_t freq_band,
                                              uint8_t *filtered_count)
{
    if (!rawx_data || !filtered_count) {
        return ZEDF9P_ERROR_INVALID_PARAM;
    }

    if (!rawx_data->valid || rawx_data->num_measurements == 0) {
        *filtered_count = 0;
        return ZEDF9P_OK;
    }

    uint8_t write_index = 0;
    bool keep_measurement;

    for (uint8_t i = 0; i < rawx_data->num_measurements; i++) {
        keep_measurement = false;
        zedf9p_rawx_meas_t *meas = &rawx_data->measurements[i];

        switch (freq_band) {
            case ZEDF9P_FREQ_L1_ONLY:
                // Keep L1 band signals
                switch (meas->gnssId) {
                    case GNSS_ID_GPS:
                        keep_measurement = (meas->sigId == GNSS_GPS_L1CA);
                        break;
                    case GNSS_ID_GALILEO:
                        keep_measurement = (meas->sigId == GNSS_GALILEO_E1C || meas->sigId == GNSS_GALILEO_E1B);
                        break;
                    case GNSS_ID_BEIDOU:
                        keep_measurement = (meas->sigId == GNSS_BEIDOU_B1I || meas->sigId == GNSS_BEIDOU_B1I_D2);
                        break;
                    case GNSS_ID_GLONASS:
                        keep_measurement = (meas->sigId == GNSS_GLONASS_L1OF);
                        break;
                    default:
                        keep_measurement = false;
                        break;
                }
                break;

            case ZEDF9P_FREQ_L2_ONLY:
                // Keep L2 band signals
                switch (meas->gnssId) {
                    case GNSS_ID_GPS:
                        keep_measurement = (meas->sigId == GNSS_GPS_L2CL || meas->sigId == GNSS_GPS_L2CM);
                        break;
                    case GNSS_ID_GALILEO:
                        keep_measurement = (meas->sigId == GNSS_GALILEO_E5BI || meas->sigId == GNSS_GALILEO_E5BQ);
                        break;
                    case GNSS_ID_BEIDOU:
                        keep_measurement = (meas->sigId == GNSS_BEIDOU_B2I || meas->sigId == GNSS_BEIDOU_B2I_D2);
                        break;
                    case GNSS_ID_GLONASS:
                        keep_measurement = (meas->sigId == GNSS_GLONASS_L2OF);
                        break;
                    default:
                        keep_measurement = false;
                        break;
                }
                break;

            case ZEDF9P_FREQ_L1_L2_DUAL:
                // Keep all L1 and L2 signals
                switch (meas->gnssId) {
                    case GNSS_ID_GPS:
                        keep_measurement = (meas->sigId == GNSS_GPS_L1CA || 
                                          meas->sigId == GNSS_GPS_L2CL || 
                                          meas->sigId == GNSS_GPS_L2CM);
                        break;
                    case GNSS_ID_GALILEO:
                        keep_measurement = (meas->sigId == GNSS_GALILEO_E1C || 
                                          meas->sigId == GNSS_GALILEO_E1B ||
                                          meas->sigId == GNSS_GALILEO_E5BI || 
                                          meas->sigId == GNSS_GALILEO_E5BQ);
                        break;
                    case GNSS_ID_BEIDOU:
                        keep_measurement = (meas->sigId == GNSS_BEIDOU_B1I || 
                                          meas->sigId == GNSS_BEIDOU_B1I_D2 ||
                                          meas->sigId == GNSS_BEIDOU_B2I || 
                                          meas->sigId == GNSS_BEIDOU_B2I_D2);
                        break;
                    case GNSS_ID_GLONASS:
                        keep_measurement = (meas->sigId == GNSS_GLONASS_L1OF || 
                                          meas->sigId == GNSS_GLONASS_L2OF);
                        break;
                    default:
                        keep_measurement = false;
                        break;
                }
                break;

            default:
                return ZEDF9P_ERROR_INVALID_PARAM;
        }

        // If we want to keep this measurement, copy it to the write position
        if (keep_measurement) {
            if (write_index != i) {
                memcpy(&rawx_data->measurements[write_index], 
                       &rawx_data->measurements[i], 
                       sizeof(zedf9p_rawx_meas_t));
            }
            write_index++;
        }
    }

    // Update the count and return filtered count
    rawx_data->num_measurements = write_index;
    *filtered_count = write_index;

    return ZEDF9P_OK;
}

/**
 * @brief Calculate UBX checksum
 */
void zedf9p_calculate_ubx_checksum(const uint8_t *data, uint16_t length, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    
    for (uint16_t i = 0; i < length; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

/**
 * @brief Convert error code to string
 */
const char* zedf9p_error_to_string(zedf9p_error_t error)
{
    switch (error) {
        case ZEDF9P_OK:                     return "OK";
        case ZEDF9P_ERROR_INVALID_PARAM:    return "Invalid parameter";
        case ZEDF9P_ERROR_COMMUNICATION:    return "Communication error";
        case ZEDF9P_ERROR_TIMEOUT:          return "Timeout";
        case ZEDF9P_ERROR_CHECKSUM:         return "Checksum error";
        case ZEDF9P_ERROR_NACK:             return "Command not acknowledged";
        case ZEDF9P_ERROR_BUFFER_OVERFLOW:  return "Buffer overflow";
        case ZEDF9P_ERROR_NOT_INITIALIZED:  return "Driver not initialized";
        case ZEDF9P_ERROR_INVALID_MESSAGE:  return "Invalid message";
        default:                            return "Unknown error";
    }
}

/* Private function implementations */

/**
 * @brief Write data to module via selected interface
 */
static zedf9p_error_t zedf9p_write_data(zedf9p_handle_t *handle, const uint8_t *data, size_t len)
{
    bool success = false;
    
    if (handle->config.interface == ZEDF9P_INTERFACE_I2C) {
        success = zedf9p_platform_i2c_write(handle->config.i2c_address, data, len);
    } else if (handle->config.interface == ZEDF9P_INTERFACE_UART) {
        success = zedf9p_platform_uart_write(data, len);
    }
    
    return success ? ZEDF9P_OK : ZEDF9P_ERROR_COMMUNICATION;
}

/**
 * @brief Read data from module via selected interface
 */
static zedf9p_error_t zedf9p_read_data(zedf9p_handle_t *handle, uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (handle->config.interface == ZEDF9P_INTERFACE_I2C) {
        /* For I2C, we need to check available bytes first */
        uint16_t available = 0;
        zedf9p_error_t result = zedf9p_i2c_read_available_bytes(handle, &available);
        
        if (result != ZEDF9P_OK || available == 0) {
            return ZEDF9P_ERROR_TIMEOUT;
        }
        
        size_t read_len = (len < available) ? len : available;
        bool success = zedf9p_platform_i2c_read(handle->config.i2c_address, data, read_len);
        return success ? ZEDF9P_OK : ZEDF9P_ERROR_COMMUNICATION;
        
    } else if (handle->config.interface == ZEDF9P_INTERFACE_UART) {
        size_t bytes_read = zedf9p_platform_uart_read(data, len, timeout_ms);
        return (bytes_read > 0) ? ZEDF9P_OK : ZEDF9P_ERROR_TIMEOUT;
    }
    
    return ZEDF9P_ERROR_COMMUNICATION;
}

/**
 * @brief Wait for ACK/NAK response
 */
static zedf9p_error_t zedf9p_wait_for_ack_nak(zedf9p_handle_t *handle, uint8_t msg_class, uint8_t msg_id)
{
    ubx_message_t response;
    uint32_t start_time = zedf9p_platform_get_tick();
    
    while ((zedf9p_platform_get_tick() - start_time) < handle->config.timeout_ms) {
        zedf9p_error_t result = zedf9p_receive_ubx_message(handle, &response, 100);
        
        if (result == ZEDF9P_OK && response.valid) {
            if (response.msg_class == UBX_CLASS_ACK && response.length == 2) {
                if (response.msg_id == UBX_ACK_ACK && 
                    response.payload[0] == msg_class && response.payload[1] == msg_id) {
                    if (handle->config.debug_enabled) {
                        zedf9p_platform_debug_print("Command ACK received for Class=0x%02X, ID=0x%02X\n", 
                                                    msg_class, msg_id);
                    }
                    return ZEDF9P_OK;
                } else if (response.msg_id == UBX_ACK_NAK && 
                          response.payload[0] == msg_class && response.payload[1] == msg_id) {
                    if (handle->config.debug_enabled) {
                        zedf9p_platform_debug_print("Command NAK received for Class=0x%02X, ID=0x%02X\n", 
                                                    msg_class, msg_id);
                    }
                    return ZEDF9P_ERROR_NACK;
                }
            }
        }
    }
    
    return ZEDF9P_ERROR_TIMEOUT;
}

/**
 * @brief Read available bytes from I2C interface
 */
static zedf9p_error_t zedf9p_i2c_read_available_bytes(zedf9p_handle_t *handle, uint16_t *available_bytes)
{
    uint8_t bytes_available_reg[2];
    
    /* Read the first 2 bytes which contain the number of available bytes */
    bool success = zedf9p_platform_i2c_read(handle->config.i2c_address, bytes_available_reg, 2);
    
    if (!success) {
        return ZEDF9P_ERROR_COMMUNICATION;
    }
    
    *available_bytes = (bytes_available_reg[0] << 8) | bytes_available_reg[1];
    
    /* Check for special values */
    if (*available_bytes == 0xFFFF) {
        *available_bytes = 0;  /* No data available */
    }
    
    return ZEDF9P_OK;
}