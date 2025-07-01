/**
 * @file zedf9p.h
 * @brief Header file for ZED-F9P GNSS module driver
 * @author Generated Driver
 * @version 1.0
 * 
 * Comprehensive driver for u-blox ZED-F9P GNSS module
 * Supports both I2C and UART communication protocols
 */

#ifndef ZEDF9P_H
#define ZEDF9P_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "zedf9p_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Driver Configuration */
#define ZEDF9P_DEFAULT_I2C_ADDR     0x42
#define ZEDF9P_DEFAULT_UART_BAUD    38400
#define ZEDF9P_MAX_PAYLOAD_SIZE     1024
#define ZEDF9P_DEFAULT_TIMEOUT_MS   1000
#define ZEDF9P_SYNC_TIMEOUT_MS      5000

/* UBX Protocol Constants */
#define UBX_SYNC_CHAR_1     0xB5
#define UBX_SYNC_CHAR_2     0x62
#define UBX_HEADER_SIZE     6
#define UBX_CHECKSUM_SIZE   2

/* Common UBX Message Classes */
#define UBX_CLASS_NAV       0x01    /* Navigation Results */
#define UBX_CLASS_RXM       0x02    /* Receiver Manager Messages */
#define UBX_CLASS_INF       0x04    /* Information Messages */
#define UBX_CLASS_ACK       0x05    /* Ack/Nak Messages */
#define UBX_CLASS_CFG       0x06    /* Configuration Input Messages */
#define UBX_CLASS_UPD       0x09    /* Firmware Update Messages */
#define UBX_CLASS_MON       0x0A    /* Monitoring Messages */
#define UBX_CLASS_TIM       0x0D    /* Timing Messages */
#define UBX_CLASS_ESF       0x10    /* External Sensor Fusion Messages */
#define UBX_CLASS_MGA       0x13    /* Multiple GNSS Assistance Messages */
#define UBX_CLASS_LOG       0x21    /* Logging Messages */
#define UBX_CLASS_SEC       0x27    /* Security Feature Messages */
#define UBX_CLASS_HNR       0x28    /* High Rate Navigation Messages */

/* Common UBX Message IDs */
#define UBX_NAV_PVT         0x07    /* Navigation Position Velocity Time Solution */
#define UBX_NAV_HPPOSLLH    0x14    /* High Precision Geodetic Position Solution */
#define UBX_NAV_STATUS      0x03    /* Receiver Navigation Status */
#define UBX_NAV_DOP         0x04    /* Dilution of precision */
#define UBX_NAV_SAT         0x35    /* Satellite Information */

#define UBX_RXM_RAWX        0x15    /* Multi-GNSS Raw Measurement Data */
#define UBX_RXM_SFRBX       0x13    /* Broadcast Navigation Data Subframe */
#define UBX_RXM_MEASX       0x14    /* Satellite Measurements for RRLP */

#define UBX_CFG_PRT         0x00    /* Port Configuration */
#define UBX_CFG_MSG         0x01    /* Message Configuration */
#define UBX_CFG_INF         0x02    /* Information Message Configuration */
#define UBX_CFG_RST         0x04    /* Reset Receiver / Clear Backup Data Structure */
#define UBX_CFG_DAT         0x06    /* Set User-defined Datum */
#define UBX_CFG_RATE        0x08    /* Navigation/Measurement Rate Settings */
#define UBX_CFG_CFG         0x09    /* Clear, Save and Load configurations */
#define UBX_CFG_SBAS        0x16    /* SBAS Configuration */
#define UBX_CFG_GNSS        0x3E    /* GNSS system configuration */
#define UBX_CFG_VALSET      0x8A    /* Set configuration values */
#define UBX_CFG_VALGET      0x8B    /* Get configuration values */
#define UBX_CFG_VALDEL      0x8C    /* Delete configuration values */

#define UBX_ACK_NAK         0x00    /* Message Not-Acknowledged */
#define UBX_ACK_ACK         0x01    /* Message Acknowledged */

#define UBX_MON_VER         0x04    /* Receiver/Software Version */
#define UBX_MON_HW          0x09    /* Hardware Status */

/* GNSS Signal ID Constants for L1/L2 Frequency Bands */
#define GNSS_GPS_L1CA       0x00    /* GPS L1 C/A */
#define GNSS_GPS_L2CL       0x03    /* GPS L2 CL */
#define GNSS_GPS_L2CM       0x04    /* GPS L2 CM */
#define GNSS_GALILEO_E1C    0x00    /* Galileo E1 C */
#define GNSS_GALILEO_E1B    0x01    /* Galileo E1 B */
#define GNSS_GALILEO_E5BI   0x05    /* Galileo E5b I */
#define GNSS_GALILEO_E5BQ   0x06    /* Galileo E5b Q */
#define GNSS_BEIDOU_B1I     0x00    /* BeiDou B1I D1 */
#define GNSS_BEIDOU_B1I_D2  0x01    /* BeiDou B1I D2 */
#define GNSS_BEIDOU_B2I     0x02    /* BeiDou B2I D1 */
#define GNSS_BEIDOU_B2I_D2  0x03    /* BeiDou B2I D2 */
#define GNSS_GLONASS_L1OF   0x00    /* GLONASS L1 OF */
#define GNSS_GLONASS_L2OF   0x02    /* GLONASS L2 OF */

/* GNSS System ID Constants */
#define GNSS_ID_GPS         0x00    /* GPS */
#define GNSS_ID_SBAS        0x01    /* SBAS */
#define GNSS_ID_GALILEO     0x02    /* Galileo */
#define GNSS_ID_BEIDOU      0x03    /* BeiDou */
#define GNSS_ID_IMES        0x04    /* IMES */
#define GNSS_ID_QZSS        0x05    /* QZSS */
#define GNSS_ID_GLONASS     0x06    /* GLONASS */

/* Configuration Key IDs for L1/L2 Band Selection */
#define CFG_SIGNAL_GPS_ENA          0x1031001F  /* GPS enable */
#define CFG_SIGNAL_GPS_L1CA_ENA     0x10310001  /* GPS L1C/A enable */
#define CFG_SIGNAL_GPS_L2C_ENA      0x10310003  /* GPS L2C enable */
#define CFG_SIGNAL_GAL_ENA          0x10310021  /* Galileo enable */
#define CFG_SIGNAL_GAL_E1_ENA       0x10310007  /* Galileo E1 enable */
#define CFG_SIGNAL_GAL_E5B_ENA      0x1031000A  /* Galileo E5b enable */
#define CFG_SIGNAL_BDS_ENA          0x10310022  /* BeiDou enable */
#define CFG_SIGNAL_BDS_B1_ENA       0x1031000D  /* BeiDou B1I enable */
#define CFG_SIGNAL_BDS_B2_ENA       0x1031000E  /* BeiDou B2I enable */
#define CFG_SIGNAL_GLO_ENA          0x10310025  /* GLONASS enable */
#define CFG_SIGNAL_GLO_L1_ENA       0x10310018  /* GLONASS L1 enable */
#define CFG_SIGNAL_GLO_L2_ENA       0x1031001A  /* GLONASS L2 enable */

/* Communication Interface Types */
typedef enum {
    ZEDF9P_INTERFACE_I2C,
    ZEDF9P_INTERFACE_UART
} zedf9p_interface_t;

/* Driver Error Codes */
typedef enum {
    ZEDF9P_OK = 0,
    ZEDF9P_ERROR_INVALID_PARAM,
    ZEDF9P_ERROR_COMMUNICATION,
    ZEDF9P_ERROR_TIMEOUT,
    ZEDF9P_ERROR_CHECKSUM,
    ZEDF9P_ERROR_NACK,
    ZEDF9P_ERROR_BUFFER_OVERFLOW,
    ZEDF9P_ERROR_NOT_INITIALIZED,
    ZEDF9P_ERROR_INVALID_MESSAGE
} zedf9p_error_t;

/* Frequency Band Selection */
typedef enum {
    ZEDF9P_FREQ_L1_ONLY,        /* L1 band only */
    ZEDF9P_FREQ_L2_ONLY,        /* L2 band only */
    ZEDF9P_FREQ_L1_L2_DUAL      /* Dual L1+L2 bands */
} zedf9p_freq_band_t;

/* UBX Message Structure */
typedef struct {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t *payload;
    uint8_t ck_a;
    uint8_t ck_b;
    bool valid;
} ubx_message_t;

/* RAWX Measurement Data Structure */
typedef struct {
    double prMes;               /* Pseudorange measurement [m] */
    double cpMes;               /* Carrier phase measurement [L1 cycles] */
    float doMes;                /* Doppler measurement [Hz] */
    uint8_t gnssId;             /* GNSS identifier */
    uint8_t svId;               /* Satellite identifier */
    uint8_t sigId;              /* Signal identifier */
    uint8_t freqId;             /* Only used for GLONASS: frequency slot */
    uint16_t locktime;          /* Carrier phase locktime counter */
    uint8_t cno;                /* Carrier-to-noise density ratio [dB-Hz] */
    uint8_t prStdev;            /* Estimated pseudorange measurement standard deviation */
    uint8_t cpStdev;            /* Estimated carrier phase measurement standard deviation */
    uint8_t doStdev;            /* Estimated Doppler measurement standard deviation */
    uint8_t trkStat;            /* Tracking status bitfield */
    uint8_t reserved;           /* Reserved */
} zedf9p_rawx_meas_t;

/* RAWX Header Structure */
typedef struct {
    double rcvTow;              /* Measurement time of week in receiver local time [s] */
    uint16_t week;              /* GPS week number in receiver local time */
    int8_t leapS;               /* GPS leap seconds */
    uint8_t numMeas;            /* Number of measurements to follow */
    uint8_t recStat;            /* Receiver tracking status bitfield */
    uint8_t version;            /* Message version */
    uint8_t reserved[2];        /* Reserved */
} zedf9p_rawx_header_t;

/* Complete RAWX Data Structure */
typedef struct {
    zedf9p_rawx_header_t header;                /* RAWX message header */
    uint8_t num_measurements;                   /* Number of valid measurements */
    zedf9p_rawx_meas_t measurements[64];        /* Individual satellite measurements */
    bool valid;                                 /* Data validity flag */
} zedf9p_rawx_data_t;

/* Navigation PVT (Position, Velocity, Time) Data */
typedef struct {
    uint32_t iTOW;          /* GPS time of week of the navigation epoch */
    uint16_t year;          /* Year (UTC) */
    uint8_t month;          /* Month, range 1..12 (UTC) */
    uint8_t day;            /* Day of month, range 1..31 (UTC) */
    uint8_t hour;           /* Hour of day, range 0..23 (UTC) */
    uint8_t min;            /* Minute of hour, range 0..59 (UTC) */
    uint8_t sec;            /* Seconds of minute, range 0..60 (UTC) */
    uint8_t valid;          /* Validity flags */
    uint32_t tAcc;          /* Time accuracy estimate (UTC) */
    int32_t nano;           /* Fraction of second, range -1e9 .. 1e9 (UTC) */
    uint8_t fixType;        /* GNSSfix Type */
    uint8_t flags;          /* Fix status flags */
    uint8_t flags2;         /* Additional flags */
    uint8_t numSV;          /* Number of satellites used in Nav Solution */
    int32_t lon;            /* Longitude */
    int32_t lat;            /* Latitude */
    int32_t height;         /* Height above ellipsoid */
    int32_t hMSL;           /* Height above mean sea level */
    uint32_t hAcc;          /* Horizontal accuracy estimate */
    uint32_t vAcc;          /* Vertical accuracy estimate */
    int32_t velN;           /* NED north velocity */
    int32_t velE;           /* NED east velocity */
    int32_t velD;           /* NED down velocity */
    int32_t gSpeed;         /* Ground Speed (2-D) */
    int32_t headMot;        /* Heading of motion (2-D) */
    uint32_t sAcc;          /* Speed accuracy estimate */
    uint32_t headAcc;       /* Heading accuracy estimate */
    uint16_t pDOP;          /* Position DOP */
    uint8_t reserved1[6];   /* Reserved */
    int32_t headVeh;        /* Heading of vehicle (2-D) */
    int16_t magDec;         /* Magnetic declination */
    uint16_t magAcc;        /* Magnetic declination accuracy */
} zedf9p_nav_pvt_t;

/* High Precision Position Data */
typedef struct {
    uint8_t version;        /* Message version */
    uint8_t reserved1[2];   /* Reserved */
    uint8_t flags;          /* Invalid LLH flag */
    uint32_t iTOW;          /* GPS time of week */
    int32_t lon;            /* Longitude */
    int32_t lat;            /* Latitude */
    int32_t height;         /* Height above ellipsoid */
    int32_t hMSL;           /* Height above mean sea level */
    int8_t lonHp;           /* High precision longitude */
    int8_t latHp;           /* High precision latitude */
    int8_t heightHp;        /* High precision height above ellipsoid */
    int8_t hMSLHp;          /* High precision height above MSL */
    uint32_t hAcc;          /* Horizontal accuracy estimate */
    uint32_t vAcc;          /* Vertical accuracy estimate */
} zedf9p_nav_hpposllh_t;

/* Driver Configuration Structure */
typedef struct {
    zedf9p_interface_t interface;
    uint8_t i2c_address;
    uint32_t uart_baudrate;
    uint32_t timeout_ms;
    bool debug_enabled;
} zedf9p_config_t;

/* Driver Handle Structure */
typedef struct {
    zedf9p_config_t config;
    bool initialized;
    uint8_t rx_buffer[ZEDF9P_MAX_PAYLOAD_SIZE + UBX_HEADER_SIZE + UBX_CHECKSUM_SIZE];
    uint8_t tx_buffer[ZEDF9P_MAX_PAYLOAD_SIZE + UBX_HEADER_SIZE + UBX_CHECKSUM_SIZE];
    size_t rx_buffer_len;
    ubx_message_t current_message;
} zedf9p_handle_t;

/* Function Prototypes */

/**
 * @brief Initialize the ZED-F9P driver
 * @param handle Pointer to driver handle
 * @param config Pointer to configuration structure
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_init(zedf9p_handle_t *handle, const zedf9p_config_t *config);

/**
 * @brief Deinitialize the ZED-F9P driver
 * @param handle Pointer to driver handle
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_deinit(zedf9p_handle_t *handle);

/**
 * @brief Send UBX message to the module
 * @param handle Pointer to driver handle
 * @param msg_class UBX message class
 * @param msg_id UBX message ID
 * @param payload Pointer to payload data (can be NULL if length is 0)
 * @param payload_len Length of payload data
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_send_ubx_message(zedf9p_handle_t *handle, uint8_t msg_class, 
                                       uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);

/**
 * @brief Receive UBX message from the module
 * @param handle Pointer to driver handle
 * @param message Pointer to message structure to store received data
 * @param timeout_ms Timeout in milliseconds
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_receive_ubx_message(zedf9p_handle_t *handle, ubx_message_t *message, uint32_t timeout_ms);

/**
 * @brief Send UBX command and wait for ACK/NAK
 * @param handle Pointer to driver handle
 * @param msg_class UBX message class
 * @param msg_id UBX message ID
 * @param payload Pointer to payload data (can be NULL if length is 0)
 * @param payload_len Length of payload data
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_send_ubx_command(zedf9p_handle_t *handle, uint8_t msg_class, 
                                       uint8_t msg_id, const uint8_t *payload, uint16_t payload_len);

/**
 * @brief Poll for specific UBX message
 * @param handle Pointer to driver handle
 * @param msg_class UBX message class to poll
 * @param msg_id UBX message ID to poll
 * @param response Pointer to message structure to store response
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_poll_ubx_message(zedf9p_handle_t *handle, uint8_t msg_class, 
                                       uint8_t msg_id, ubx_message_t *response);

/**
 * @brief Get Navigation PVT data
 * @param handle Pointer to driver handle
 * @param pvt_data Pointer to PVT data structure
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_get_nav_pvt(zedf9p_handle_t *handle, zedf9p_nav_pvt_t *pvt_data);

/**
 * @brief Get High Precision Position data
 * @param handle Pointer to driver handle
 * @param hppos_data Pointer to high precision position data structure
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_get_nav_hpposllh(zedf9p_handle_t *handle, zedf9p_nav_hpposllh_t *hppos_data);

/**
 * @brief Set measurement rate
 * @param handle Pointer to driver handle
 * @param rate_ms Measurement rate in milliseconds
 * @param nav_rate Navigation rate (cycles)
 * @param time_ref Time reference (0=UTC, 1=GPS)
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_set_measurement_rate(zedf9p_handle_t *handle, uint16_t rate_ms, 
                                          uint16_t nav_rate, uint16_t time_ref);

/**
 * @brief Configure message output rate
 * @param handle Pointer to driver handle
 * @param msg_class UBX message class
 * @param msg_id UBX message ID
 * @param rate Output rate (0=disabled, 1=every solution, etc.)
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_configure_message_rate(zedf9p_handle_t *handle, uint8_t msg_class, 
                                            uint8_t msg_id, uint8_t rate);

/**
 * @brief Reset the module
 * @param handle Pointer to driver handle
 * @param reset_type Reset type (0x00=Hot start, 0x01=Warm start, 0xFF=Cold start)
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_reset(zedf9p_handle_t *handle, uint8_t reset_type);

/**
 * @brief Save current configuration to non-volatile memory
 * @param handle Pointer to driver handle
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_save_config(zedf9p_handle_t *handle);

/**
 * @brief Load configuration from non-volatile memory
 * @param handle Pointer to driver handle
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_load_config(zedf9p_handle_t *handle);

/**
 * @brief Get module version information
 * @param handle Pointer to driver handle
 * @param version_info Pointer to buffer to store version string
 * @param buffer_size Size of version_info buffer
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_get_version(zedf9p_handle_t *handle, char *version_info, size_t buffer_size);

/**
 * @brief Enable/disable RAWX message output
 * @param handle Pointer to driver handle
 * @param enable true to enable RAWX output, false to disable
 * @param output_rate Output rate (0=disabled, 1=every measurement epoch, etc.)
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_enable_rawx(zedf9p_handle_t *handle, bool enable, uint8_t output_rate);

/**
 * @brief Get RAWX raw measurement data
 * @param handle Pointer to driver handle
 * @param rawx_data Pointer to RAWX data structure
 * @param timeout_ms Timeout in milliseconds
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_get_rawx_data(zedf9p_handle_t *handle, zedf9p_rawx_data_t *rawx_data, uint32_t timeout_ms);

/**
 * @brief Poll for RAWX data (request single measurement)
 * @param handle Pointer to driver handle
 * @param rawx_data Pointer to RAWX data structure
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_poll_rawx_data(zedf9p_handle_t *handle, zedf9p_rawx_data_t *rawx_data);

/**
 * @brief Set frequency band configuration for GNSS signals
 * @param handle Pointer to driver handle
 * @param freq_band Frequency band selection (L1, L2, or dual L1+L2)
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_set_frequency_band(zedf9p_handle_t *handle, zedf9p_freq_band_t freq_band);

/**
 * @brief Configure GPS L1/L2 signal reception
 * @param handle Pointer to driver handle
 * @param enable_l1 Enable GPS L1 C/A signal
 * @param enable_l2 Enable GPS L2C signal
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_configure_gps_bands(zedf9p_handle_t *handle, bool enable_l1, bool enable_l2);

/**
 * @brief Configure Galileo E1/E5b signal reception
 * @param handle Pointer to driver handle
 * @param enable_e1 Enable Galileo E1 signal
 * @param enable_e5b Enable Galileo E5b signal
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_configure_galileo_bands(zedf9p_handle_t *handle, bool enable_e1, bool enable_e5b);

/**
 * @brief Configure BeiDou B1/B2 signal reception
 * @param handle Pointer to driver handle
 * @param enable_b1 Enable BeiDou B1I signal
 * @param enable_b2 Enable BeiDou B2I signal
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_configure_beidou_bands(zedf9p_handle_t *handle, bool enable_b1, bool enable_b2);

/**
 * @brief Configure GLONASS L1/L2 signal reception
 * @param handle Pointer to driver handle
 * @param enable_l1 Enable GLONASS L1 OF signal
 * @param enable_l2 Enable GLONASS L2 OF signal
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_configure_glonass_bands(zedf9p_handle_t *handle, bool enable_l1, bool enable_l2);

/**
 * @brief Get current frequency band configuration
 * @param handle Pointer to driver handle
 * @param gps_l1_enabled Pointer to store GPS L1 enable status
 * @param gps_l2_enabled Pointer to store GPS L2 enable status
 * @param gal_e1_enabled Pointer to store Galileo E1 enable status
 * @param gal_e5b_enabled Pointer to store Galileo E5b enable status
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_get_frequency_config(zedf9p_handle_t *handle, 
                                          bool *gps_l1_enabled, bool *gps_l2_enabled,
                                          bool *gal_e1_enabled, bool *gal_e5b_enabled);

/**
 * @brief Filter RAWX measurements by frequency band
 * @param rawx_data Pointer to RAWX data structure
 * @param freq_band Frequency band to filter (L1, L2, or dual)
 * @param filtered_count Pointer to store number of filtered measurements
 * @return zedf9p_error_t Error code
 */
zedf9p_error_t zedf9p_filter_rawx_by_frequency(zedf9p_rawx_data_t *rawx_data, 
                                              zedf9p_freq_band_t freq_band,
                                              uint8_t *filtered_count);

/**
 * @brief Calculate UBX checksum
 * @param data Pointer to data buffer
 * @param length Length of data
 * @param ck_a Pointer to store checksum A
 * @param ck_b Pointer to store checksum B
 */
void zedf9p_calculate_ubx_checksum(const uint8_t *data, uint16_t length, uint8_t *ck_a, uint8_t *ck_b);

/**
 * @brief Convert error code to string
 * @param error Error code
 * @return Pointer to error string
 */
const char* zedf9p_error_to_string(zedf9p_error_t error);

#ifdef __cplusplus
}
#endif

#endif /* ZEDF9P_H */