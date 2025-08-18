/**
 * @file zedf9p.h
 * @brief ZED-F9P GNSS Module Driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-15
 * 
 * @details Comprehensive driver for u-blox ZED-F9P GNSS module
 * @details Supports both I2C and UART communication protocols
 */

#ifndef ZEDF9P_H
#define ZEDF9P_H

#ifdef __cplusplus
// extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/**
 *@brief Driver Configuration
 */
#define ZEDF9P_DEFAULT_I2C_ADDR     0x42
#define ZEDF9P_DEFAULT_UART_BAUD    38400
#define ZEDF9P_MAX_PAYLOAD_SIZE     1024
#define ZEDF9P_DEFAULT_TIMEOUT_MS   1000
#define ZEDF9P_SYNC_TIMEOUT_MS      5000

/**
 * @brief UBX Protocol Constants
 */
#define UBX_SYNC_CHAR_1     0xB5
#define UBX_SYNC_CHAR_2     0x62
#define UBX_HEADER_SIZE     6
#define UBX_CHECKSUM_SIZE   2

/**
 * @brief Communication Interface Types
 */
typedef enum {
    ZEDF9P_INTERFACE_I2C,
    ZEDF9P_INTERFACE_UART
} zedf9p_interface_type_t;

/**
 * @brief ZEDF9P driver return status codes
 */
typedef enum {
    ZEDF9P_OK = 0,
    ZEDF9P_ERR_I2C = -1,
    ZEDF9P_ERR_UART = -2,
    ZEDF9P_ERR_TIMEOUT = -3,
    ZEDF9P_ERR_NULL = -4,
    ZEDF9P_ERR_CRC = -5,
    ZEDF9P_ERR_INVALID_ARG = -6,
    ZEDF9P_ERR_INVALID_RESPONSE = -7,
    ZEDF9P_ERR_BUFFER_OVERFLOW = -8,
    ZEDF9P_ERR_NOT_INITIALIZED = -9,
    ZEDF9P_ERR_CHECKSUM = -10
} zedf9p_status_t;

/**
 * @brief UBX message classes
 */
typedef enum {
    ZEDF9P_CLASS_NAV = 0x01,    ///< Navigation Results Messages
    ZEDF9P_CLASS_RXM = 0x02,    ///< Receiver Manager Messages
    ZEDF9P_CLASS_INF = 0x04,    ///< Information Messages
    ZEDF9P_CLASS_ACK = 0x05,    ///< Ack/Nak Messages
    ZEDF9P_CLASS_CFG = 0x06,    ///< Configuration Input Messages
    ZEDF9P_CLASS_UPD = 0x09,    ///< Firmware Update Messages
    ZEDF9P_CLASS_MON = 0x0A,    ///< Monitoring Messages
    ZEDF9P_CLASS_TIM = 0x0D,    ///< Timing Messages
    ZEDF9P_CLASS_ESF = 0x10,    ///< External Sensor Fusion Messages
    ZEDF9P_CLASS_MGA = 0x13,    ///< Multiple GNSS Assistance Messages
    ZEDF9P_CLASS_LOG = 0x21,    ///< Logging Messages
    ZEDF9P_CLASS_SEC = 0x27,    ///< Security Feature Messages
    ZEDF9P_CLASS_HNR = 0x28     ///< High Rate Navigation Results Messages
} zedf9p_ubx_class_t;

/**
 * @brief Common UBX message IDs
 */
typedef enum {
    // NAV Class Messages
    ZEDF9P_ID_NAV_PVT = 0x07,       ///< Position, Velocity, Time
    UBX_NAV_HPPOSLLH = 0x14,        ///< High Precision Geodetic Position Solution
    ZEDF9P_ID_NAV_STATUS = 0x03,    ///< Receiver Navigation Status
    ZEDF9P_ID_NAV_SAT = 0x35,       ///< Satellite Information
    ZEDF9P_ID_NAV_DOP = 0x04,       ///< Dilution of Precision
    ZEDF9P_ID_NAV_POSLLH = 0x02,    ///< Position in LLH
    ZEDF9P_ID_NAV_VELNED = 0x12,    ///< Velocity in NED

    // CFG Class Messages
    UBX_CFG_PRT = 0x00,             ///< Port Configuration
    UBX_CFG_MSG = 0x01,             ///< Message Configuration
    UBX_CFG_INF = 0x02,             ///< Information Message Configuration
    UBX_CFG_RST = 0x04,             ///< Reset Receiver / Clear Backup Data Structure
    UBX_CFG_DAT = 0x06,             ///< Set User-defined Datum
    UBX_CFG_RATE = 0x08,            ///< Navigation/Measurement Rate Settings
    UBX_CFG_CFG = 0x09,             ///< Clear, Save and Load configurations
    UBX_CFG_SBAS = 0x16,            ///< SBAS Configuration
    UBX_CFG_GNSS = 0x3E,            ///< GNSS system configuration
    UBX_CFG_VALSET = 0x8A,          ///< Set configuration values
    UBX_CFG_VALGET = 0x8B,          ///< Get configuration values
    UBX_CFG_VALDEL = 0x8C,          ///< Delete configuration values

    // MON Class Messages
    ZEDF9P_ID_MON_VER = 0x04,       ///< Receiver/Software Version
    ZEDF9P_ID_MON_HW = 0x09,        ///< Hardware Status

    // ACK Class Messages
    ZEDF9P_ID_ACK_ACK = 0x01,       ///< Message Acknowledged
    ZEDF9P_ID_ACK_NAK = 0x00        ///< Message Not-Acknowledged
} zedf9p_ubx_msg_id_t;

/**
 * @brief GNSS Signal ID Constants for L1/L2 Frequency Bands
 */
typedef enum {
    GNSS_GPS_L1CA = 0x00,           ///< GPS L1 C/A
    GNSS_GPS_L2CL = 0x03,           ///< GPS L2 CL
    GNSS_GPS_L2CM = 0x04,           ///< GPS L2 CM
    GNSS_GALILEO_E1C = 0x00,        ///< Galileo E1 C
    GNSS_GALILEO_E1B = 0x01,        ///< Galileo E1 B
    GNSS_GALILEO_E5BI = 0x05,       ///< Galileo E5b I
    GNSS_GALILEO_E5BQ = 0x06,       ///< Galileo E5b Q
    GNSS_BEIDOU_B1I = 0x00,         ///< BeiDou B1I D1
    GNSS_BEIDOU_B1I_D2 = 0x01,      ///< BeiDou B1I D2
    GNSS_BEIDOU_B2I = 0x02,         ///< BeiDou B2I D1
    GNSS_BEIDOU_B2I_D2 = 0x03,      ///< BeiDou B2I D2
    GNSS_GLONASS_L1OF = 0x00,       ///< GLONASS L1 OF
    GNSS_GLONASS_L2OF = 0x02,       ///< GLONASS L2 OF
} zedf9p_signal_id_t;

/**
 * @brief GNSS System ID Constants
 */
typedef enum {
    GNSS_ID_GPS = 0x00,             ///< GPS
    GNSS_ID_SBAS = 0x01,            ///< SBAS
    GNSS_ID_GALILEO = 0x02,         ///< Galileo
    GNSS_ID_BEIDOU = 0x03,          ///< BeiDou
    GNSS_ID_IMES = 0x04,            ///< IMES
    GNSS_ID_QZSS = 0x05,            ///< QZSS
    GNSS_ID_GLONASS = 0x06,         ///< GLONASS
} zedf9p_system_id_t;

/**
 * @brief Configuration Key IDs for L1/L2 Band Selection
 */
typedef enum {
    CFG_SIGNAL_GPS_ENA = 0x1031001F,        ///< GPS enable
    CFG_SIGNAL_GPS_L1CA_ENA = 0x10310001,   ///< GPS L1C/A enable
    CFG_SIGNAL_GPS_L2C_ENA = 0x10310003,    ///< GPS L2C enable
    CFG_SIGNAL_GAL_ENA = 0x10310021,        ///< Galileo enable
    CFG_SIGNAL_GAL_E1_ENA = 0x10310007,     ///< Galileo E1 enable
    CFG_SIGNAL_GAL_E5B_ENA = 0x1031000A,    ///< Galileo E5b enable
    CFG_SIGNAL_BDS_ENA = 0x10310022,        ///< BeiDou enable
    CFG_SIGNAL_BDS_B1_ENA = 0x1031000D,     ///< BeiDou B1I enable
    CFG_SIGNAL_BDS_B2_ENA = 0x1031000E,     ///< BeiDou B2I enable
    CFG_SIGNAL_GLO_ENA = 0x10310025,        ///< GLONASS enable
    CFG_SIGNAL_GLO_L1_ENA = 0x10310018,     ///< GLONASS L1 enable
    CFG_SIGNAL_GLO_L2_ENA = 0x1031001A,     ///< GLONASS L2 enable
} zedf9p_config_key_id_t;

/**
 * @brief Frequency Band Selection
 */
typedef enum {
    ZEDF9P_FREQ_L1_ONLY,        ///< L1 band only
    ZEDF9P_FREQ_L2_ONLY,        ///< L2 band only
    ZEDF9P_FREQ_L1_L2_DUAL      ///< Dual L1+L2 bands
} zedf9p_freq_band_t;

/**
 * @brief UBX Message Structure
 */
typedef struct {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t payload[ZEDF9P_MAX_PAYLOAD_SIZE];
    uint8_t checksum_a;
    uint8_t checksum_b;
    bool valid;
} ubx_message_t;

/**
 * @brief RAWX Measurement Data Structure
 */
typedef struct {
    double prMes;               ///< Pseudorange measurement [m]
    double cpMes;               ///< Carrier phase measurement [L1 cycles]
    float doMes;                ///< Doppler measurement [Hz]
    uint8_t gnssId;             ///< GNSS identifier
    uint8_t svId;               ///< Satellite identifier
    uint8_t sigId;              ///< Signal identifier
    uint8_t freqId;             ///< Only used for GLONASS: frequency slot
    uint16_t locktime;          ///< Carrier phase locktime counter
    uint8_t cno;                ///< Carrier-to-noise density ratio [dB-Hz]
    uint8_t prStdev;            ///< Estimated pseudorange measurement standard deviation
    uint8_t cpStdev;            ///< Estimated carrier phase measurement standard deviation
    uint8_t doStdev;            ///< Estimated Doppler measurement standard deviation
    uint8_t trkStat;            ///< Tracking status bitfield
} zedf9p_rawx_meas_t;

/**
 * @brief RAWX Header Structure
 */
typedef struct {
    double rcvTow;              ///< Measurement time of week in receiver local time [s]
    uint16_t week;              ///< GPS week number in receiver local time
    int8_t leapS;               ///< GPS leap seconds
    uint8_t numMeas;            ///< Number of measurements to follow
    uint8_t recStat;            ///< Receiver tracking status bitfield
    uint8_t version;            ///< Message version
} zedf9p_rawx_header_t;

/**
 * @brief Complete RAWX Data Structure
 */
typedef struct {
    zedf9p_rawx_header_t header;                ///< RAWX message header
    uint8_t num_measurements;                   ///< Number of valid measurements
    zedf9p_rawx_meas_t measurements[64];        ///< Individual satellite measurements
    bool valid;                                 ///< Data validity flag
} zedf9p_rawx_data_t;

/**
 * @brief Navigation PVT (Position, Velocity, Time) Data
 */
typedef struct {
    uint32_t iTOW;          ///< GPS time of week of the navigation epoch
    uint16_t year;          ///< Year (UTC)
    uint8_t month;          ///< Month, range 1..12 (UTC)
    uint8_t day;            ///< Day of month, range 1..31 (UTC)
    uint8_t hour;           ///< Hour of day, range 0..23 (UTC)
    uint8_t min;            ///< Minute of hour, range 0..59 (UTC)
    uint8_t sec;            ///< Seconds of minute, range 0..60 (UTC)
    uint8_t valid;          ///< Validity flags
    uint32_t tAcc;          ///< Time accuracy estimate (UTC)
    int32_t nano;           ///< Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t fixType;        ///< GNSSfix Type
    uint8_t flags;          ///< Fix status flags
    uint8_t flags2;         ///< Additional flags
    uint8_t numSV;          ///< Number of satellites used in Nav Solution
    int32_t lon;            ///< Longitude
    int32_t lat;            ///< Latitude
    int32_t height;         ///< Height above ellipsoid
    int32_t hMSL;           ///< Height above mean sea level
    uint32_t hAcc;          ///< Horizontal accuracy estimate
    uint32_t vAcc;          ///< Vertical accuracy estimate
    int32_t velN;           ///< NED north velocity
    int32_t velE;           ///< NED east velocity
    int32_t velD;           ///< NED down velocity
    int32_t gSpeed;         ///< Ground Speed (2-D)
    int32_t headMot;        ///< Heading of motion (2-D)
    uint32_t sAcc;          ///< Speed accuracy estimate
    uint32_t headAcc;       ///< Heading accuracy estimate
    uint16_t pDOP;          ///< Position DOP
    int32_t headVeh;        ///< Heading of vehicle (2-D)
    int16_t magDec;         ///< Magnetic declination
    uint16_t magAcc;        ///< Magnetic declination accuracy

    // Convenience fields (converted from raw values)
    double lat_deg;             // Latitude in degrees
    double lon_deg;             // Longitude in degrees
    double height_msl_mm;       // Height above MSL in mm
    double ground_speed_ms;     // Ground speed in m/s
    double heading_deg;         // Heading in degrees
} zedf9p_pvt_t;

/**
 * @brief High Precision Position Data
 */
typedef struct {
    uint8_t version;        ///< Message version
    uint8_t flags;          ///< Invalid LLH flag
    uint32_t iTOW;          ///< GPS time of week
    int32_t lon;            ///< Longitude
    int32_t lat;            ///< Latitude
    int32_t height;         ///< Height above ellipsoid
    int32_t hMSL;           ///< Height above mean sea level
    int8_t lonHp;           ///< High precision longitude
    int8_t latHp;           ///< High precision latitude
    int8_t heightHp;        ///< High precision height above ellipsoid
    int8_t hMSLHp;          ///< High precision height above MSL
    uint32_t hAcc;          ///< Horizontal accuracy estimate
    uint32_t vAcc;          ///< Vertical accuracy estimate
} zedf9p_nav_hpposllh_t;

/**
 * @brief Driver I2C Interface Configuration
 */
typedef struct {
    int (*i2c_write)(uint8_t dev_addr, const uint8_t *data, uint16_t len);
    int (*i2c_read)(uint8_t dev_addr, uint8_t *data, uint16_t len);
    int (*i2c_available)(uint8_t dev_addr);
} zedf9p_interface_i2c_t;

typedef struct {
    int (*uart_write)(const uint8_t *data, uint16_t len);
    int (*uart_read)(uint8_t *data, uint16_t len);
    int (*uart_available)();
    void (*uart_flush)();

    uint8_t rx_buffer[ZEDF9P_MAX_PAYLOAD_SIZE + UBX_HEADER_SIZE + UBX_CHECKSUM_SIZE];
    uint8_t tx_buffer[ZEDF9P_MAX_PAYLOAD_SIZE + UBX_HEADER_SIZE + UBX_CHECKSUM_SIZE];
    size_t rx_buffer_len;
} zedf9p_interface_uart_t;

/**
 * @brief Driver Interface Structure
 */
typedef struct {
    void (*delay_ms)(uint32_t ms);
    void (*delay_us)(uint32_t us);
    zedf9p_interface_type_t interface_type;


    zedf9p_interface_i2c_t i2c;
    zedf9p_interface_uart_t uart;
} zedf9p_interface_t;

/**
 * @brief Driver Handle Structure
 */
typedef struct {
    uint8_t i2c_address;
    uint32_t uart_baudrate;
    zedf9p_interface_t io;
    bool debug_enabled;
    bool initialized;

    uint32_t message_timeout_ms;
    uint32_t last_nav_pvt_time;

    zedf9p_pvt_t last_pvt;
    bool pvt_data_valid;
} zedf9p_t;

///< @section Public Function Prototypes

/**
 * @brief Initialize the ZED-F9P driver
 */
zedf9p_status_t zedf9p_init(zedf9p_t *dev, const zedf9p_interface_t *io, uint8_t address);

/**
 * @brief Send UBX message to the module
 */
zedf9p_status_t zedf9p_send_ubx_message(zedf9p_t *dev, uint8_t msg_class,
                                       uint8_t msg_id, uint8_t *payload, uint16_t payload_len);

/**
 * @brief Receive UBX message from the module
 */
zedf9p_status_t zedf9p_receive_ubx_message(zedf9p_t *dev, ubx_message_t *message, uint32_t timeout_ms);


/**
 * @brief Send UBX command and wait for ACK/NAK
 */
zedf9p_status_t zedf9p_send_ubx_command(zedf9p_t *dev, uint8_t msg_class,
                                       uint8_t msg_id, uint8_t *payload, uint16_t payload_len);

/**
 * @brief Poll for specific UBX message
 */
zedf9p_status_t zedf9p_poll_ubx_message(zedf9p_t *dev, uint8_t msg_class,
                                       uint8_t msg_id, ubx_message_t *response);

/**
 * @brief Get Navigation PVT data
 */
zedf9p_status_t zedf9p_get_nav_pvt(zedf9p_t *dev, zedf9p_pvt_t *pvt_data);

/**
 * @brief Get High Precision Position data
 */
zedf9p_status_t zedf9p_get_nav_hpposllh(zedf9p_t *dev, zedf9p_nav_hpposllh_t *hppos_data);

/**
 * @brief Set measurement rate
 */
zedf9p_status_t zedf9p_set_measurement_rate(zedf9p_t *dev, uint16_t rate_ms,
                                          uint16_t nav_rate, uint16_t time_ref);

/**
 * @brief Configure message output rate
 */
zedf9p_status_t zedf9p_configure_message_rate(zedf9p_t *dev, uint8_t msg_class,
                                            uint8_t msg_id, uint8_t rate);

/**
 * @brief Reset the module
 */
zedf9p_status_t zedf9p_reset(zedf9p_t *dev, uint8_t reset_type);

/**
 * @brief Save current configuration to non-volatile memory
 */
zedf9p_status_t zedf9p_save_config(zedf9p_t *dev);

/**
 * @brief Load configuration from non-volatile memory
 */
zedf9p_status_t zedf9p_load_config(zedf9p_t *dev);

/**
 * @brief Get module version information
 */
zedf9p_status_t zedf9p_get_version(zedf9p_t *dev, char *version_info, size_t buffer_size);

/**
 * @brief Enable/disable RAWX message output
 */
zedf9p_status_t zedf9p_enable_rawx(zedf9p_t *dev, bool enable, uint8_t output_rate);

/**
 * @brief Get RAWX raw measurement data
 */
zedf9p_status_t zedf9p_get_rawx_data(zedf9p_t *dev, zedf9p_rawx_data_t *rawx_data, uint32_t timeout_ms);

/**
 * @brief Poll for RAWX data (request single measurement)
 */
zedf9p_status_t zedf9p_poll_rawx_data(zedf9p_t *dev, zedf9p_rawx_data_t *rawx_data);

/**
 * @brief Set frequency band configuration for GNSS signals
 */
zedf9p_status_t zedf9p_set_frequency_band(zedf9p_t *dev, zedf9p_freq_band_t freq_band);

/**
 * @brief Configure GPS L1/L2 signal reception
 */
zedf9p_status_t zedf9p_configure_gps_bands(zedf9p_t *dev, bool enable_l1, bool enable_l2);

/**
 * @brief Configure Galileo E1/E5b signal reception
 */
zedf9p_status_t zedf9p_configure_galileo_bands(zedf9p_t *dev, bool enable_e1, bool enable_e5b);

/**
 * @brief Configure BeiDou B1/B2 signal reception
 */
zedf9p_status_t zedf9p_configure_beidou_bands(zedf9p_t *dev, bool enable_b1, bool enable_b2);

/**
 * @brief Configure GLONASS L1/L2 signal reception
 */
zedf9p_status_t zedf9p_configure_glonass_bands(zedf9p_t *dev, bool enable_l1, bool enable_l2);

/**
 * @brief Get current frequency band configuration
 */
zedf9p_status_t zedf9p_get_frequency_config(zedf9p_t *dev,
                                          bool *gps_l1_enabled, bool *gps_l2_enabled,
                                          bool *gal_e1_enabled, bool *gal_e5b_enabled);

/**
 * @brief Filter RAWX measurements by frequency band
 */
zedf9p_status_t zedf9p_filter_rawx_by_frequency(zedf9p_rawx_data_t *rawx_data,
                                              zedf9p_freq_band_t freq_band,
                                              uint8_t *filtered_count);

/**
 * @brief Calculate UBX checksum
 */
void zedf9p_calculate_ubx_checksum(const uint8_t *data, uint16_t length, uint8_t *ck_a, uint8_t *ck_b);

/**
 * @brief Convert error code to string
 */
const char* zedf9p_error_to_string(zedf9p_status_t error);

#ifdef __cplusplus
// }
#endif

#endif /* ZEDF9P_H */