/**
 * @file zedf9p.h
 * @brief ZEDF9P GNSS Module Driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-09-10
 *
 * This driver supports the ZEDF9P GNSS module from u-blox with full UBX protocol support
 * including RAWX data acquisition. System agnostic design for Arduino, STM32, Zephyr, etc.
 */

#ifndef ZEDF9P_H
#define ZEDF9P_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

// Default I2C address for ZEDF9P
#define ZEDF9P_I2C_ADDR                     0x42

// UBX Protocol Constants
#define UBX_SYNCH_1                         0xB5
#define UBX_SYNCH_2                         0x62
#define UBX_MAX_PAYLOAD_SIZE                16384
#define UBX_HEADER_SIZE                     6
#define UBX_CHECKSUM_SIZE                   2

// UBX Message Classes
#define UBX_CLASS_NAV                       0x01
#define UBX_CLASS_RXM                       0x02
#define UBX_CLASS_INF                       0x04
#define UBX_CLASS_ACK                       0x05
#define UBX_CLASS_CFG                       0x06
#define UBX_CLASS_UPD                       0x09
#define UBX_CLASS_MON                       0x0A
#define UBX_CLASS_AID                       0x0B
#define UBX_CLASS_TIM                       0x0D
#define UBX_CLASS_ESF                       0x10
#define UBX_CLASS_MGA                       0x13
#define UBX_CLASS_LOG                       0x21
#define UBX_CLASS_SEC                       0x27
#define UBX_CLASS_HNR                       0x28

// UBX Message IDs - NAV Class
#define UBX_NAV_CLOCK                       0x22
#define UBX_NAV_DOP                         0x04
#define UBX_NAV_EOE                         0x61
#define UBX_NAV_GEOFENCE                    0x39
#define UBX_NAV_HPPOSECEF                   0x13
#define UBX_NAV_HPPOSLLH                    0x14
#define UBX_NAV_ODO                         0x09
#define UBX_NAV_ORB                         0x34
#define UBX_NAV_POSECEF                     0x01
#define UBX_NAV_POSLLH                      0x02
#define UBX_NAV_PVT                         0x07
#define UBX_NAV_RELPOSNED                   0x3C
#define UBX_NAV_RESETODO                    0x10
#define UBX_NAV_SAT                         0x35
#define UBX_NAV_SIG                         0x43
#define UBX_NAV_STATUS                      0x03
#define UBX_NAV_SVIN                        0x3B
#define UBX_NAV_TIMEBDS                     0x24
#define UBX_NAV_TIMEGAL                     0x25
#define UBX_NAV_TIMEGLO                     0x23
#define UBX_NAV_TIMEGPS                     0x20
#define UBX_NAV_TIMELS                      0x26
#define UBX_NAV_TIMEQZSS                    0x27
#define UBX_NAV_TIMEUTC                     0x21
#define UBX_NAV_VELECEF                     0x11
#define UBX_NAV_VELNED                      0x12

// UBX Message IDs - RXM Class
#define UBX_RXM_MEASX                       0x14
#define UBX_RXM_RAWX                        0x15
#define UBX_RXM_RLM                         0x59
#define UBX_RXM_RTCM                        0x32
#define UBX_RXM_SFRBX                       0x13
#define UBX_RXM_SPARTN                      0x33

// UBX Message IDs - CFG Class
#define UBX_CFG_ANT                         0x13
#define UBX_CFG_BATCH                       0x93
#define UBX_CFG_CFG                         0x09
#define UBX_CFG_DAT                         0x06
#define UBX_CFG_DGNSS                       0x70
#define UBX_CFG_GEOFENCE                    0x69
#define UBX_CFG_GNSS                        0x3E
#define UBX_CFG_HNR                         0x5C
#define UBX_CFG_INF                         0x02
#define UBX_CFG_ITFM                        0x39
#define UBX_CFG_LOGFILTER                   0x47
#define UBX_CFG_MSG                         0x01
#define UBX_CFG_NAV5                        0x24
#define UBX_CFG_NAVX5                       0x23
#define UBX_CFG_NMEA                        0x17
#define UBX_CFG_ODO                         0x1E
#define UBX_CFG_PM2                         0x3B
#define UBX_CFG_PMS                         0x86
#define UBX_CFG_PRT                         0x00
#define UBX_CFG_PWR                         0x57
#define UBX_CFG_RATE                        0x08
#define UBX_CFG_RINV                        0x34
#define UBX_CFG_RST                         0x04
#define UBX_CFG_RXM                         0x11
#define UBX_CFG_SBAS                        0x16
#define UBX_CFG_TMODE3                      0x71
#define UBX_CFG_TP5                         0x31
#define UBX_CFG_USB                         0x1B
#define UBX_CFG_VALGET                      0x8B
#define UBX_CFG_VALDEL                      0x8C
#define UBX_CFG_VALSET                      0x8A

// UBX Message IDs - ACK Class
#define UBX_ACK_NAK                         0x00
#define UBX_ACK_ACK                         0x01

// UBX Message IDs - MON Class
#define UBX_MON_COMMS                       0x36
#define UBX_MON_GNSS                        0x28
#define UBX_MON_HW                          0x09
#define UBX_MON_HW2                         0x0B
#define UBX_MON_IO                          0x02
#define UBX_MON_MSGPP                       0x06
#define UBX_MON_PATCH                       0x27
#define UBX_MON_RF                          0x38
#define UBX_MON_RXR                         0x21
#define UBX_MON_SPAN                        0x31
#define UBX_MON_TXBUF                       0x08
#define UBX_MON_VER                         0x04

// Configuration Key Values for CFG-VALSET/VALGET

// Communication Interface Configuration
#define UBLOX_CFG_I2C_ADDRESS               0x20510001
#define UBLOX_CFG_I2C_EXTENDEDTIMEOUT       0x10510002
#define UBLOX_CFG_I2C_ENABLED               0x10510003

#define UBLOX_CFG_UART1_BAUDRATE            0x40520001
#define UBLOX_CFG_UART1_STOPBITS            0x20520002
#define UBLOX_CFG_UART1_DATABITS            0x20520003
#define UBLOX_CFG_UART1_PARITY              0x20520004
#define UBLOX_CFG_UART1_ENABLED             0x10520005
#define UBLOX_CFG_UART1_REMAP               0x10520006

#define UBLOX_CFG_UART2_BAUDRATE            0x40530001
#define UBLOX_CFG_UART2_STOPBITS            0x20530002
#define UBLOX_CFG_UART2_DATABITS            0x20530003
#define UBLOX_CFG_UART2_PARITY              0x20530004
#define UBLOX_CFG_UART2_ENABLED             0x10530005
#define UBLOX_CFG_UART2_REMAP               0x10530006

// Message Output Configuration
#define UBLOX_CFG_MSGOUT_UBX_NAV_PVT_I2C    0x20910006
#define UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1  0x20910007
#define UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART2  0x20910008

#define UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_I2C   0x20910033
#define UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1 0x20910034
#define UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART2 0x20910035

#define UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_I2C  0x2091002E
#define UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1 0x2091002F
#define UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART2 0x20910030

#define UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_I2C   0x209102A4
#define UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_UART1 0x209102A5
#define UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_UART2 0x209102A6

#define UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_I2C  0x20910204
#define UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_UART1 0x20910205
#define UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_UART2 0x20910206

#define UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_I2C  0x20910231
#define UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_UART1 0x20910232
#define UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_UART2 0x20910233

#define UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_I2C   0x2091008D
#define UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1 0x2091008E
#define UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2 0x2091008F

#define UBLOX_CFG_MSGOUT_UBX_NAV_SAT_I2C    0x20910015
#define UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART1  0x20910016
#define UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART2  0x20910017

#define UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_I2C      0x20910065
#define UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_UART1    0x20910066
#define UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_UART2    0x20910067

#define UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_I2C    0x2091005B
#define UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1  0x2091005C
#define UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART2  0x2091005D

#define UBLOX_CFG_MSGOUT_UBX_MON_RF_I2C     0x20910359
#define UBLOX_CFG_MSGOUT_UBX_MON_RF_UART1   0x2091035A
#define UBLOX_CFG_MSGOUT_UBX_MON_RF_UART2   0x2091035B

// RTCM Message Output Configuration
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C   0x209102BD
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1 0x209102BE
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART2 0x209102BF

#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_I2C   0x209102CC
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1 0x209102CD
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART2 0x209102CE

#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_I2C   0x209102D1
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1 0x209102D2
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART2 0x209102D3

#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_I2C   0x20910318
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART1 0x20910319
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART2 0x2091031A

#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_I2C   0x209102D6
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART1 0x209102D7
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART2 0x209102D8

#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C   0x20910303
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1 0x20910304
#define UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART2 0x20910305

// Navigation and Positioning Configuration
#define UBLOX_CFG_RATE_MEAS                 0x30210001
#define UBLOX_CFG_RATE_NAV                  0x30210002
#define UBLOX_CFG_RATE_TIMEREF              0x20210003

#define UBLOX_CFG_NAVSPG_DYNMODEL           0x20110021
#define UBLOX_CFG_NAVSPG_FIXMODE            0x20110011
#define UBLOX_CFG_NAVSPG_UTCSTANDARD        0x20110023
#define UBLOX_CFG_NAVSPG_CONSTR_ALT         0x40110031
#define UBLOX_CFG_NAVSPG_CONSTR_ALT_VAR     0x40110032
#define UBLOX_CFG_NAVSPG_CONSTR_DGNS        0x20110034

// GNSS Signal Configuration
#define UBLOX_CFG_SIGNAL_GPS_ENA            0x1031001F
#define UBLOX_CFG_SIGNAL_GPS_L1CA_ENA       0x10310001
#define UBLOX_CFG_SIGNAL_GPS_L2C_ENA        0x10310003

#define UBLOX_CFG_SIGNAL_SBAS_ENA           0x10310020
#define UBLOX_CFG_SIGNAL_SBAS_L1CA_ENA      0x10310005

#define UBLOX_CFG_SIGNAL_GAL_ENA            0x10310021
#define UBLOX_CFG_SIGNAL_GAL_E1_ENA         0x10310007
#define UBLOX_CFG_SIGNAL_GAL_E5B_ENA        0x1031000A

#define UBLOX_CFG_SIGNAL_BDS_ENA            0x10310022
#define UBLOX_CFG_SIGNAL_BDS_B1_ENA         0x1031000D
#define UBLOX_CFG_SIGNAL_BDS_B2_ENA         0x1031000E

#define UBLOX_CFG_SIGNAL_QZSS_ENA           0x10310024
#define UBLOX_CFG_SIGNAL_QZSS_L1CA_ENA      0x10310012
#define UBLOX_CFG_SIGNAL_QZSS_L1S_ENA       0x10310014
#define UBLOX_CFG_SIGNAL_QZSS_L2C_ENA       0x10310015

#define UBLOX_CFG_SIGNAL_GLO_ENA            0x10310025
#define UBLOX_CFG_SIGNAL_GLO_L1_ENA         0x10310018
#define UBLOX_CFG_SIGNAL_GLO_L2_ENA         0x1031001A

// SBAS Configuration
#define UBLOX_CFG_SBAS_USE_TESTMODE         0x10360002
#define UBLOX_CFG_SBAS_USE_RANGING          0x10360003
#define UBLOX_CFG_SBAS_USE_DIFFCORR         0x10360004
#define UBLOX_CFG_SBAS_USE_INTEGRITY        0x10360005
#define UBLOX_CFG_SBAS_PRNSCANMASK          0x50360006

// RTK and Base Station Configuration
#define UBLOX_CFG_TMODE_MODE                0x20030001
#define UBLOX_CFG_TMODE_POS_TYPE            0x20030002
#define UBLOX_CFG_TMODE_ECEF_X              0x40030003
#define UBLOX_CFG_TMODE_ECEF_Y              0x40030004
#define UBLOX_CFG_TMODE_ECEF_Z              0x40030005
#define UBLOX_CFG_TMODE_ECEF_X_HP           0x20030006
#define UBLOX_CFG_TMODE_ECEF_Y_HP           0x20030007
#define UBLOX_CFG_TMODE_ECEF_Z_HP           0x20030008
#define UBLOX_CFG_TMODE_LAT                 0x40030009
#define UBLOX_CFG_TMODE_LON                 0x4003000A
#define UBLOX_CFG_TMODE_HEIGHT              0x4003000B
#define UBLOX_CFG_TMODE_LAT_HP              0x2003000C
#define UBLOX_CFG_TMODE_LON_HP              0x2003000D
#define UBLOX_CFG_TMODE_HEIGHT_HP           0x2003000E
#define UBLOX_CFG_TMODE_FIXED_POS_ACC       0x4003000F
#define UBLOX_CFG_TMODE_SVIN_MIN_DUR        0x40030010
#define UBLOX_CFG_TMODE_SVIN_ACC_LIMIT      0x40030011

// Timing Configuration
#define UBLOX_CFG_TP_PULSE_DEF              0x20050023
#define UBLOX_CFG_TP_PULSE_LENGTH_DEF       0x20050030
#define UBLOX_CFG_TP_ANT_CABLEDELAY         0x30050001
#define UBLOX_CFG_TP_PERIOD_TP1             0x40050002
#define UBLOX_CFG_TP_PERIOD_LOCK_TP1        0x40050003
#define UBLOX_CFG_TP_FREQ_TP1               0x40050024
#define UBLOX_CFG_TP_FREQ_LOCK_TP1          0x40050025
#define UBLOX_CFG_TP_LEN_TP1                0x40050004
#define UBLOX_CFG_TP_LEN_LOCK_TP1           0x40050005
#define UBLOX_CFG_TP_DUTY_TP1               0x5005002A
#define UBLOX_CFG_TP_DUTY_LOCK_TP1          0x5005002B
#define UBLOX_CFG_TP_USER_DELAY_TP1         0x40050006
#define UBLOX_CFG_TP_TP1_ENA                0x10050007
#define UBLOX_CFG_TP_SYNC_GNSS_TP1          0x10050008
#define UBLOX_CFG_TP_USE_LOCKED_TP1         0x10050009
#define UBLOX_CFG_TP_ALIGN_TO_TOW_TP1       0x1005000A
#define UBLOX_CFG_TP_POL_TP1                0x1005000B
#define UBLOX_CFG_TP_TIMEGRID_TP1           0x2005000C

// Power Management Configuration
#define UBLOX_CFG_PM_OPERATEMODE            0x20D00001
#define UBLOX_CFG_PM_POSUPDATEPERIOD        0x40D00002
#define UBLOX_CFG_PM_ACQPERIOD              0x40D00003
#define UBLOX_CFG_PM_GRIDOFFSET             0x40D00004
#define UBLOX_CFG_PM_ONTIME                 0x40D00005
#define UBLOX_CFG_PM_MINACQTIME             0x40D00006
#define UBLOX_CFG_PM_MAXACQTIME             0x40D00007

// Hardware Configuration
#define UBLOX_CFG_HW_ANT_CFG_VOLTCTRL       0x10A3002E
#define UBLOX_CFG_HW_ANT_CFG_SHORTDET       0x10A3002F
#define UBLOX_CFG_HW_ANT_CFG_SHORTDET_POL   0x10A30030
#define UBLOX_CFG_HW_ANT_CFG_OPENDET        0x10A30031
#define UBLOX_CFG_HW_ANT_CFG_OPENDET_POL    0x10A30032
#define UBLOX_CFG_HW_ANT_CFG_PWRDOWN        0x10A30033
#define UBLOX_CFG_HW_ANT_CFG_PWRDOWN_POL    0x10A30034
#define UBLOX_CFG_HW_ANT_CFG_RECOVER        0x10A30035
#define UBLOX_CFG_HW_ANT_SUP_SWITCH_PIN     0x20A30036
#define UBLOX_CFG_HW_ANT_SUP_SHORT_THR      0x20A30037
#define UBLOX_CFG_HW_ANT_SUP_OPEN_THR       0x20A30038

// Interference Mitigation
#define UBLOX_CFG_ITFM_ENABLE               0x1041000D
#define UBLOX_CFG_ITFM_ANTSETTING           0x20410001
#define UBLOX_CFG_ITFM_ENABLE_AUX           0x10410002

// Jamming Monitor Configuration
#define UBLOX_CFG_JAMMING_MONITOR           0x20150001

// SPARTN Configuration
#define UBLOX_CFG_SPARTN_USE_SOURCE         0x10590001
#define UBLOX_CFG_MSGOUT_SPARTN_I2C         0x20910366
#define UBLOX_CFG_MSGOUT_SPARTN_UART1       0x20910367
#define UBLOX_CFG_MSGOUT_SPARTN_UART2       0x20910368

// Add layer definitions to zedf9p.h
#define UBLOX_CFG_LAYER_RAM     0x01
#define UBLOX_CFG_LAYER_BBR     0x02  // Battery Backed RAM
#define UBLOX_CFG_LAYER_FLASH   0x04

// Dynamic Models
#define DYN_MODEL_PORTABLE                  0
#define DYN_MODEL_STATIONARY                2
#define DYN_MODEL_PEDESTRIAN                3
#define DYN_MODEL_AUTOMOTIVE                4
#define DYN_MODEL_SEA                       5
#define DYN_MODEL_AIRBORNE_1G               6
#define DYN_MODEL_AIRBORNE_2G               7
#define DYN_MODEL_AIRBORNE_4G               8
#define DYN_MODEL_WRIST                     9
#define DYN_MODEL_BIKE                      10

// Fix Modes
#define FIX_MODE_2D_ONLY                    1
#define FIX_MODE_3D_ONLY                    2
#define FIX_MODE_AUTO                       3

// Time Reference
#define TIME_REF_UTC                        0
#define TIME_REF_GPS                        1
#define TIME_REF_GLONASS                    2
#define TIME_REF_BEIDOU                     3
#define TIME_REF_GALILEO                    4

// Survey-in Mode
#define TMODE_DISABLED                      0
#define TMODE_SURVEY_IN                     1
#define TMODE_FIXED_MODE                    2

// Position Type
#define POS_TYPE_ECEF                       0
#define POS_TYPE_LLH                        1

// Power Management Modes
#define PM_OPERATEMODE_FULLPOWER            0
#define PM_OPERATEMODE_BALANCED             1
#define PM_OPERATEMODE_INTERVAL             2
#define PM_OPERATEMODE_AGGRESSIVE_1HZ       3
#define PM_OPERATEMODE_AGGRESSIVE_2HZ       4
#define PM_OPERATEMODE_AGGRESSIVE_4HZ       5

// UART Parity Options
#define ZEDF9P_UART_PARITY_NONE             0
#define ZEDF9P_UART_PARITY_ODD              1
#define ZEDF9P_UART_PARITY_EVEN             2

// UART Stop Bits
#define ZEDF9P_UART_STOPBITS_1              0
#define ZEDF9P_UART_STOPBITS_1_5            1
#define ZEDF9P_UART_STOPBITS_2              2

// UART Data Bits
#define UART_DATABITS_7                     0
#define UART_DATABITS_8                     1

// Antenna Configuration
#define ANT_VOLTAGE_CTRL_DISABLE            0
#define ANT_VOLTAGE_CTRL_ENABLE             1

#define ANT_DETECTION_DISABLE               0
#define ANT_DETECTION_ENABLE                1

// Time Pulse Polarity
#define TP_POL_FALLING                      0
#define TP_POL_RISING                       1

// Time Grid
#define TP_TIMEGRID_UTC                     0
#define TP_TIMEGRID_GPS                     1
#define TP_TIMEGRID_GLONASS                 2
#define TP_TIMEGRID_BEIDOU                  3
#define TP_TIMEGRID_GALILEO                 4

/**
 * @brief ZEDF9P driver return status codes.
 */
typedef enum {
    ZEDF9P_OK = 0,
    ZEDF9P_ERR_I2C = -1,
    ZEDF9P_ERR_UART = -2,
    ZEDF9P_ERR_TIMEOUT = -3,
    ZEDF9P_ERR_NULL = -4,
    ZEDF9P_ERR_CRC = -5,
    ZEDF9P_ERR_INVALID_ARG = -6,
    ZEDF9P_ERR_BUFFER_FULL = -7,
    ZEDF9P_ERR_NO_DATA = -8,
    ZEDF9P_ERR_NACK = -9,
    ZEDF9P_ERR_INVALID_MESSAGE = -10,
    ZEDF9P_ERR_PARSE_ERROR = -11
} zedf9p_status_t;

/**
 * @brief Communication interface types.
 */
typedef enum {
    ZEDF9P_INTERFACE_I2C,
    ZEDF9P_INTERFACE_UART
} zedf9p_interface_type_t;

/**
 * @brief UBX message structure.
 */
typedef struct {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t payload[UBX_MAX_PAYLOAD_SIZE];
    bool valid;
    uint32_t timestamp_ms;
} ubx_message_t;

/**
 * @brief Pending message tracking structure.
 */
typedef struct {
    uint8_t msg_class;
    uint8_t msg_id;
    uint8_t expected_class;
    uint8_t expected_id;
    uint32_t timestamp_ms;
    uint32_t timeout_ms;
    bool waiting_for_ack;
    bool waiting_for_response;
    bool ack_received;
    bool response_received;
    bool is_ack;
} pending_message_t;

/**
 * @brief Navigation Position Velocity Time (PVT) data structure.
 */
typedef struct {
    uint32_t i_tow;                 // GPS time of week (ms)
    uint16_t year;                  // Year (UTC)
    uint8_t month;                  // Month (UTC)
    uint8_t day;                    // Day (UTC)
    uint8_t hour;                   // Hour (UTC)
    uint8_t min;                    // Minute (UTC)
    uint8_t sec;                    // Second (UTC)
    uint8_t valid;                  // Validity flags
    uint32_t t_acc;                 // Time accuracy estimate (ns)
    int32_t nano;                   // Fraction of second (ns)
    uint8_t fix_type;               // GNSS fix type
    uint8_t flags;                  // Fix status flags
    uint8_t flags2;                 // Additional flags
    uint8_t num_sv;                 // Number of satellites
    int32_t lon;                    // Longitude (1e-7 deg)
    int32_t lat;                    // Latitude (1e-7 deg)
    int32_t height;                 // Height above ellipsoid (mm)
    int32_t h_msl;                  // Height above MSL (mm)
    uint32_t h_acc;                 // Horizontal accuracy (mm)
    uint32_t v_acc;                 // Vertical accuracy (mm)
    int32_t vel_n;                  // North velocity (mm/s)
    int32_t vel_e;                  // East velocity (mm/s)
    int32_t vel_d;                  // Down velocity (mm/s)
    int32_t g_speed;                // Ground speed (mm/s)
    int32_t head_mot;               // Heading of motion (1e-5 deg)
    uint32_t s_acc;                 // Speed accuracy (mm/s)
    uint32_t head_acc;              // Heading accuracy (1e-5 deg)
    uint16_t p_dop;                 // Position DOP (0.01)
    uint8_t flags3;                 // Additional flags
    uint8_t reserved1[5];           // Reserved
    int32_t head_veh;               // Heading of vehicle (1e-5 deg)
    int16_t mag_dec;                // Magnetic declination (1e-2 deg)
    uint16_t mag_acc;               // Magnetic declination accuracy (1e-2 deg)
} zedf9p_nav_pvt_t;

/**
 * @brief High Precision Position (HPPOSLLH) data structure.
 */
typedef struct {
    uint8_t version;                // Message version
    uint8_t reserved1[2];           // Reserved
    uint8_t invalid_llh;            // Invalid position flag
    uint32_t i_tow;                 // GPS time of week (ms)
    int32_t lon;                    // Longitude (1e-7 deg)
    int32_t lat;                    // Latitude (1e-7 deg)
    int32_t height;                 // Height above ellipsoid (mm)
    int32_t h_msl;                  // Height above MSL (mm)
    int8_t lon_hp;                  // High precision longitude (1e-9 deg)
    int8_t lat_hp;                  // High precision latitude (1e-9 deg)
    int8_t height_hp;               // High precision height above ellipsoid (0.1 mm)
    int8_t h_msl_hp;                // High precision height above MSL (0.1 mm)
    uint32_t h_acc;                 // Horizontal accuracy (0.1 mm)
    uint32_t v_acc;                 // Vertical accuracy (0.1 mm)
} zedf9p_nav_hpposllh_t;

/**
 * @brief Raw measurement data (RAWX) structure.
 */
typedef struct {
    double pr_mes;                  // Pseudorange measurement (m)
    double cp_mes;                  // Carrier phase measurement (cycles)
    float do_mes;                   // Doppler measurement (Hz)
    uint8_t gnss_id;                // GNSS identifier
    uint8_t sv_id;                  // Satellite identifier
    uint8_t sig_id;                 // Signal identifier
    uint8_t freq_id;                // Frequency identifier
    uint16_t lock_time;             // Carrier phase lock time (ms)
    uint8_t cno;                    // Carrier-to-noise ratio (dB-Hz)
    uint8_t pr_stdev;               // Pseudorange standard deviation
    uint8_t cp_stdev;               // Carrier phase standard deviation
    uint8_t do_stdev;               // Doppler standard deviation
    uint8_t trk_stat;               // Tracking status
    uint8_t reserved3;              // Reserved
} zedf9p_rawx_meas_t;

/**
 * @brief RAWX message data structure.
 */
typedef struct {
    double rc_tow;                  // Measurement time of week (s)
    uint16_t week;                  // GPS week number
    int8_t leap_s;                  // GPS leap seconds
    uint8_t num_meas;               // Number of measurements
    uint8_t rec_stat;               // Receiver tracking status
    uint8_t version;                // Message version
    uint8_t reserved1[2];           // Reserved
    zedf9p_rawx_meas_t meas[32];    // Measurement data (max 32 measurements)
} zedf9p_rawx_t;

/**
 * @brief Navigation Clock Solution (CLOCK) data structure.
 */
typedef struct {
    uint32_t i_tow;                 // GPS time of week (ms)
    int32_t clk_b;                  // Clock bias (ns)
    int32_t clk_d;                  // Clock drift (ns/s)
    uint32_t t_acc;                 // Time accuracy estimate (ns)
    uint32_t f_acc;                 // Frequency accuracy estimate (ps/s)
} zedf9p_nav_clock_t;

/**
 * @brief High Precision Position ECEF (HPPOSECEF) data structure.
 */
typedef struct {
    uint8_t version;                // Message version
    uint8_t reserved1[3];           // Reserved
    uint32_t i_tow;                 // GPS time of week (ms)
    int32_t ecef_x;                 // ECEF X coordinate (cm)
    int32_t ecef_y;                 // ECEF Y coordinate (cm)
    int32_t ecef_z;                 // ECEF Z coordinate (cm)
    int8_t ecef_x_hp;               // High precision ECEF X (0.1 mm)
    int8_t ecef_y_hp;               // High precision ECEF Y (0.1 mm)
    int8_t ecef_z_hp;               // High precision ECEF Z (0.1 mm)
    uint8_t invalid_ecef;           // Invalid ECEF position flag
    uint32_t p_acc;                 // Position accuracy (0.1 mm)
} zedf9p_nav_hpposecef_t;

/**
 * @brief Time UTC (TIMEUTC) data structure.
 */
typedef struct {
    uint32_t i_tow;                 // GPS time of week (ms)
    uint32_t t_acc;                 // Time accuracy estimate (ns)
    int32_t nano;                   // Fraction of second (ns)
    uint16_t year;                  // Year (UTC)
    uint8_t month;                  // Month (UTC)
    uint8_t day;                    // Day (UTC)
    uint8_t hour;                   // Hour (UTC)
    uint8_t min;                    // Minute (UTC)
    uint8_t sec;                    // Second (UTC)
    uint8_t valid;                  // Validity flags
} zedf9p_nav_timeutc_t;

/**
 * @brief Subframe Buffer (SFRBX) data structure.
 */
typedef struct {
    uint8_t gnss_id;                // GNSS identifier
    uint8_t sv_id;                  // Satellite identifier
    uint8_t reserved1;              // Reserved
    uint8_t freq_id;                // Frequency identifier (GLONASS only)
    uint8_t num_words;              // Number of data words
    uint8_t chn;                    // Channel number
    uint8_t version;                // Message version
    uint8_t reserved2;              // Reserved
    uint32_t dwrd[10];              // Data words (max 10 words for GPS/Galileo)
} zedf9p_sfrbx_t;

/**
 * @brief MON-VER version information structure.
 */
typedef struct {
    char sw_version[30];            // Software version string
    char hw_version[10];            // Hardware version string
    char rom_version[30];           // ROM version string
    uint8_t num_extensions;         // Number of extension strings
    char extensions[30][30];        // Extension strings (max 30)
    bool valid;                     // Data validity flag
} zedf9p_mon_ver_t;

/**
 * @brief UART configuration structure.
 */
typedef struct {
    uint32_t baudrate;
    uint8_t databits;
    uint8_t stopbits;
    uint8_t parity;
    bool enabled;
    bool remap;
} zedf9p_uart_config_t;

/**
 * @brief I2C configuration structure.
 */
typedef struct {
    uint8_t address;
    bool extended_timeout;
    bool enabled;
} zedf9p_i2c_config_t;

/**
 * @brief GNSS signal configuration structure.
 */
typedef struct {
    bool beidou_enabled;
    bool beidou_b1_enabled;
    bool beidou_b2_enabled;
    bool galileo_enabled;
    bool galileo_e1_enabled;
    bool galileo_e5b_enabled;
    bool glonass_enabled;
    bool glonass_l1_enabled;
    bool glonass_l2_enabled;
    bool gps_enabled;
    bool gps_l1ca_enabled;
    bool gps_l2c_enabled;
    bool qzss_enabled;
    bool qzss_l1ca_enabled;
    bool qzss_l1s_enabled;
    bool qzss_l2c_enabled;
    bool sbas_enabled;
    bool sbas_l1ca_enabled;

} zedf9p_gnss_config_t;

/**
 * @brief SBAS configuration structure.
 */
typedef struct {
    bool use_testmode;
    bool use_ranging;
    bool use_diffcorr;
    bool use_integrity;
    uint64_t prn_scan_mask;
} zedf9p_sbas_config_t;

/**
 * @brief Time mode configuration structure.
 */
typedef struct {
    uint8_t mode;                   // TMODE_*
    uint8_t pos_type;               // POS_TYPE_*
    // ECEF coordinates
    int32_t ecef_x;                 // cm
    int32_t ecef_y;                 // cm
    int32_t ecef_z;                 // cm
    int8_t ecef_x_hp;               // 0.1 mm
    int8_t ecef_y_hp;               // 0.1 mm
    int8_t ecef_z_hp;               // 0.1 mm
    // LLH coordinates
    int32_t lat;                    // 1e-7 deg
    int32_t lon;                    // 1e-7 deg
    int32_t height;                 // cm
    int8_t lat_hp;                  // 1e-9 deg
    int8_t lon_hp;                  // 1e-9 deg
    int8_t height_hp;               // 0.1 mm
    // Survey-in parameters
    uint32_t fixed_pos_acc;         // 0.1 mm
    uint32_t svin_min_dur;          // s
    uint32_t svin_acc_limit;        // 0.1 mm
} zedf9p_tmode_config_t;

/**
 * @brief Time pulse configuration structure.
 */
typedef struct {
    uint32_t period_us;             // Period in microseconds
    uint32_t period_lock_us;        // Period when locked in microseconds
    uint32_t freq_hz;               // Frequency in Hz
    uint32_t freq_lock_hz;          // Frequency when locked in Hz
    uint32_t length_us;             // Pulse length in microseconds
    uint32_t length_lock_us;        // Pulse length when locked in microseconds
    double duty_percent;            // Duty cycle in percent
    double duty_lock_percent;       // Duty cycle when locked in percent
    int32_t user_delay_ns;          // User delay in nanoseconds
    bool enabled;
    bool sync_gnss;
    bool use_locked;
    bool align_to_tow;
    uint8_t polarity;               // TP_POL_*
    uint8_t time_grid;              // TP_TIMEGRID_*
    int32_t ant_cable_delay_ns;     // Antenna cable delay in nanoseconds
} zedf9p_timepulse_config_t;

/**
 * @brief Power management configuration structure.
 */
typedef struct {
    uint8_t operate_mode;           // PM_OPERATEMODE_*
    uint32_t pos_update_period_ms;  // Position update period
    uint32_t acq_period_ms;         // Acquisition period
    uint32_t grid_offset_ms;        // Grid offset
    uint32_t on_time_ms;            // On time
    uint32_t min_acq_time_ms;       // Minimum acquisition time
    uint32_t max_acq_time_ms;       // Maximum acquisition time
} zedf9p_power_config_t;

/**
 * @brief Antenna configuration structure.
 */
typedef struct {
    bool voltage_ctrl;              // Antenna voltage control
    bool short_det;                 // Short detection
    bool short_det_pol;             // Short detection polarity
    bool open_det;                  // Open detection
    bool open_det_pol;              // Open detection polarity
    bool power_down;                // Power down
    bool power_down_pol;            // Power down polarity
    bool recover;                   // Recovery
    uint8_t switch_pin;             // Switch pin configuration
    uint8_t short_thr;              // Short threshold
    uint8_t open_thr;               // Open threshold
} zedf9p_antenna_config_t;

/**
 * @brief Platform interface abstraction for ZEDF9P driver.
 */
typedef struct {
    int (*i2c_write)(uint8_t dev_addr, const uint8_t *data, uint16_t len);
    int (*i2c_read)(uint8_t dev_addr, uint8_t *data, uint16_t len);
    int (*uart_write)(const uint8_t *data, uint16_t len);
    int (*uart_read)(uint8_t *data, uint16_t len);
    void (*delay_ms)(uint32_t ms);
    uint32_t (*get_millis)(void);
    void (*debug_print)(const char *message);  // Add this
} zedf9p_interface_t;

// Enhanced message parsing states
typedef enum {
    UBX_STATE_SYNC1,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LENGTH1,
    UBX_STATE_LENGTH2,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CHECKSUM1,
    UBX_STATE_CHECKSUM2
} ubx_parse_state_t;

/**
 * @brief Message callback function pointer type.
 */
typedef void (*zedf9p_message_callback_t)(const ubx_message_t *message, void *user_data);

/**
 * @brief ZEDF9P driver instance.
 */
typedef struct {
    uint8_t i2c_address;
    zedf9p_interface_type_t interface_type;
    zedf9p_interface_t io;
    bool initialized;

    // Enhanced message parsing state
    uint8_t rx_buffer[UBX_MAX_PAYLOAD_SIZE + UBX_HEADER_SIZE + UBX_CHECKSUM_SIZE];
    uint16_t rx_buffer_idx;
    ubx_parse_state_t parse_state;
    ubx_message_t current_message;
    bool ubx_7f_check_disabled;  // Flag to disable 7F check for RAWX

    // Parsing state variables (moved from static to make reentrant)
    uint16_t expected_length;
    uint16_t bytes_remaining;
    uint8_t calculated_ck_a;
    uint8_t calculated_ck_b;

    // Pending message tracking for enhanced communication
    pending_message_t pending_msg;
    ubx_message_t pending_response;

    // Latest received data
    zedf9p_nav_pvt_t nav_pvt;
    zedf9p_nav_hpposllh_t nav_hpposllh;
    zedf9p_rawx_t rawx;
    zedf9p_mon_ver_t mon_ver;
    zedf9p_nav_clock_t nav_clock;
    zedf9p_nav_hpposecef_t nav_hpposecef;
    zedf9p_nav_timeutc_t nav_timeutc;
    zedf9p_sfrbx_t sfrbx;

    // Data flags
    bool nav_pvt_valid;
    bool nav_hpposllh_valid;
    bool rawx_valid;
    bool mon_ver_valid;
    bool nav_clock_valid;
    bool nav_hpposecef_valid;
    bool nav_timeutc_valid;
    bool sfrbx_valid;

    // Callback system
    zedf9p_message_callback_t nav_pvt_callback;
    zedf9p_message_callback_t nav_hpposllh_callback;
    zedf9p_message_callback_t rawx_callback;
    zedf9p_message_callback_t nav_clock_callback;
    zedf9p_message_callback_t nav_hpposecef_callback;
    zedf9p_message_callback_t nav_timeutc_callback;
    zedf9p_message_callback_t sfrbx_callback;
    zedf9p_message_callback_t generic_callback;
    void *callback_user_data;

    // Configuration
    uint16_t measurement_rate_ms;
    uint16_t navigation_rate;
    uint8_t dynamic_model;
} zedf9p_t;

/**
 * @brief Human-readable description of a ZEDF9P status code.
 * @param status The status code to translate.
 * @return A string describing the error.
 */
const char* zedf9p_status_error(zedf9p_status_t status);

// Initialization and Configuration Functions

/**
 * @brief Initialize the ZEDF9P driver.
 * @param dev Pointer to driver handle
 * @param interface_type Communication interface to use
 * @param address I2C address (ignored for UART)
 * @param io Interface function pointers
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_init(zedf9p_t *dev, zedf9p_interface_type_t interface_type,
                           uint8_t address, zedf9p_interface_t io);

/**
 * @brief Perform a software reset of the module.
 * @param dev Pointer to initialized driver struct
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_soft_reset(const zedf9p_t *dev);

/**
 * @brief Perform a hardware reset of the module.
 * @param dev Pointer to initialized driver struct
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_hard_reset(const zedf9p_t *dev);

/**
 * @brief Set the measurement rate and navigation rate.
 * @param dev Pointer to initialized driver struct
 * @param layer_mask Configuration layer (RAM, BBR, or FLASH)
 * @param meas_rate_ms Measurement rate in milliseconds
 * @param nav_rate Navigation rate (measurements per navigation solution)
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_set_measurement_rate(zedf9p_t *dev, uint8_t layer_mask, uint16_t meas_rate_ms, uint16_t nav_rate);

/**
 * @brief Set the dynamic model for navigation.
 * @param dev Pointer to initialized driver struct
 * @param layer_mask Configuration layer (RAM, BBR, or FLASH)
 * @param model Dynamic model (DYN_MODEL_*)
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_set_dynamic_model(zedf9p_t *dev, uint8_t layer_mask, uint8_t model);

/**
 * @brief Enable or disable a UBX message.
 * @param dev Pointer to initialized driver struct
 * @param msg_class UBX message class
 * @param msg_id UBX message ID
 * @param rate Output rate (0 = disabled, >0 = every N navigation solutions)
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_set_message_rate(const zedf9p_t *dev, uint8_t layer_mask, uint8_t msg_class, uint8_t msg_id, uint8_t rate);

/**
 * @brief Configure a setting using CFG-VALSET.
 * @param dev Pointer to initialized driver struct
 * @param layer_mask Configuration layer (RAM, BBR, or FLASH)
 * @param key_id Configuration key ID
 * @param value Configuration value
 * @param size Size of value in bytes (1, 2, 4, or 8)
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_config_set_val(const zedf9p_t *dev, uint8_t layer_mask, uint32_t key_id, uint64_t value, uint8_t size);

/**
 * @brief Read a configuration value using CFG-VALGET.
 * @param dev Pointer to initialized driver struct
 * @param layer_mask Configuration layer (RAM, BBR, or FLASH)
 * @param key_id Configuration key ID
 * @param value Pointer to store the retrieved value
 * @param size Size of value in bytes (1, 2, 4, or 8)
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_config_get_val(zedf9p_t *dev, uint8_t layer_mask, uint32_t key_id, uint64_t *value, uint8_t size);

// Data Acquisition Functions

/**
 * @brief Process incoming data and parse UBX messages.
 * @param dev Pointer to initialized driver struct
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_process_data(zedf9p_t *dev);

/**
 * @brief Get the latest PVT data.
 * @param dev Pointer to initialized driver struct
 * @param pvt Pointer to store PVT data
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_get_pvt(zedf9p_t *dev, zedf9p_nav_pvt_t *pvt);

/**
 * @brief Get the latest high precision position data.
 * @param dev Pointer to initialized driver struct
 * @param hppos Pointer to store high precision position data
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_get_hpposllh(zedf9p_t *dev, zedf9p_nav_hpposllh_t *hppos);

/**
 * @brief Get the latest RAWX data.
 * @param dev Pointer to initialized driver struct
 * @param rawx Pointer to store RAWX data
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_get_rawx(zedf9p_t *dev, zedf9p_rawx_t *rawx);

zedf9p_status_t zedf9p_get_clock(zedf9p_t *dev, zedf9p_nav_clock_t *clock);
zedf9p_status_t zedf9p_get_hpposecef(zedf9p_t *dev, zedf9p_nav_hpposecef_t *hpposecef);
zedf9p_status_t zedf9p_get_timeutc(zedf9p_t *dev, zedf9p_nav_timeutc_t *timeutc);
zedf9p_status_t zedf9p_get_sfrbx(zedf9p_t *dev, zedf9p_sfrbx_t *sfrbx);

/**
 * @brief Check if new PVT data is available.
 * @param dev Pointer to initialized driver struct
 * @return true if new data is available, false otherwise
 */
bool zedf9p_is_pvt_available(const zedf9p_t *dev);

/**
 * @brief Check if new high precision position data is available.
 * @param dev Pointer to initialized driver struct
 * @return true if new data is available, false otherwise
 */
bool zedf9p_is_hpposllh_available(const zedf9p_t *dev);

/**
 * @brief Check if new RAWX data is available.
 * @param dev Pointer to initialized driver struct
 * @return true if new data is available, false otherwise
 */
bool zedf9p_is_rawx_available(const zedf9p_t *dev);

bool zedf9p_is_clock_available(const zedf9p_t *dev);
bool zedf9p_is_hpposecef_available(const zedf9p_t *dev);
bool zedf9p_is_timeutc_available(const zedf9p_t *dev);
bool zedf9p_is_sfrbx_available(const zedf9p_t *dev);

// Callback Functions

/**
 * @brief Register a callback for PVT messages.
 * @param dev Pointer to initialized driver struct
 * @param callback Callback function pointer
 * @param user_data User data pointer to pass to callback
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_register_pvt_callback(zedf9p_t *dev, zedf9p_message_callback_t callback, void *user_data);

/**
 * @brief Register a callback for high precision position messages.
 * @param dev Pointer to initialized driver struct
 * @param callback Callback function pointer
 * @param user_data User data pointer to pass to callback
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_register_hpposllh_callback(zedf9p_t *dev, zedf9p_message_callback_t callback, void *user_data);

/**
 * @brief Register a callback for RAWX messages.
 * @param dev Pointer to initialized driver struct
 * @param callback Callback function pointer
 * @param user_data User data pointer to pass to callback
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_register_rawx_callback(zedf9p_t *dev, zedf9p_message_callback_t callback, void *user_data);

zedf9p_status_t zedf9p_register_clock_callback(zedf9p_t *dev, zedf9p_message_callback_t callback, void *user_data);
zedf9p_status_t zedf9p_register_hpposecef_callback(zedf9p_t *dev, zedf9p_message_callback_t callback, void *user_data);
zedf9p_status_t zedf9p_register_timeutc_callback(zedf9p_t *dev, zedf9p_message_callback_t callback, void *user_data);
zedf9p_status_t zedf9p_register_sfrbx_callback(zedf9p_t *dev, zedf9p_message_callback_t callback, void *user_data);

/**
 * @brief Register a generic callback for all UBX messages.
 * @param dev Pointer to initialized driver struct
 * @param callback Callback function pointer
 * @param user_data User data pointer to pass to callback
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_register_generic_callback(zedf9p_t *dev, zedf9p_message_callback_t callback, void *user_data);

// Enhanced UBX Message Functions

/**
 * @brief Send a raw UBX message.
 * @param dev Pointer to initialized driver struct
 * @param msg_class UBX message class
 * @param msg_id UBX message ID
 * @param payload Payload data
 * @param payload_len Payload length
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_send_ubx_message(const zedf9p_t *dev, uint8_t msg_class, uint8_t msg_id,
                                       const uint8_t *payload, uint16_t payload_len);

/**
 * @brief Send a UBX message and wait for acknowledgment.
 * @param dev Pointer to initialized driver struct
 * @param msg_class UBX message class
 * @param msg_id UBX message ID
 * @param payload Payload data
 * @param payload_len Payload length
 * @param timeout_ms Timeout in milliseconds
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_send_ubx_message_with_ack(zedf9p_t *dev, uint8_t msg_class, uint8_t msg_id,
                                                const uint8_t *payload, uint16_t payload_len, uint32_t timeout_ms);

/**
 * @brief Send a UBX message and wait for a specific response.
 * @param dev Pointer to initialized driver struct
 * @param send_class UBX message class to send
 * @param send_id UBX message ID to send
 * @param payload Payload data
 * @param payload_len Payload length
 * @param expected_class Expected response message class
 * @param expected_id Expected response message ID (0xFF for any ACK/NAK)
 * @param timeout_ms Timeout in milliseconds
 * @param response Pointer to store the response message
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_send_ubx_message_with_response(zedf9p_t *dev, uint8_t send_class, uint8_t send_id,
                                                     const uint8_t *payload, uint16_t payload_len,
                                                     uint8_t expected_class, uint8_t expected_id,
                                                     uint32_t timeout_ms, ubx_message_t *response);

/**
 * @brief Poll a UBX message and wait for response.
 * @param dev Pointer to initialized driver struct
 * @param msg_class UBX message class
 * @param msg_id UBX message ID
 * @param timeout_ms Timeout in milliseconds
 * @param response Pointer to store the response message
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_poll_ubx_message(zedf9p_t *dev, uint8_t msg_class, uint8_t msg_id,
                                        uint32_t timeout_ms, ubx_message_t *response);

// Utility Functions

/**
 * @brief Calculate UBX checksum for a message.
 * @param data Message data (class + id + length + payload)
 * @param len Length of data
 * @param ck_a Pointer to store checksum A
 * @param ck_b Pointer to store checksum B
 */
void zedf9p_calculate_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);

/**
 * @brief Validate UBX message checksum.
 * @param data Message data
 * @param len Length of data
 * @param ck_a Checksum A
 * @param ck_b Checksum B
 * @return true if checksum is valid, false otherwise
 */
bool zedf9p_validate_checksum(const uint8_t *data, uint16_t len, uint8_t ck_a, uint8_t ck_b);

// Version Information Functions

/**
 * @brief Poll for MON-VER version information.
 * @param dev Pointer to initialized driver struct
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_poll_mon_ver(zedf9p_t *dev);

/**
 * @brief Get the latest MON-VER data.
 * @param dev Pointer to initialized driver struct
 * @param mon_ver Pointer to store version data
 * @return zedf9p_status_t Error code
 */
zedf9p_status_t zedf9p_get_mon_ver(zedf9p_t *dev, zedf9p_mon_ver_t *mon_ver);

/**
 * @brief Check if MON-VER data is available.
 * @param dev Pointer to initialized driver struct
 * @return true if new data is available, false otherwise
 */
bool zedf9p_is_mon_ver_available(const zedf9p_t *dev);

zedf9p_status_t zedf9p_config_uart(const zedf9p_t *dev, uint8_t layer_mask, uint8_t port, const zedf9p_uart_config_t *config);

zedf9p_status_t zedf9p_config_i2c(const zedf9p_t *dev, uint8_t layer_mask, const zedf9p_i2c_config_t *config);

zedf9p_status_t zedf9p_config_gnss_signals(const zedf9p_t *dev, uint8_t layer_mask, const zedf9p_gnss_config_t *config);

zedf9p_status_t zedf9p_config_sbas(const zedf9p_t *dev, uint8_t layer_mask, const zedf9p_sbas_config_t *config);

zedf9p_status_t zedf9p_config_time_mode(const zedf9p_t *dev, uint8_t layer_mask, const zedf9p_tmode_config_t *config);

zedf9p_status_t zedf9p_config_time_pulse(const zedf9p_t *dev, uint8_t layer_mask, const zedf9p_timepulse_config_t *config);

zedf9p_status_t zedf9p_config_power_management(const zedf9p_t *dev, uint8_t layer_mask, const zedf9p_power_config_t *config);

zedf9p_status_t zedf9p_config_antenna(const zedf9p_t *dev, uint8_t layer_mask, const zedf9p_antenna_config_t *config);

zedf9p_status_t zedf9p_config_interference_mitigation(const zedf9p_t *dev, uint8_t layer_mask, bool enable, uint8_t ant_setting, bool enable_aux);

zedf9p_status_t zedf9p_config_jamming_monitor(const zedf9p_t *dev, uint8_t layer_mask, bool enable);

// RTCM Configuration Functions
zedf9p_status_t zedf9p_enable_rtcm_message(const zedf9p_t *dev, uint8_t layer_mask, uint16_t message_type, uint8_t rate, uint8_t interface_mask);

zedf9p_status_t zedf9p_config_rtcm_base_station(const zedf9p_t *dev, uint8_t layer_mask, bool enable_1005, bool enable_1077,
                                               bool enable_1087, bool enable_1097, bool enable_1127, bool enable_1230);

// SPARTN Configuration Functions
zedf9p_status_t zedf9p_config_spartn(const zedf9p_t *dev, uint8_t layer_mask, bool enable, bool use_source);

// Survey-in Functions
zedf9p_status_t zedf9p_start_survey_in(const zedf9p_t *dev, uint8_t layer_mask, uint32_t min_duration_s, uint32_t accuracy_limit_mm);

zedf9p_status_t zedf9p_stop_survey_in(const zedf9p_t *dev, uint8_t layer_mask);

zedf9p_status_t zedf9p_set_fixed_base_position(const zedf9p_t *dev, uint8_t layer_mask, double lat_deg, double lon_deg,
                                              double height_m, uint32_t accuracy_mm);

zedf9p_status_t zedf9p_set_fixed_base_position_ecef(const zedf9p_t *dev, uint8_t layer_mask, double x_m, double y_m,
                                                   double z_m, uint32_t accuracy_mm);

zedf9p_status_t zedf9p_disable_7f_check(zedf9p_t *dev, bool disabled);

#ifdef __cplusplus
}
#endif

#endif // ZEDF9P_H