#ifndef BQ76905_H
#define BQ76905_H

#include <stdint.h>
#include <stdbool.h>
#include "bq76905_platform.h"

// Device I2C Address (7-bit)
#define BQ76905_I2C_ADDR1    0x10
#define BQ76905_I2C_ADDR1    0x11

// Direct Commands
#define BQ76905_REG_SAFETY_ALERT_A      0x02
#define BQ76905_REG_SAFETY_STATUS_A     0x03
#define BQ76905_REG_SAFETY_ALERT_B      0x04
#define BQ76905_REG_SAFETY_STATUS_B     0x05
#define BQ76905_REG_BATTERY_STATUS      0x12
#define BQ76905_REG_CELL_1_VOLTAGE      0x14
#define BQ76905_REG_CELL_2_VOLTAGE      0x16
#define BQ76905_REG_CELL_3_VOLTAGE      0x18
#define BQ76905_REG_CELL_4_VOLTAGE      0x1A
#define BQ76905_REG_CELL_5_VOLTAGE      0x1C
#define BQ76905_REG_REG18_VOLTAGE       0x22
#define BQ76905_REG_VSS_VOLTAGE         0x24
#define BQ76905_REG_STACK_VOLTAGE       0x26
#define BQ76905_REG_INT_TEMPERATURE     0x28
#define BQ76905_REG_TS_MEASUREMENT      0x2A
#define BQ76905_REG_RAW_CURRENT         0x36
#define BQ76905_REG_CURRENT             0x3A
#define BQ76905_REG_CC1_CURRENT         0x3C

// Status and Control Registers
#define BQ76905_REG_ALARM_STATUS        0x62
#define BQ76905_REG_ALARM_ENABLE        0x66
#define BQ76905_REG_FET_CONTROL         0x68
#define BQ76905_REG_REGOUT_CONTROL      0x69
#define BQ76905_REG_DSG_PWM_CONTROL     0x6A
#define BQ76905_REG_CHG_PWM_CONTROL     0x6C

// Command-Only Subcommands
#define BQ76905_REG_RESET_PASSQ         0x0005
#define BQ76905_REG_EXIT_DEEPSLEEP      0x000E
#define BQ76905_REG_DEEPSLEEP           0x000F
#define BQ76905_REG_SHUTDOWN            0x0010
#define BQ76905_REG_RESET               0x0012
#define BQ76905_REG_FET_ENABLE          0x0022
#define BQ76905_REG_SEAL                0x0030
#define BQ76905_REG_SET_CFGUPDATE       0x0090
#define BQ76905_REG_EXIT_CFGUPDATE      0x0092
#define BQ76905_REG_SLEEP_ENABLE        0x0099
#define BQ76905_REG_SLEEP_DISABLE       0x009A

// Subcommands with Data
#define BQ76905_REG_DEVICE_NUMBER       0x0001
#define BQ76905_REG_FW_VERSION          0x0002
#define BQ76905_REG_HW_VERSION          0x0003
#define BQ76905_REG_PASSQ               0x0004
#define BQ76905_REG_SECURITY_KEYS       0x0035
#define BQ76905_REG_CB_ACTIVE_CELLS     0x0083
#define BQ76905_REG_PROG_TIMER          0x0094
#define BQ76905_REG_PROT_RECOVERY       0x009B


// Safety Alert A Register Bits
typedef struct {
    bool occ            : 1;
    bool ocd2           : 1;
    bool ocd1           : 1;
    bool scd            : 1;
    bool cuv            : 1;
    bool cov            : 1;
} BQ76905_Safety_Alert_A_t;

// Safety Status A Register Bits
typedef struct {
    bool regout         : 1;
    bool curlatch       : 1;
    bool occ            : 1;
    bool ocd2           : 1;
    bool ocd1           : 1;
    bool scd            : 1;
    bool cuv            : 1;
    bool cov            : 1;
} BQ76905_Safety_Status_A_t;

// Safety Alert B Register Bits
typedef struct {
    bool vss            : 1;
    bool vref           : 1;
    bool hwd            : 1;
    bool otint          : 1;
    bool utc            : 1;
    bool utd            : 1;
    bool otc            : 1;
    bool otd            : 1;
} BQ76905_Safety_Alert_B_t;

// Safety Status B Register Bits
typedef struct {
    bool vss            : 1;
    bool vref           : 1;
    bool hwd            : 1;
    bool otint          : 1;
    bool utc            : 1;
    bool utd            : 1;
    bool otc            : 1;
    bool otd            : 1;
} BQ76905_Safety_Status_B_t;

// Battery Status Register Bits
typedef struct {
    bool chgdetflag     : 1;
    bool dsg            : 1;
    bool chg            : 1;
    bool alertpin       : 1;
    bool cfgupdate      : 1;
    bool sleep_en       : 1;
    bool por            : 1;
    bool fet_en         : 1;
    uint8_t sec         : 2;
    bool ss             : 1;
    bool sa             : 1;
    bool deepsleep      : 1;
    bool sleep          : 1;
} BQ76905_Battery_Status_t;

// Alarm Status Register Bits
typedef struct {
    bool por            : 1;
    bool cdtoggle       : 1;
    bool initcomp       : 1;
    bool timer_alarm    : 1;
    bool sleep          : 1;
    bool wake           : 1;
    bool adscan         : 1;
    bool fullscan       : 1;
    bool cb             : 1;
    bool shutv          : 1;
    bool xdsg           : 1;
    bool xchg           : 1;
    bool sab            : 1;
    bool saa            : 1;
    bool ssb            : 1;
    bool ssa            : 1;
} BQ76905_Alarm_Status_t;

// Alarm Enable Register Bits
typedef struct {
    bool por            : 1;
    bool cdtoggle       : 1;
    bool initcomp       : 1;
    bool timer_alarm    : 1;
    bool sleep          : 1;
    bool wake           : 1;
    bool adscan         : 1;
    bool fullscan       : 1;
    bool cb             : 1;
    bool shutv          : 1;
    bool xdsg           : 1;
    bool xchg           : 1;
    bool sab            : 1;
    bool saa            : 1;
    bool ssb            : 1;
    bool ssa            : 1;
} BQ76905_Alarm_Enable_t;

// Function Definitions

// Initialization
BQ76905_Status bq76905_init(uint8_t i2c_addr);

// Safety Status Functions
BQ76905_Status bq76905_read_safety_alert_a(BQ76905_Safety_Alert_A_t *alert);
BQ76905_Status bq76905_read_safety_status_a(BQ76905_Safety_Status_A_t *status);
BQ76905_Status bq76905_read_safety_alert_b(BQ76905_Safety_Alert_B_t *alert);
BQ76905_Status bq76905_read_safety_status_b(BQ76905_Safety_Status_B_t *status);
BQ76905_Status bq76905_read_battery_status(BQ76905_Battery_Status_t *status);
BQ76905_Status bq76905_read_alarm_status(BQ76905_Alarm_Status_t *status);

// Measurement Functions
BQ76905_Status bq76905_read_cell_voltage(uint8_t cell_num, uint16_t *millivolts);
BQ76905_Status bq76905_read_stack_voltage(uint16_t *millivolts);
BQ76905_Status bq76905_read_internal_temp(int16_t *celsius);
BQ76905_Status bq76905_read_ts_adc(uint16_t *adc_codes);
BQ76905_Status bq76905_read_current(int16_t *current);
BQ76905_Status bq76905_read_cc1_current(int16_t *current);

// Control Functions
BQ76905_Status bq76905_enable_alarm(BQ76905_Alarm_Enable_t *enable);
BQ76905_Status bq76905_set_fet_control(bool chg_on, bool dsg_on, bool chg_off, bool dsg_off);
BQ76905_Status bq76905_set_regout_control(uint8_t voltage, bool enable);
BQ76905_Status bq76905_set_dsg_pwm_control(uint8_t time_off, uint8_t time_on, bool enable);
BQ76905_Status bq76905_set_chg_pwm_control(uint8_t time_off, uint8_t time_on, bool enable);
BQ76905_Status bq76905_enter_shutdown(void);
BQ76905_Status bq76905_exit_shutdown(void);
BQ76905_Status bq76905_enter_deepsleep(void);
BQ76905_Status bq76905_exit_deepsleep(void);

#endif // BQ76905_H