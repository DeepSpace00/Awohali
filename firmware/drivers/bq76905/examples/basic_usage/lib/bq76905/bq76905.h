/**
 * @file bq76905.h
 * @brief BQ76905 Battery Monitor and Protector Driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-13
 *
 * This driver supports the BQ76905 IC from Texas Instruments
 */

#ifndef BQ76905_H
#define BQ76905_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
// extern "C" {
#endif

#define BQ76905_I2C_DEFAULT_ADDRESS 0x10        // Default I2C address
#define BQ76905_I2C_COMMAND_ADDRESS 0x3E        // I2C command address
#define BQ76905_I2C_DATA_BUFFER_ADDRESS 0x40    // I2C data buffer address

/**
 * @brief BQ76905 IC driver return status codes.
 */
typedef enum {
    BQ76905_OK = 0,
    BQ76905_ERR_I2C = -1,
    BQ76905_ERR_TIMEOUT = -2,
    BQ76905_ERR_NULL = -3,
    BQ76905_ERR_INVALID_ARG = -4,
} bq76905_status_t;

/**
 * @brief BQ76905 security control
 */
typedef enum {
    BQ76905_SEC_NOT_INTIALIZED = 0x00,  ///< Device has not initialized yet.
    BQ76905_SEC_FULLACCESS = 0x01,      ///< Device is in FULLACCESS mode.
    BQ76905_SEC_UNUSED = 0x02,          ///< Unused
    BQ76905_SEC_SEALED = 0x03,          ///< Device is in SEALED mode.
} bq76905_security_state_t;

/**
 * @brief BQ76905 REGOUT LDO voltage control
 */
typedef enum {
    BQ76905_REGOUTV_1V8 = 0x00,     ///< REGOUT LDO is set to 1.8V
    BQ76905_REGOUTV_2V5 = 0x04,     ///< REGOUT LDO is set to 2.8V
    BQ76905_REGOUTV_3V0 = 0x05,     ///< REGOUT LDO is set to 3.0V
    BQ76905_REGOUTV_3V3 = 0x06,     ///< REGOUT LDO is set to 3.3V
    BQ76905_REGOUTV_5V0 = 0x07,     ///< REGOUT LDO is set to 5.0V
} bq76905_regout_voltage_ctrl;

/**
 * @breif BQ76905 REGOUT disable timer
 */
typedef enum {
    BQ76905_REGOUT_DIS_DLY_NONE = 0x00,     ///< Zero delay
    BQ76905_REGOUT_DIS_DLY_250MS = 0x01,    ///< 250ms delay
    BQ76905_REGOUT_DIS_DLY_1S = 0x02,       ///< 1s delay
    BQ76905_REGOUT_DIS_DLY_4S = 0x03,       ///< 4s delay
} bq76905_regout_disable_delay_t;

/**
 * @brief BQ76905 Alarms
 */
typedef enum {
    BQ76905_ALARM_SSA = 15,     ///< Safety Status A() alarm
    BQ76905_ALARM_SSB = 14,     ///< Safety Status B() alarm
    BQ76905_ALARM_SAA = 13,     ///< Safety Alert A() alarm
    BQ76905_ALARM_SAB = 12,     ///< Safety Alert B() alarm
    BQ76905_ALARM_XCHG = 11,    ///< Charge driver alarm
    BQ76905_ALARM_XDSG = 10,    ///< Discharge driver alarm
    BQ76905_ALARM_SHUTV = 9,    ///< Shutdown Voltage alarm
    BQ76905_ALARM_CB = 8,       ///< Cell balancing active alarm
    BQ76905_ALARM_FULLSCAN = 7, ///< Full scan complete alarm
    BQ76905_ALARM_ADSCAN = 6,   ///< ADC measurement scan complete alarm
    BQ76905_ALARM_WAKE = 5,     ///< Device wakened alarm
    BQ76905_ALARM_SLEEP = 4,    ///< Device in SLEEP alarm
    BQ76905_ALARM_TIMER = 3,    ///< Programmable timer expired alarm
    BQ76905_ALARM_INITCOMP = 2, ///< Startup measurements complete alarm
    BQ76905_ALARM_CDTOGGLE = 1, ///< Debounced CHG Detector signal alarm
    BQ76905_ALARM_POR = 0,      ///< POR in Battery Status asserted alarm
} bq76905_alarms_t;

/**
 * @brief Platform interface abstraction for BQ76905 driver.
 *
 * The user must provide these functions to enable hardware-specific I2C and timing.
 */
typedef struct {
    int (*i2c_write)(uint8_t dev_addr, const uint8_t *data, uint16_t len);
    int (*i2c_read)(uint8_t dev_addr, uint8_t *data, uint16_t len);
    void (*delay_ms)(uint32_t ms);
} bq76905_interface_t;

/**
 * @brief BQ76905 driver instance.
 */
typedef struct {
    uint8_t i2c_address;
    bq76905_interface_t io;
    bool initialized;

} bq76905_t;

/**
 * @brief BQ76905 alert signals
 */
typedef struct {
    bool cov;       ///< Cell Overvoltage Safety Alert
    bool cuv;       ///< Cell Undervoltage Safety Alert
    bool scd;       ///< Short Circuit in Discharge Safety Alert
    bool ocd1;      ///< Overcurrent in Discharge 1 Safety Alert
    bool ocd2;      ///< Overcurrent in Discharge 2 Safety Alert
    bool occ;       ///< Overcurrent in Charge Safety Alert
    bool otd;       ///< Overtemperature in Discharge Safety Alert
    bool otc;       ///< Overtemperature in Charge Safety Alert
    bool utd;       ///< Undertemperature in Discharge Safety Alert
    bool utc;       ///< Undertemperature in Charge Safety Alert
    bool otint;     ///< Internal Overtemperature Safety Alert
    bool hwd;       ///< Host Watchdog Safety Alert
    bool vref;      ///< VREF Measurement Diagnostic Alert
    bool vss;       ///< VSS Measurement Diagnostic Alert
} bq76905_safety_alerts_t;

/**
 * @brief BQ76905 fault signals
 */
typedef struct {
    bool cov;       ///< Cell Overvoltage Safety Fault
    bool cuv;       ///< Cell Undervoltage Safety Fault
    bool scd;       ///< Short Circuit in Discharge Safety Fault
    bool ocd1;      ///< Overcurrent in Discharge 1 Safety Fault
    bool ocd2;      ///< Overcurrent in Discharge 2 Safety Fault
    bool occ;       ///< Overcurrent in Charge Safety Fault
    bool curlatch;  ///< Current Protection Latch Safety Fault
    bool regout;    ///< REGOUT Safety Fault
    bool otd;       ///< Overtemperature in Discharge Safety Fault
    bool otc;       ///< Overtemperature in Charge Safety Fault
    bool utd;       ///< Undertemperature in Discharge Safety Fault
    bool utc;       ///< Undertemperature in Charge Safety Fault
    bool otint;     ///< Internal Overtemperature Safety Fault
    bool hwd;       ///< Host Watchdog Safety Fault
    bool vref;      ///< VREF Measurement Diagnostic Fault
    bool vss;       ///< VSS Measurement Diagnostic Fault
} bq76905_safety_faults_t;

/**
 * @brief BQ76905 battery status flags
 */
typedef struct {
    bool sleep;                     ///< SLEEP Mode
    bool deepsleep;                 ///< DEEPSLEEP Mode
    bool sa;                        ///< Safety Alert is Present
    bool ss;                        ///< Safety Fault is Present
    bq76905_security_state_t sec;   ///< Security State of Device
    bool fet_en;                    ///< FET Control Mode
    bool por;                       ///< Device Fully Reset
    bool sleep_en;                  ///< SLEEP Mode Enable
    bool cfg_update;                ///< CONFIG_UPDATE Mode
    bool alert_pin;                 ///< ALERT Pin Assertion (pulled low)
    bool chg;                       ///< CHG Driver Enabled
    bool dsg;                       ///< DSG Driver Enabled
    bool chg_det_flag;              ///< Debounced CHG Detector Signal State
} bq76905_battery_status_t;

/**
 * @brief BQ76905 alarm status: latched signal used to assert the ALERT pin. Write a bit high to clear the latched bit.
 */
typedef struct {
    bool ssa;           ///< Latched when a bit in Safety Status A() is set
    bool ssb;           ///< Latched when a bit in Safety Status B() is set
    bool saa;           ///< Latched when a bit in Safety Alert A() is set
    bool sab;           ///< Latched when a bit in Safety Alert B() is set
    bool xchg;          ///< Latched when the CHG driver is disabled
    bool xdsg;          ///< Latched when the DSG driver is disabled
    bool shutv;         ///< Latched when either a cell voltage < Shutdown Cell Voltage or stack voltage < Shutdown Stack Voltage
    bool cb;            ///< Latched when cell balancing is active
    bool fullscan;      ///< Latched when a full scan is complete
    bool adscan;        ///< Latched when a voltage ADC measurement scan is complete
    bool wake;          ///< Latched when the device is wakened from SLEEP mode
    bool sleep;         ///< Latched when the device enters SLEEP mode
    bool timer_alarm;   ///< Latched when the programmable timer expires
    bool init_comp;     ///< Latched when the device completes the startup measurement sequence
    bool cd_toggle;     ///< Latched when the debounced CHG Detector signal is different from the last debounced value
    bool por;           ///< Latched when the POR bit in Battery Status is asserted
} bq76905_alarm_status_t;

/**
 * @brief BQ76905 raw alarm status: unlatched flags which can be selected to be latched using Alarm Enable()
 */
typedef struct {
    bool ssa;           ///< Latched when a bit in Safety Status A() is set
    bool ssb;           ///< Latched when a bit in Safety Status B() is set
    bool saa;           ///< Latched when a bit in Safety Alert A() is set
    bool sab;           ///< Latched when a bit in Safety Alert B() is set
    bool xchg;          ///< Latched when the CHG driver is disabled
    bool xdsg;          ///< Latched when the DSG driver is disabled
    bool shutv;         ///< Latched when either a cell voltage < Shutdown Cell Voltage or stack voltage < Shutdown Stack Voltage
    bool cb;            ///< Latched when cell balancing is active
    bool fullscan;      ///< Latched when a full scan is complete
    bool adscan;        ///< Latched when a voltage ADC measurement scan is complete
    bool wake;          ///< Latched when the device is wakened from SLEEP mode
    bool sleep;         ///< Latched when the device enters SLEEP mode
    bool timer_alarm;   ///< Latched when the programmable timer expires
    bool init_comp;     ///< Latched when the device completes the startup measurement sequence
    bool cd_toggle;     ///< Latched when the debounced CHG Detector signal is different from the last debounced value
    bool por;           ///< Latched when the POR bit in Battery Status is asserted
} bq76905_raw_alarm_status_t;

/**
 * @brief BQ76905 Mask for Alarm Status(). Can be written to change which alarm sources are enabled
 */
typedef struct {
    bool ssa;           ///< Safety Status A()
    bool ssb;           ///< Safety Status B()
    bool saa;           ///< Safety Alert A()
    bool sab;           ///< Safety Alert B()
    bool xchg;          ///< CHG driver is disabled
    bool xdsg;          ///< DSG driver is disabled
    bool shutv;         ///< Either a cell voltage < Shutdown Cell Voltage or stack voltage < Shutdown Stack Voltage
    bool cb;            ///< Cell balancing is active
    bool fullscan;      ///< Full scan is complete
    bool adscan;        ///< Voltage ADC measurement scan is complete
    bool wake;          ///< Device is wakened from SLEEP mode
    bool sleep;         ///< Device enters SLEEP mode
    bool timer_alarm;   ///< Programmable timer expires
    bool init_comp;     ///< Device completes the startup measurement sequence
    bool cd_toggle;     ///< Debounced CHG Detector signal is different from the last debounced value
    bool por;           ///< POR bit in Battery Status is asserted
} bq76905_alarm_enable_t;

/**
 * @brief BQ76905 FET control.
 *
 * - 0 = FET driver is allowed to turn on if other conditions are met
 * - 1 = FET driver is forced off
 */
typedef struct {
    bool chg_off;       ///< CHG FET driver control
    bool dsg_off;       ///< DSG FET driver control
    bool chg_on;        ///< CHG FET driver control
    bool dsg_on;        ///< DSG FET driver control
} bq76905_fet_ctrl_t;

/**
 * @breif BQ76905 REGOUT control: changes voltage regulator settings
 */
typedef struct {
    bool ts_on;                             ///< Control for TS pullup to stay biased continuously
    bool reg_en;                            ///< REGOUT LDO enable
    bq76905_regout_voltage_ctrl regoutv;    ///< REGOUT LDO voltage control
} bq76905_regout_ctrl_t;

/**
 * @brief BQ76905 DSG FET driver PWM control
 */
typedef struct {
    bool dsg_pwm_en;    ///< DSG FET driver PWM mode control
    float pwm_on;       ///< Time the DSG FET driver is enabled when PWM mode is enabled
    float pwm_off;      ///< Time the DSG FET driver is disabled each cycle when PWM mode is enabled
} bq76905_dsg_fet_pwm_ctrl_t;

/**
 * @brief BQ76905 CHG FET driver PWM control
 */
typedef struct {
    bool chg_pwm_en;    ///< CHG FET driver PWM mode control
    float pwm_on;       ///< Time the CHG FET driver is enabled when PWM mode is enabled
    float pwm_off;      ///< Time the CHG FET driver is disabled each cycle when PWM mode is enabled
} bq76905_chg_fet_pwm_ctrl_t;

/**
 * @brief BQ76905 measurements
 */
typedef struct {
    int16_t cell_1_voltage;     ///< 16-bit voltage on cell 1 (mV)
    int16_t cell_2_voltage;     ///< 16-bit voltage on cell 2 (mV)
    int16_t cell_3_voltage;     ///< 16-bit voltage on cell 3 (mV)
    int16_t cell_4_voltage;     ///< 16-bit voltage on cell 4 (mV)
    int16_t cell_5_voltage;     ///< 16-bit voltage on cell 5 (mV)
    int16_t reg18_voltage;      ///< Internal 1.8V regulator voltage measured using bandgap reference, used for diagnostic of VREF1 vs VREF2 (mV)
    int16_t vss_voltage;        ///< Measurement of VSS using ADC, used for diagnostic of ADC input mux (mV)
    uint16_t stack_voltage;     ///< 16-bit voltage on top of stack (mV)
    float internal_temp;        ///< Most recent measured internal die temperature (°C)
    int16_t ts_temp;            ///< ADC measurement of the TS pin
    int32_t raw_current;        ///< 32-bit raw current measurement (mA)
    int16_t current;            ///< 16-bit CC2 current measurement (mA)
    int16_t cc1_current;        ///< 16-bit CC1 current measurement (mA)
} bq76905_measurements_t;

/**
 * @brief BQ76905 settings
 */
typedef struct {
    bool fet_en;                ///< Toggle FET_EN bit in Battery Status(). FET_EN=0 means manual FET control. FET_EN=1 means autonomous device FET control
    uint16_t device_number;     ///< Device number
    uint16_t firmware_version;  ///< Device firmware major and minor version number
    uint16_t build_number;      ///< Firmware build number
    uint16_t hardware_version;  ///< Reports the device hardware version number
    int64_t passq;              ///< Accumulated charge
    uint32_t passtime;          ///< Accumulated time
    uint16_t security_keys;     ///< Security key that must be sent to transition from SEALED to FULLACCESS mode. Array of two 16-bit words
    uint16_t prog_timer;        ///< Programmable timer which allows the REGOUT LDO to be disabled and wakened after a programmed time or by alarm
    uint8_t prot_recovery;      ///< Enabled the host to allow recovery of selected protection faults
} bq76905_settings_t;

/**
 * @brief BQ76905 active cells
 */
typedef struct {
    bool cell_1;    ///< First active cell (connected between VC1 and VC0)
    bool cell_2;    ///< Second active cell
    bool cell_3;    ///< Third active cell
    bool cell_4;    ///< Fourth active cell
    bool cell_5;    ///< Fifth active cell
} bq76905_active_cells_t;

/**
 * @brief BQ76905 timer settings
 */
typedef struct {
    bool regout_alarm_wake;                 ///< Control to determine if REGOUT is wakened when an Alarm Status() bit asserts
    bq76905_regout_disable_delay_t delay;   ///< REGOUT disable delay
    bool regout_disabled;                   ///< Is REGOUT disabled
    uint16_t prog_timer;                    ///< Timer value programmable from 250ms to 243s
} bq76905_prog_timer_t;

typedef struct {
    uint16_t cell_1_gain;       ///< Cell 1 Gain
    uint16_t stack_gain;        ///< Battery Stack Gain
    int8_t cell_2_gain;         ///< Cell 2 Gain Delta
    int8_t cell_3_gain;         ///< Cell 3 Gain Delta
    int8_t cell_4_gain;         ///< Cell 4 Gain Delta
    int8_t cell_5_gain;         ///< Cell 5 Gain Delta
    uint16_t current_gain;      ///< Current Gain
    uint16_t current_offset;    ///< Current Offset
    int16_t ts_offset;          ///< TS Offset
    uint16_t int_temp_gain;     ///< Internal Temperature Gain
    int16_t int_temp_offset;    ///< Internal Temperature Offset
} bq76905_calibration_data_t;

/**
 * @brief Human-readable description of an BQ76905 status code.
 *
 * @param status The status code to translate.
 * @return A string describing the error.
 */
const char* bq76905_stat_error(bq76905_status_t status);

bq76905_status_t bq76905_init(bq76905_t *dev, uint8_t address, bq76905_interface_t io);

bq76905_status_t bq76905_get_safety_alert_A(bq76905_t *dev, bq76905_safety_alerts_t *safety_alerts);

bq76905_status_t bq76905_get_safety_fault_A(bq76905_t *dev, bq76905_safety_faults_t *safety_faults);

bq76905_status_t bq76905_get_safety_alert_B(bq76905_t *dev, bq76905_safety_alerts_t *safety_alerts);

bq76905_status_t bq76905_get_safety_fault_B(bq76905_t *dev, bq76905_safety_faults_t *safety_faults);

bq76905_status_t bq76905_get_battery_status(bq76905_t *dev, bq76905_battery_status_t *battery_status);

bq76905_status_t bq76905_get_cell_1_voltage(bq76905_t *dev, int16_t *cell_1_voltage);

bq76905_status_t bq76905_get_cell_2_voltage(bq76905_t *dev, int16_t *cell_2_voltage);

bq76905_status_t bq76905_get_cell_3_voltage(bq76905_t *dev, int16_t *cell_3_voltage);

bq76905_status_t bq76905_get_cell_4_voltage(bq76905_t *dev, int16_t *cell_4_voltage);

bq76905_status_t bq76905_get_cell_5_voltage(bq76905_t *dev, int16_t *cell_5_voltage);

bq76905_status_t bq76905_get_reg18_voltage(bq76905_t *dev, int16_t *reg18_voltage);

bq76905_status_t bq76905_get_vss_voltage(bq76905_t *dev, int16_t *vss_voltage);

bq76905_status_t bq76905_get_stack_voltage(bq76905_t *dev, uint16_t *stack_voltage);

bq76905_status_t bq76905_get_ic_temperature(bq76905_t *dev, float *internal_temp);

bq76905_status_t bq76905_get_ts_temperature(bq76905_t *dev, int16_t *ts_temp);

bq76905_status_t bq76905_get_raw_current(bq76905_t *dev, int32_t *raw_current);

bq76905_status_t bq76905_get_current(bq76905_t *dev, int16_t *current);

bq76905_status_t bq76905_get_cc1_current(bq76905_t *dev, int16_t *cc1_current);

bq76905_status_t bq76905_get_latched_alarms(bq76905_t *dev, bq76905_alarm_status_t *alarm_status);

bq76905_status_t bq76905_clear_alarm(bq76905_t *dev, bq76905_alarms_t alarm);

bq76905_status_t bq76905_check_alarms(bq76905_t *dev, bq76905_alarm_status_t *alarm_status);

bq76905_status_t bq76905_get_enabled_alarms(bq76905_t *dev, bq76905_alarm_enable_t *alarm);
bq76905_status_t bq76905_enable_alarm(bq76905_t *dev, bq76905_alarms_t alarm);
bq76905_status_t bq76905_disable_alarm(bq76905_t *dev, bq76905_alarms_t alarm);

//bq76905_status_t bq76905_get_chg_off(bq76905_t *dev, bool *chg_off);
//bq76905_status_t bq76905_set_chg_off(bq76905_t *dev, bool chg_off);

//bq76905_status_t bq76905_get_dsg_off(bq76905_t *dev, bool *dsg_off);
//bq76905_status_t bq76905_set_dsg_off(bq76905_t *dev, bool dsg_off);

//bq76905_status_t bq76905_get_chg_on(bq76905_t *dev, bool *chg_on);
//bq76905_status_t bq76905_set_chg_on(bq76905_t *dev, bool chg_on);

//bq76905_status_t bq76905_get_dsg_on(bq76905_t *dev, bool *dsg_on);
//bq76905_status_t bq76905_set_dsg_on(bq76905_t *dev, bool dsg_on);

bq76905_status_t bq76905_get_ts_on(bq76905_t *dev, bool *ts_on);
bq76905_status_t bq76905_set_ts_on(bq76905_t *dev, bool ts_on);

bq76905_status_t bq76905_get_reg_en(bq76905_t *dev, bool *reg_en);
bq76905_status_t bq76905_set_reg_en(bq76905_t *dev, bool reg_en);

bq76905_status_t bq76905_get_regoutv(bq76905_t *dev, bq76905_regout_voltage_ctrl *regoutv);
bq76905_status_t bq76905_set_regoutv(bq76905_t *dev, bq76905_regout_voltage_ctrl regoutv);

bq76905_status_t bq76905_is_dsg_pwm_enabled(bq76905_t *dev, bool *dsg_pwm_en);
bq76905_status_t bq76905_enable_dsg_pwm(bq76905_t *dev, bool dsg_pwm_en);

bq76905_status_t bq76905_get_dsg_pwm_on(bq76905_t *dev, float *dsg_pwm_on);
bq76905_status_t bq76905_set_dsg_pwm_on(bq76905_t *dev, float dsg_pwm_on);
bq76905_status_t bq76905_disable_dsg_pwm_on(bq76905_t *dev);

bq76905_status_t bq76905_get_dsg_pwm_off(bq76905_t *dev, float *dsg_pwm_off);
bq76905_status_t bq76905_set_dsg_pwm_off(bq76905_t *dev, float dsg_pwm_off);
bq76905_status_t bq76905_disable_dsg_pwm_off(bq76905_t *dev);

bq76905_status_t bq76905_is_chg_pwm_enabled(bq76905_t *dev, bool *chg_pwm_en);
bq76905_status_t bq76905_enable_chg_pwm(bq76905_t *dev, bool chg_pwm_en);

bq76905_status_t bq76905_get_chg_pwm_on(bq76905_t *dev, float *chg_pwm_on);
bq76905_status_t bq76905_set_chg_pwm_on(bq76905_t *dev, float chg_pwm_on);
bq76905_status_t bq76905_disable_chg_pwm_on(bq76905_t *dev);

bq76905_status_t bq76905_get_chg_pwm_off(bq76905_t *dev, float *chg_pwm_off);
bq76905_status_t bq76905_set_chg_pwm_off(bq76905_t *dev, float chg_pwm_off);
bq76905_status_t bq76905_disable_chg_pwm_off(bq76905_t *dev);

///< @section Command-Only Subcommands

bq76905_status_t bq76905_reset_passq(bq76905_t *dev);

bq76905_status_t bq76905_exit_deepsleep(bq76905_t *dev);

bq76905_status_t bq76905_enter_deepsleep(bq76905_t *dev);   ///< Must be sent twice in a row within 4s to take effect

bq76905_status_t bq76905_shutdown(bq76905_t *dev);      ///< Must be sent twice in a row within 4s to take effect

bq76905_status_t bq76905_reset(bq76905_t *dev);

bq76905_status_t bq76905_fet_enable(bq76905_t *dev, bq76905_settings_t *settings, bool fet_en);

//bq76905_status_t bq76905_seal(bq76905_t *dev);

bq76905_status_t bq76905_enter_cfg_update(bq76905_t *dev);

bq76905_status_t bq76905_exit_cfg_update(bq76905_t *dev);

bq76905_status_t bq76905_sleep_enable(bq76905_t *dev);

bq76905_status_t bq76905_sleep_disable(bq76905_t *dev);

///< @section Subcommands with Data

bq76905_status_t bq76905_get_device_number(bq76905_t *dev, bq76905_settings_t *settings);

bq76905_status_t bq76905_get_firmware_version(bq76905_t *dev, bq76905_settings_t *settings);

bq76905_status_t bq76905_get_hardware_version(bq76905_t *dev, uint16_t *hardware_version);

bq76905_status_t bq76905_get_passq(bq76905_t *dev, bq76905_settings_t *settings);

bq76905_status_t bq76905_get_passtime(bq76905_t *dev, bq76905_settings_t *settings);

bq76905_status_t bq76905_use_security_keys(bq76905_t *dev, uint16_t *security_keys);

bq76905_status_t bq7605_get_active_cells(bq76905_t *dev, bq76905_active_cells_t *active_cells);
bq76905_status_t bq76905_set_active_cells(bq76905_t *dev, bq76905_active_cells_t active_cells);

bq76905_status_t bq76905_get_measurements(bq76905_t *dev, bq76905_measurements_t *measurements);

#ifdef __cplusplus
// }
#endif

#endif //BQ76905_H