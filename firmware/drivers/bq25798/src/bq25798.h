#ifndef BQ25798_H
#define BQ25798_H

#include <stdint.h>
#include <stdbool.h>
#include "bq25798_platform.h"

#define BQ25798_I2C_ADDR1   0x6A
#define BQ25798_I2C_ADDR2   0x6B


// Register Addresses
#define BQ25798_REG_MIN_SYS_VOLT    0x00
#define BQ25798_REG_CHG_VOLT_LIM    0x01
#define BQ25798_REG_CHG_CURR_LIM    0x03
#define BQ25798_REG_INPT_VOLT_LIM   0x05
#define BQ25798_REG_INPT_CURR_LIM   0x06
#define BQ25798_REG_PRECHG_CTRL     0x08
#define BQ25798_REG_TERM_CTRL       0x09
#define BQ25798_REG_RECHG_CTRL      0x0A
#define BQ25798_REG_VOLTG_REG       0x0B
#define BQ25798_REG_IOTG_REG        0x0D
#define BQ25798_REG_TIMER_CTRL      0x0E
#define BQ25798_REG_CHG_CTRL0       0x0F
#define BQ25798_REG_CHG_CTRL1       0x10
#define BQ25798_REG_CHG_CTRL2       0x11
#define BQ25798_REG_CHG_CTRL3       0x12
#define BQ25798_REG_CHG_CTRL4       0x13
#define BQ25798_REG_CHG_CTRL5       0x14
#define BQ25798_REG_MPPT_CTRL       0x15
#define BQ25798_REG_TEMP_CTRL       0x16
#define BQ25798_REG_NTC_CTRL0       0x17
#define BQ25798_REG_NTC_CTRL1       0x18
#define BQ25798_REG_ICO_CURR_LIM    0x19
#define BQ25798_REG_CHG_STAT0       0x1B
#define BQ25798_REG_CHG_STAT1       0x1C
#define BQ25798_REG_CHG_STAT2       0x1D
#define BQ25798_REG_CHG_STAT3       0x1E
#define BQ25798_REG_CHG_STAT4       0x1F
#define BQ25798_REG_FAULT_STAT0     0x20
#define BQ25798_REG_FAULT_STAT1     0x21
#define BQ25798_REG_CHG_FLAG0       0x22
#define BQ25798_REG_CHG_FLAG1       0x23
#define BQ25798_REG_CHG_FLAG2       0x24
#define BQ25798_REG_CHG_FLAG3       0x25
#define BQ25798_REG_FAULT_FLAG0     0x26
#define BQ25798_REG_FAULT_FLAG1     0x27
#define BQ25798_REG_CHG_MASK0       0x28
#define BQ25798_REG_CHG_MASK1       0x29
#define BQ25798_REG_CHG_MASK2       0x2A
#define BQ25798_REG_CHG_MASK3       0x2B
#define BQ25798_REG_FAULT_MASK0     0x2C
#define BQ25798_REG_FAULT_MASK1     0x2D
#define BQ25798_REG_ADC_CTRL        0x2E
#define BQ25798_REG_ADC_FUNC_DIS0   0x2F
#define BQ25798_REG_ADC_FUNC_DIS1   0x30
#define BQ25798_REG_IBUS_ADC        0x31
#define BQ25798_REG_IBAT_ADC        0x33
#define BQ25798_REG_VBUS_ADC        0x35
#define BQ25798_REG_VAC1_ADC        0x37
#define BQ25798_REG_VAC2_ADC        0x39
#define BQ25798_REG_VBAT_ADC        0x3B
#define BQ25798_REG_VSYS_ADC        0x3D
#define BQ25798_REG_TS_ADC          0x3F
#define BQ25798_REG_TDIE_ADC        0x41
#define BQ25798_REG_DP_ADC          0x43
#define BQ25798_REG_DM_ADC          0x45
#define BQ25798_REG_DPDM_DRIVER     0x47
#define BQ25798_REG_PART_INFO       0x48

typedef struct {
    uint8_t en_mppt     : 1;
    uint8_t voc_rate    : 2;
    uint8_t voc_dly     : 2;
    uint8_t voc_pct     : 3;
} BQ25798_MPPT_Control_t;

typedef struct {
    uint8_t bkup_acfet1_on  : 1;
    uint8_t vac2_pd_en      : 1;
    uint8_t vac1_pd_en      : 1;
    uint8_t vbus_pd_en      : 1;
    uint8_t tshut           : 2;
    uint8_t treg            : 2;
} BQ25798_Temperature_Control_t;

typedef struct {
    uint8_t jeita_isetc : 2;
    uint8_t jeita_iseth : 2;
    uint8_t jeita_vset  : 3;
} BQ25798_NTC_JEITA_Control_t;

typedef struct {
    uint8_t ts_ignore   : 1;
    uint8_t bcold       : 1;
    uint8_t bhot        : 2;
    uint8_t ts_warm     : 2;
    uint8_t ts_cool     : 2;
} BQ25798_NTC_Temp_Control_t;

typedef struct {
    uint8_t adc_avg_init    : 1;
    uint8_t adc_avg         : 1;
    uint8_t adc_sample      : 2;
    uint8_t adc_rate        : 1;
    uint8_t adc_en          : 1;
} BQ25798_ADC_Control_t;

typedef struct {
    bool tdie_adc_dis   : 1;
    bool ts_adc_dis     : 1;
    bool vsys_adc_dis   : 1;
    bool vbat_adc_dis   : 1;
    bool vbus_adc_dis   : 1;
    bool ibat_adc_dis   : 1;
    bool ibus_adc_dis   : 1;
    bool vac1_adc_dis   : 1;
    bool vac2_adc_dis   : 1;
    bool dm_adc_dis     : 1;
    bool dp_adc_dis     : 1;
} BQ25798_ADC_Funct_Disable_t;

typedef struct {
    bool vbus_present_stat  : 1;
    bool ac1_present_stat   : 1;
    bool ac2_present_stat   : 1;
    bool pg_stat            : 1;
    bool wd_stat            : 1;
    bool vimpm_stat         : 1;
    bool iindpm_stat        : 1;
} BQ25798_Charger_Status_t;

typedef struct {
    bool vac1_ovp_stat      : 1;
    bool vac2_ovp_stat      : 1;
    bool conv_ocp_stat      : 1;
    bool ibat_ocp_stat      : 1;
    bool ibus_ocp_stat      : 1;
    bool vbat_ovp_stat      : 1;
    bool vbus_ovp_stat      : 1;
    bool ibat_reg_stat      : 1;
    bool tshut_stat         : 1;
    bool otg_uvp_stat       : 1;
    bool otg_ovp_stat       : 1;
    bool vsys_ovp_stat      : 1;
    bool vsys_short_stat    : 1;
} BQ25798_Fault_Status_t;

// Initialization
BQ25798_Status bq25798_init(uint8_t i2c_addr);

// Configuration Functions
BQ25798_Status bq25798_set_min_system_voltage(uint8_t millivolts);
BQ25798_Status bq25798_set_charge_voltage_limit(uint16_t millivolts);
BQ25798_Status bq25798_set_charge_current_limit(uint16_t milliamps);
BQ25798_Status bq25798_set_input_voltage_limit(uint8_t millivolts);
BQ25798_Status bq25798_set_input_current_limit(uint16_t milliamps);

// Control Functions
BQ25798_Status bq25798_mppt_control(BQ25798_MPPT_Control_t control);
BQ25798_Status bq25798_temperature_control(BQ25798_Temperature_Control_t control);
BQ25798_Status bq25798_ntc_jeita_control(BQ25798_NTC_JEITA_Control_t control);
BQ25798_Status bq25798_ntc_temp_control(BQ25798_NTC_Temp_Control_t control);
BQ25798_Status bq25798_adc_control(BQ25798_ADC_Control_t control);
BQ25798_Status bq25798_adc_function_disable(BQ25798_ADC_Funct_Disable_t control);

// Status Reading Functions
BQ25798_Status bq25798_read_charger_status(BQ25798_Charger_Status_t *status);
BQ25798_Status bq25798_read_fault_status(BQ25798_Fault_Status_t *fault);

// ADC Reading Functions
BQ25798_Status bq25798_read_ibus_adc(int16_t *milliamps);
BQ25798_Status bq25798_read_ibat_adc(int16_t *milliamps);
BQ25798_Status bq25798_read_vbus_adc(uint16_t *millivolts);
BQ25798_Status bq25798_read_vbat_adc(uint16_t *millivolts);
BQ25798_Status bq25798_read_vsys_adc(uint16_t *millivolts);
BQ25798_Status bq25798_read_tdie_adc(int16_t *celsius);

#endif // BQ25798_H