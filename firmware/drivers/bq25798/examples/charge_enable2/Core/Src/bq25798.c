#include "bq25798.h"

static uint8_t bq25798_i2c_addr = 0x00;

// Helper Functions
static BQ25798_Status read_register(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    if (bq25798_i2c_read(bq25798_i2c_addr, reg, data, 2) != BQ25798_OK)
        return BQ25798_ERROR;

    *value = (data[1] << 8) | data[0];
    return BQ25798_OK;
}

static BQ25798_Status write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[2] = {value & 0xFF, (value >> 8) & 0xFF};
    return bq25798_i2c_write(bq25798_i2c_addr, reg, data, 2);
}

// Initialization
BQ25798_Status bq25798_init(uint8_t i2c_addr)
{
    bq25798_i2c_addr = i2c_addr;

    uint8_t id = 0;
    if (bq25798_i2c_read(bq25798_i2c_addr, BQ25798_REG_PART_INFO, &id, 1) != BQ25798_OK)
        return BQ25798_ERROR;
}

// Configuration
BQ25798_Status bq25798_set_min_system_voltage(uint8_t millivolts)
{
    uint8_t value = (millivolts - 2500) / 250;
    return bq25798_i2c_write(bq25798_i2c_addr, BQ25798_REG_MIN_SYS_VOLT, (uint8_t)value, 1);
}

BQ25798_Status bq25798_set_charge_voltage_limit(uint16_t millivolts)
{
    uint16_t value = millivolts / 10;
    return write_register(BQ25798_REG_CHG_VOLT_LIM, value);
}

BQ25798_Status bq25798_set_charge_current_limit(uint16_t milliamps)
{
    uint16_t value = milliamps / 10;
    return write_register(BQ25798_REG_CHG_CURR_LIM, value);
}

BQ25798_Status bq25798_set_input_voltage_limit(uint8_t millivolts)
{
    uint8_t value = millivolts / 100;
    return bq25798_i2c_write(bq25798_i2c_addr, BQ25798_REG_INPT_VOLT_LIM, value, 1);
}

BQ25798_Status bq25798_set_input_current_limit(uint16_t milliamps)
{
    uint16_t value = milliamps / 10;
    return write_register(BQ25798_REG_INPT_CURR_LIM, value);
}

// Control
BQ25798_Status bq25798_mppt_control(BQ25798_MPPT_Control_t control)
{
    uint8_t value = 0;
    value |= (control.en_mppt   & 0x01) << 0;
    value |= (control.voc_rate  & 0x03) << 1;
    value |= (control.voc_dly   & 0x03) << 3;
    value |= (control.voc_pct   & 0x07) << 5;

    return bq25798_i2c_write(bq25798_i2c_addr, BQ25798_REG_MPPT_CTRL, value, 1);
}

BQ25798_Status bq25798_temperature_control(BQ25798_Temperature_Control_t control)
{
    uint8_t value = 0;
    value |= (control.bkup_acfet1_on    & 0x01) << 0;
    value |= (control.vac2_pd_en        & 0x01) << 1;
    value |= (control.vac1_pd_en        & 0x01) << 2;
    value |= (control.vbus_pd_en        & 0x01) << 3;
    value |= (control.tshut             & 0x03) << 4;
    value |= (control.treg              & 0x03) << 6;

    return bq25798_i2c_write(bq25798_i2c_addr, BQ25798_REG_TEMP_CTRL, value, 1);
}

BQ25798_Status bq25798_ntc_jeita_control(BQ25798_NTC_JEITA_Control_t control)
{
    uint8_t value = 0;
    value |= (control.jeita_isetc   & 0x03) << 1;
    value |= (control.jeita_iseth   & 0x03) << 3;
    value |= (control.jeita_vset    & 0x07) << 5;

    return bq25798_i2c_write(bq25798_i2c_addr, BQ25798_REG_NTC_CTRL0, value, 1);
}

BQ25798_Status bq25798_ntc_temp_control(BQ25798_NTC_Temp_Control_t control)
{
    uint8_t value = 0;
    value |= (control.ts_ignore & 0x01) << 0;
    value |= (control.bcold     & 0x01) << 1;
    value |= (control.bhot      & 0x03) << 2;
    value |= (control.ts_warm   & 0x03) << 4;
    value |= (control.ts_cool   & 0x03) << 6;

    return bq25798_i2c_write(bq25798_i2c_addr, BQ25798_REG_NTC_CTRL1, value, 1);
}

BQ25798_Status bq25798_adc_control(BQ25798_ADC_Control_t control)
{
    uint8_t value = 0;
    value |= (control.adc_avg_init  & 0x01) << 2;
    value |= (control.adc_avg       & 0x01) << 3;
    value |= (control.adc_sample    & 0x03) << 4;
    value |= (control.adc_rate      & 0x01) << 6;
    value |= (control.adc_en        & 0x01) << 7;

    return bq25798_i2c_write(bq25798_i2c_addr, BQ25798_REG_ADC_CTRL, value, 1);
}

BQ25798_Status bq25798_adc_control(BQ25798_ADC_Funct_Disable_t control)
{
    uint8_t value0 = 0;
    value0 |= (control.tdie_adc_dis ? 1 : 0) << 1;
    value0 |= (control.ts_adc_dis   ? 1 : 0) << 2;
    value0 |= (control.vsys_adc_dis ? 1 : 0) << 3;
    value0 |= (control.vbat_adc_dis ? 1 : 0) << 4;
    value0 |= (control.vbus_adc_dis ? 1 : 0) << 5;
    value0 |= (control.ibat_adc_dis ? 1 : 0) << 6;
    value0 |= (control.ibus_adc_dis ? 1 : 0) << 7;

    if (bq25798_i2c_write(bq25798_i2c_addr, BQ25798_REG_ADC_FUNC_DIS0, value0, 1) != BQ25798_OK)
        return BQ25798_ERROR;
    
    uint8_t value1 = 0;
    value1 |= (control.vac1_adc_dis ? 1 : 0) << 4;
    value1 |= (control.vac2_adc_dis ? 1 : 0) << 5;
    value1 |= (control.dm_adc_dis   ? 1 : 0) << 6;
    value1 |= (control.dp_adc_dis   ? 1 : 0) << 7;

    if (bq25798_i2c_write(bq25798_i2c_addr, BQ25798_REG_ADC_FUNC_DIS1, value1, 1) != BQ25798_OK)
        return BQ25798_ERROR;

    return BQ25798_OK;
}

BQ25798_Status bq25798_read_charger_status(BQ25798_Charger_Status_t *status)
{
    uint8_t values = 0;
    if (bq25798_i2c_read(bq25798_i2c_addr, BQ25798_REG_CHG_STAT0, values, 1) != BQ25798_OK)
        return BQ25798_ERROR;
    
    BQ25798_Charger_Status_t stats = *status;

    stats.vbus_present_stat = (values >> 0) & 0x01;
    stats.ac1_present_stat  = (values >> 1) & 0x01;
    stats.ac2_present_stat  = (values >> 2) & 0x01;
    stats.pg_stat           = (values >> 3) & 0x01;
    stats.wd_stat           = (values >> 5) & 0x01;
    stats.vimpm_stat        = (values >> 6) & 0x01;
    stats.iindpm_stat       = (values >> 7) & 0x01;

    *status = stats;

    return BQ25798_OK;
}

BQ25798_Status bq25798_read_fault_status(BQ25798_Fault_Status_t *fault)
{
    uint8_t values0 = 0;
    if (bq25798_i2c_read(bq25798_i2c_addr, BQ25798_REG_FAULT_STAT0, values0, 1) != BQ25798_OK)
        return BQ25798_ERROR;
    
    BQ25798_Fault_Status_t faults = *fault;

    faults.vac1_ovp_stat    = (values0 >> 0) & 0x01;
    faults.vac2_ovp_stat    = (values0 >> 1) & 0x01;
    faults.conv_ocp_stat    = (values0 >> 2) & 0x01;
    faults.ibat_ocp_stat    = (values0 >> 3) & 0x01;
    faults.ibus_ocp_stat    = (values0 >> 4) & 0x01;
    faults.vbat_ovp_stat    = (values0 >> 5) & 0x01;
    faults.vbus_ovp_stat    = (values0 >> 6) & 0x01;
    faults.ibat_reg_stat    = (values0 >> 7) & 0x01;

    uint8_t values1 = 0;
    if (bq25798_i2c_read(bq25798_i2c_addr, BQ25798_REG_FAULT_STAT1, values1, 1) != BQ25798_OK)
        return BQ25798_ERROR;
    
    faults.tshut_stat       = (values1 >> 2) & 0x01;
    faults.otg_uvp_stat     = (values1 >> 4) & 0x01;
    faults.otg_ovp_stat     = (values1 >> 5) & 0x01;
    faults.vsys_ovp_stat    = (values1 >> 6) & 0x01;
    faults.vsys_short_stat  = (values1 >> 7) & 0x01;

    *fault = faults;

    return BQ25798_OK;
}

BQ25798_Status bq25798_read_ibus_adc(int16_t *milliamps)
{
    uint16_t val;
    if (read_register(BQ25798_REG_IBUS_ADC, &val) != BQ25798_OK)
        return BQ25798_ERROR;
    
    *milliamps = (int16_t)val;

    return BQ25798_OK;
}

BQ25798_Status bq25798_read_ibat_adc(int16_t *milliamps)
{
    uint16_t val;
    if (read_register(BQ25798_REG_IBUS_ADC, &val) != BQ25798_OK)
        return BQ25798_ERROR;
    
    *milliamps = (int16_t)val;

    return BQ25798_OK;
}

BQ25798_Status bq25798_read_vbus_adc(uint16_t *millivolts)
{
    return read_register(BQ25798_REG_VBUS_ADC, millivolts);
}

BQ25798_Status bq25798_read_vbat_adc(uint16_t *millivolts)
{
    return read_register(BQ25798_REG_VBAT_ADC, millivolts);
}

BQ25798_Status bq25798_read_vsys_adc(uint16_t *millivolts)
{
    return read_register(BQ25798_REG_VSYS_ADC, millivolts);
}

BQ25798_Status bq25798_read_tdie_adc(int16_t *celsius)
{
    uint16_t val;
    if (read_register(BQ25798_REG_TDIE_ADC, &val) != BQ25798_OK)
        return BQ25798_ERROR;

    *celsius = (int16_t)val;

    return BQ25798_OK;
}
