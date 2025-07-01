#include "bq76905.h"

static uint8_t bq76905_i2c_addr = 0x00;

// Helper Functions
static BQ76905_Status read_register(uint8_t reg, uint8_t *data, uint16_t len)
{
    return bq76905_i2c_read(bq76905_i2c_addr, reg, data, len);
}

static BQ76905_Status write_register(uint8_t reg, const uint8_t *data, uint16_t len)
{
    return bq76905_i2c_write(bq76905_i2c_addr, reg, data, len);
}

// Initialization
BQ76905_Status bq76905_init(uint8_t i2c_addr)
{
    bq76905_i2c_addr = i2c_addr;
    
    // Read battery status to verify communication
    BQ76905_Battery_Status_t status;
    return bq76905_read_battery_status(&status);
}

// Safety Status Functions
BQ76905_Status bq76905_read_safety_alert_a(BQ76905_Safety_Alert_A_t *alert)
{
    uint8_t value;
    if (read_register(BQ76905_REG_SAFETY_ALERT_A, &value, 1) != BQ76905_OK)
        return BQ76905_ERROR;
    
    *((uint8_t*)alert) = value;
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_safety_status_a(BQ76905_Safety_Status_A_t *status)
{
    uint8_t value;
    if (read_register(BQ76905_REG_SAFETY_STATUS_A, &value, 1) != BQ76905_OK)
        return BQ76905_ERROR;

    *((uint8_t*)status) = value;
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_safety_alert_b(BQ76905_Safety_Alert_B_t *alert)
{
    uint8_t value;
    if (read_register(BQ76905_REG_SAFETY_ALERT_B, &value, 1) != BQ76905_OK)
        return BQ76905_ERROR;
        
    *((uint8_t*)alert) = value;
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_safety_status_b(BQ76905_Safety_Status_B_t *status)
{
    uint8_t value;
    if (read_register(BQ76905_REG_SAFETY_STATUS_B, &value, 1) != BQ76905_OK)
        return BQ76905_ERROR;
    
    *((uint8_t*)status) = value;
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_battery_status(BQ76905_Battery_Status_t *status)
{
    uint8_t values[2];
    if (read_register(BQ76905_REG_BATTERY_STATUS, values, 2) != BQ76905_OK)
        return BQ76905_ERROR;
    
    *((uint16_t*)status) = (values[1] << 8) | values[0];
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_alarm_status(BQ76905_Alarm_Status_t *status)
{
    uint8_t values[2];
    if (read_register(BQ76905_REG_ALARM_STATUS, values, 2) != BQ76905_OK)
        return BQ76905_ERROR;

    *((uint16_t*)status) = (values[1] << 8) | values[0];

    return BQ76905_OK;
}

// Measurement Functions
BQ76905_Status bq76905_read_cell_voltage(uint8_t cell_num, uint16_t *millivolts)
{
    if (cell_num < 1 || cell_num > 5)
        return BQ76905_ERROR;

    uint8_t values[2];
    if (read_register((uint8_t)(BQ76905_REG_CELL_1_VOLTAGE + ((cell_num - 1) * 2)), values, 2) != BQ76905_OK)
        return BQ76905_ERROR;    
    
    *millivolts = (values[1] << 8) | values[0];
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_stack_voltage(uint16_t *millivolts)
{
    uint8_t values[2];
    if (read_register(BQ76905_REG_STACK_VOLTAGE, values, 2) != BQ76905_OK)
        return BQ76905_ERROR;
    
        *millivolts = (values[1] << 8) | values[0];
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_internal_temp(int16_t *celsius)
{
    uint8_t values[2];
    if (read_register(BQ76905_REG_INT_TEMPERATURE, values, 2) != BQ76905_OK)
        return BQ76905_ERROR;
        
    *celsius = (int16_t)((values[1] << 8) | values[0]);
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_ts_adc(uint16_t *adc_codes)
{
    uint8_t values[2];
    if (read_register(BQ76905_REG_TS_MEASUREMENT, values, 2) != BQ76905_OK)
        return BQ76905_ERROR;
    
    *adc_codes = (values[1] << 8) | values[0];
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_current(int16_t *current)
{
    uint8_t values[2];
    if (read_register(BQ76905_REG_CURRENT, values, 2) != BQ76905_OK)
        return BQ76905_ERROR;
    *current = (int16_t)((values[1] << 8) | values[0]);
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_read_cc1_current(int16_t *current)
{
    uint8_t values[2];
    if (read_register(BQ76905_REG_CC1_CURRENT, values, 2) != BQ76905_OK)
        return BQ76905_ERROR;
    
    *current = (int16_t)((values[1] << 8) | values[0]);
    
    return BQ76905_OK;
}

// Control Functions
BQ76905_Status bq76905_set_fet_control(bool chg_on, bool dsg_on, bool chg_off, bool dsg_off)
{
    uint8_t value = 0;
    value |= (dsg_on    ? 1 : 0) << 0;
    value |= (chg_on    ? 1 : 0) << 1;
    value |= (dsg_off   ? 1 : 0) << 2;
    value |= (chg_off   ? 1 : 0) << 3;
    
    return write_register(BQ76905_REG_FET_CONTROL, &value, 1);
}

BQ76905_Status bq76905_set_regout_control(uint8_t voltage, bool enable)
{
    uint8_t value = voltage & 0x07;
    value |= (enable ? 1 : 0) << 3;
    
    return write_register(BQ76905_REG_REGOUT_CONTROL, &value, 1);
}

BQ76905_Status bq76905_set_dsg_pwm_control(uint8_t time_off, uint8_t time_on, bool enable)
{
    uint8_t values[2];
    values[0] = time_off & 0xFF;  // First byte: time off duration
    values[1] = ((time_on & 0x7F) << 1) | (enable ? 1 : 0);  // Second byte: time on and enable bit
    
    return write_register(BQ76905_REG_DSG_PWM_CONTROL, values, 2);
}

BQ76905_Status bq76905_set_chg_pwm_control(uint8_t time_off, uint8_t time_on, bool enable)
{
    uint8_t values[2];
    values[0] = time_off & 0xFF;  // First byte: time off duration
    values[1] = ((time_on & 0x7F) << 1) | (enable ? 1 : 0);  // Second byte: time on and enable bit
    
    return write_register(BQ76905_REG_CHG_PWM_CONTROL, values, 2);
}

BQ76905_Status bq76905_enter_shutdown(void)
{
    const uint8_t shutdown_cmd = 0x10;
    if (write_register(shutdown_cmd, 0x00, 0) != BQ76905_OK)
        return BQ76905_ERROR;
    
    // Send second command within 4s
    if (write_register(shutdown_cmd, 0x00, 0) != BQ76905_OK)
        return BQ76905_ERROR;
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_exit_shutdown(void)
{
    // To exit shutdown, just try to communicate with the device
    return bq76905_init(bq76905_i2c_addr);
}

BQ76905_Status bq76905_enter_deepsleep(void)
{
    const uint8_t deepsleep_cmd = 0x0F;
    if (write_register(deepsleep_cmd, 0x00, 0) != BQ76905_OK)
        return BQ76905_ERROR;
    
    // Send second command within 4s
    if (write_register(deepsleep_cmd, 0x00, 0) != BQ76905_OK)
        return BQ76905_ERROR;
    
    return BQ76905_OK;
}

BQ76905_Status bq76905_exit_deepsleep(void)
{
    const uint8_t exit_deepsleep_cmd = 0x0E;
    return write_register(exit_deepsleep_cmd, 0x00, 0);
}