/**
 * @file bq25798.c
 * @brief BQ25798 MPPT Battery Driver Implementation
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-11
 */

#include <stddef.h>
#include <math.h>
#include "bq25798.h"

/* Register addresses */
#define BQ25798_CMD_MINIMAL_SYSTEM_VOLTAGE 0x00     ///< Minimal System Voltage
#define BQ25798_CMD_CHARGE_VOLTAGE_LIMIT 0x01       ///< Charge Voltage Limit
#define BQ25798_CMD_CHARGE_CURRENT_LIMIT 0x03       ///< Charge Current Limit
#define BQ25798_CMD_INPUT_VOLTAGE_LIMIT 0x05        ///< Input Voltage Limit
#define BQ25798_CMD_INPUT_CURRENT_LIMIT 0x06        ///< Input Current Limit
#define BQ25798_CMD_PRECHARGE_CONTROL 0x08          ///< Precharge Control
#define BQ25798_CMD_TERMINATION_CONTROL 0x09        ///< Termination Control
#define BQ25798_CMD_RECHARGE_CONTROL 0x0A           ///< Re-charge Control
#define BQ25798_CMD_VOTG_REGULATION 0x0B            ///< VOTG Regulation
#define BQ25798_CMD_IOTG_REGULATION 0x0D            ///< IOTG Regulation
#define BQ25798_CMD_TIMER_CONTROL 0x0E              ///< Timer Control
#define BQ25798_CMD_CHARGER_CONTROL_0 0x0F          ///< Charger Control 0
#define BQ25798_CMD_CHARGER_CONTROL_1 0x10          ///< Charger Control 1
#define BQ25798_CMD_CHARGER_CONTROL_2 0x11          ///< Charger Control 2
#define BQ25798_CMD_CHARGER_CONTROL_3 0x12          ///< Charger Control 3
#define BQ25798_CMD_CHARGER_CONTROL_4 0x13          ///< Charger Control 4
#define BQ25798_CMD_CHARGER_CONTROL_5 0x14          ///< Charger Control 5
#define BQ25798_CMD_MPPT_CONTROL 0x15               ///< MPPT Control
#define BQ25798_CMD_TEMPERATURE_CONTROL 0x16        ///< Temperature Control
#define BQ25798_CMD_NTC_CONTROL_0 0x17              ///< NTC Control 0
#define BQ25798_CMD_NTC_CONTROL_1 0x18              ///< NTC Control 1
#define BQ25798_CMD_ICO_CURRENT_LIMIT 0x19          ///< ICO Current Limit
#define BQ25798_CMD_CHARGER_STATUS_0 0x1B           ///< Charger Status 0
#define BQ25798_CMD_CHARGER_STATUS_1 0x1C           ///< Charger Status 1
#define BQ25798_CMD_CHARGER_STATUS_2 0x1D           ///< Charger Status 2
#define BQ25798_CMD_CHARGER_STATUS_3 0x1E           ///< Charger Status 3
#define BQ25798_CMD_CHARGER_STATUS_4 0x1F           ///< Charger Status 4
#define BQ25798_CMD_FAULT_STATUS_0 0x20             ///< FAULT Status 0
#define BQ25798_CMD_FAULT_STATUS_1 0x21             ///< FAULT Status 1
#define BQ25798_CMD_CHARGER_FLAG_0 0x22             ///< Charger Flag 0
#define BQ25798_CMD_CHARGER_FLAG_1 0x23             ///< Charger Flag 1
#define BQ25798_CMD_CHARGER_FLAG_2 0x24             ///< Charger Flag 2
#define BQ25798_CMD_CHARGER_FLAG_3 0x25             ///< Charger Flag 3
#define BQ25798_CMD_FAULT_FLAG_0 0x26               ///< FAULT Flag 0
#define BQ25798_CMD_FAULT_FLAG_1 0x27               ///< FAULT Flag 1
#define BQ25798_CMD_CHARGER_MASK_0 0x28             ///< Charger Mask 0
#define BQ25798_CMD_CHARGER_MASK_1 0x29             ///< Charger Mask 1
#define BQ25798_CMD_CHARGER_MASK_2 0x2A             ///< Charger Mask 2
#define BQ25798_CMD_CHARGER_MASK_3 0x2B             ///< Charger Mask 3
#define BQ25798_CMD_FAULT_MASK_0 0x2C               ///< FAULT Mask 0
#define BQ25798_CMD_FAULT_MASK_1 0x2D               ///< FAULT Mask 1
#define BQ25798_CMD_ADC_CONTROL 0x2E                ///< ADC Control
#define BQ25798_CMD_ADC_FUNCTION_DISABLE_0 0x2F     ///< ADC Function Disable 0
#define BQ25798_CMD_ADC_FUNCTION_DISABLE_1 0x30     ///< ADC Function Disable 1
#define BQ25798_CMD_IBUS_ADC 0x31                   ///< IBUS ADC
#define BQ25798_CMD_IBAT_ADC 0x33                   ///< IBAT ADC
#define BQ25798_CMD_VBUS_ADC 0x35                   ///< VBUS ADC
#define BQ25798_CMD_VAC1_ADC 0x37                   ///< VAC1 ADC
#define BQ25798_CMD_VAC2_ADC 0x39                   ///< VAC2 ADC
#define BQ25798_CMD_VBAT_ADC 0x3B                   ///< VBAT ADC
#define BQ25798_CMD_VSYS_ADC 0x3D                   ///< VSYS ADC
#define BQ25798_CMD_TS_ADC 0x3F                     ///< TS ADC
#define BQ25798_CMD_TDIE_ADC 0x41                   ///< TDIE ADC
#define BQ25798_CMD_DPLUS_ADC 0x43                  ///< D+ ADC
#define BQ25798_CMD_DMINUS_ADC 0x45                 ///< D- ADC
#define BQ25798_CMD_DPDM_DRIVER 0x47                ///< DPDM Driver
#define BQ25798_CMD_PART_INFORMATION 0x48           ///< Part Information

/**
 * @brief Helper function to write to a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Value to write to a register
 * @return
 */
static bq25798_status_t bq25798_write_register(const bq25798_t *dev, const uint8_t reg, const uint8_t value) {
    const uint8_t data[2] = {reg, value};
    return dev->io.i2c_write(dev->i2c_address, data, 2) == 0 ? BQ25798_OK : BQ25798_ERR_I2C;
}

/**
 * @brief Helper function to read data from a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Pointer to data variable
 * @return
 */
static bq25798_status_t bq25798_read_register(const bq25798_t *dev, const uint8_t reg, uint8_t *value) {
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return BQ25798_ERR_I2C;

    return dev->io.i2c_read(dev->i2c_address, value, 1) == 0 ? BQ25798_OK : BQ25798_ERR_I2C;
}

/**
 * @brief Helper function to write multiple bytes to a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param data Pointer to data array
 * @param length Length of data array
 * @return
 */
static bq25798_status_t bq25798_write_registers(const bq25798_t *dev, const uint8_t reg, const uint8_t *data, const size_t length) {
    uint8_t buffer[length + 1];
    buffer[0] = reg;
    for (size_t i = 0; i < length; i++) {
        buffer[i + 1] = data[i];
    }

    return dev->io.i2c_write(dev->i2c_address, buffer, length + 1) == 0 ? BQ25798_OK : BQ25798_ERR_I2C;
}

/**
 * @brief Helper function to read multiple bytes from a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param data Pointer to data array
 * @param length Length of data array
 * @return
 */
static bq25798_status_t bq25798_read_registers(const bq25798_t *dev, const uint8_t reg, uint8_t *data, const size_t length) {
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return BQ25798_ERR_I2C;

    return dev->io.i2c_read(dev->i2c_address, data, length) == 0 ? BQ25798_OK : BQ25798_ERR_I2C;
}

/**
 * @brief Helper function to insert bits into a register
 * @param reg Pointer to the register array
 * @param reg_len Length of the register array
 * @param bits Bits to insert into the register
 * @param bits_len Length of the bits to insert into the register
 * @param shift Number of bits to shift the data into the register
 * @return bq25798_status_t Error code
 */
static bq25798_status_t bq25798_register_bits(uint8_t *reg, const size_t reg_len, uint32_t bits, const uint32_t bits_len, const uint32_t shift) {
    if (!reg || !reg_len || !bits || !bits_len) return BQ25798_ERR_NULL;
    if (shift + bits_len > reg_len * 8) return BQ25798_ERR_INVALID_ARG;

    const uint32_t mask = (1U << bits_len) - 1;
    bits &= mask;

    for (uint32_t i = 0; i < bits_len; ++i) {
        const uint32_t bit_pos = shift + i;
        const uint32_t byte_index = bit_pos / 8;
        const uint8_t bit_index = bit_pos % 8;

        const uint8_t bit = (bits >> i) & 0x1;

        reg[byte_index] &= ~(1 << bit_index);
        reg[byte_index] |= (bit << bit_index);
    }

    return BQ25798_OK;
}

/**
 * @brief Convert status error messages to human-readable strings
 * @param status Status to be converted to a string
 * @return String from status
 */
const char* bq25798_stat_error(const bq25798_status_t status) {
    switch (status) {
        case BQ25798_OK:                return "OK";
        case BQ25798_ERR_I2C:           return "I2C communication failed";
        case BQ25798_ERR_TIMEOUT:       return "Timeout occurred";
        case BQ25798_ERR_NULL:          return "Null pointer";
        case BQ25798_ERR_INVALID_ARG:   return "Invalid argument";
        default:                        return "Unknown error";
    }
}

/**
 * @brief Initialize the BQ25798 driver
 * @param dev Pointer to driver handle
 * @param address I2C address (0 to use default)
 * @param io Interface structure with platform-specific functions
 * @return bq25798_status_t Error code
 */
bq25798_status_t bq25798_init(bq25798_t *dev, const uint8_t address, const bq25798_interface_t io){
    if (!dev || !io.i2c_write || ! io.i2c_read || !io.delay_ms) return BQ25798_ERR_NULL;

    dev->i2c_address = address ? address : BQ25798_I2C_ADDR;
    dev->io = io;
    dev->initialized = false;

    const uint8_t reg = BQ25798_CMD_PART_INFORMATION;
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return BQ25798_ERR_I2C; // Try to read part number to check if I2C is working
    uint8_t value;
    io.i2c_read(dev->i2c_address, &value, 1);

    const uint8_t partNumber = (value >> 3) & 0x07;

    if (partNumber != 0x03) return BQ25798_ERR_I2C;

    dev->initialized = true;

    return BQ25798_OK;
}

/**
 * @brief Get the minimal system voltage
 * @param dev Pointer to driver handle
 * @param vsysmin Pointer to minimum system voltage variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsysmin(bq25798_t *dev, int *vsysmin) {
    if (!dev || !dev->initialized || !vsysmin) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MINIMAL_SYSTEM_VOLTAGE, &data);
    if (status != BQ25798_OK) return status;

    *vsysmin = ((data & 0x3F) * 250) + 2500;

    return BQ25798_OK;
}

/**
 * @brief Set the minimal system voltage
 * @param dev Pointer to driver handle
 * @param vsysmin Minimum system voltage
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vsysmin(bq25798_t *dev, const int vsysmin) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    if (vsysmin < 2500 || vsysmin > 16000) return BQ25798_ERR_INVALID_ARG;

    const uint8_t val = (uint8_t)roundf(((float)vsysmin - 2500) / 250); // Convert VSYSMIN to bit version

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MINIMAL_SYSTEM_VOLTAGE, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), val, 6, 0); // Insert bits into register
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_MINIMAL_SYSTEM_VOLTAGE, data);
}

/**
 * @brief Get the charger voltage limit
 * @param dev Pointer to driver handle
 * @param vreg_lim Pointer to charger voltage limit variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_charge_limit_v(bq25798_t *dev, int *vreg_lim) {
    if (!dev || !dev->initialized || !vreg_lim) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_CHARGE_VOLTAGE_LIMIT, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    const uint16_t dataCombined = ((uint16_t)data[0] << 8) | data[1];

    *vreg_lim = (dataCombined & 0x0FF) * 10; // Extract bottom 11 bits and multiply by 10mV

    return BQ25798_OK;
}

/**
 * @brief Set the charger voltage limit
 * @param dev Pointer to driver handle
 * @param vreg_lim Charger voltage limit
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_charge_limit_v(bq25798_t *dev, const int vreg_lim) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    if (vreg_lim < 3000 || vreg_lim > 18800) return BQ25798_ERR_INVALID_ARG;

    const uint16_t val = (uint16_t)roundf((float)vreg_lim / 10); // Convert battery voltage limit to bit version

    uint8_t data[2];
    bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_CHARGE_VOLTAGE_LIMIT, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(data, sizeof(data), val, 11, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_registers(dev, BQ25798_CMD_CHARGE_VOLTAGE_LIMIT, data, sizeof(data));
}

/**
 * @brief Get the charger current limit
 * @param dev Pointer to driver handle
 * @param ireg_lim Pointer to charger current limit variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_charge_limit_i(bq25798_t *dev, int *ireg_lim) {
    if (!dev || !dev->initialized || !ireg_lim) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_CHARGE_CURRENT_LIMIT, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    const uint16_t dataCombined = ((uint16_t)data[0] << 8) | data[1];

    *ireg_lim = (dataCombined & 0x01FF) * 10; // Extract bottom 9 bits and multiply by 10mA

    return BQ25798_OK;
}

/**
 * @brief Set the charger current limit
 * @param dev Pointer to driver handle
 * @param ireg_lim Charger current limit
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_charge_limit_i(bq25798_t *dev, const int ireg_lim) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    if (ireg_lim < 50 || ireg_lim > 5000) return BQ25798_ERR_INVALID_ARG;

    const uint16_t val = (uint16_t)roundf((float)ireg_lim / 10);

    uint8_t data[2];
    bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_CHARGE_CURRENT_LIMIT, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(data, sizeof(data), val, 9, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_registers(dev, BQ25798_CMD_CHARGE_CURRENT_LIMIT, data, sizeof(data));
}

/**
 * @brief Get the input voltage limit
 * @param dev Pointer to driver handle
 * @param vin_lim Pointer to input voltage limit variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_input_limit_v(bq25798_t *dev, int *vin_lim) {
    if (!dev || !dev->initialized || !vin_lim) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_INPUT_VOLTAGE_LIMIT, &data);
    if (status != BQ25798_OK) return status;

    *vin_lim = data * 100;

    return BQ25798_OK;
}

/**
 * @brief Set the input voltage limit
 * @param dev Pointer to driver handle
 * @param vin_lim Input voltage limit variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_input_limit_v(bq25798_t *dev, const int vin_lim) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    if (vin_lim < 3600 || vin_lim > 22000) return BQ25798_ERR_INVALID_ARG;

    const uint8_t data = (uint8_t)roundf((float)vin_lim / 100);

    return bq25798_write_register(dev, BQ25798_CMD_INPUT_VOLTAGE_LIMIT, data);
}

/**
 * @brief Get the input current limit
 * @param dev Pointer to driver handle
 * @param iin_lim Pointer to input current limit variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_input_limit_i(bq25798_t *dev, int *iin_lim) {
    if (!dev || !dev->initialized || !iin_lim) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_INPUT_CURRENT_LIMIT, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    const uint16_t dataCombined = ((uint16_t)data[0] << 8) | data[1];

    *iin_lim = (dataCombined & 0x01FF) * 10;

    return BQ25798_OK;
}

/**
 * @brief Set the input current limit
 * @param dev Pointer to driver handle
 * @param iin_lim Input current limit
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_input_limit_i(bq25798_t *dev, const int iin_lim) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    if (iin_lim < 100 || iin_lim > 3300) return BQ25798_ERR_INVALID_ARG;

    const uint16_t val = (uint16_t)roundf((float)iin_lim / 10);

    uint8_t data[2];
    bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_INPUT_CURRENT_LIMIT, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(data, sizeof(data), val, 9, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_registers(dev, BQ25798_CMD_INPUT_CURRENT_LIMIT, data, sizeof(data));
}

/**
 * @brief Get the battery voltage threshold for precharge to fast charge transition
 * @param dev Pointer to driver handle
 * @param threshold Pointer to battery low voltage threshold variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbat_lowv(bq25798_t *dev, bq25798_vbat_lowv_t *threshold) {
    if (!dev || !dev->initialized || !threshold) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_PRECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *threshold = (bq25798_vbat_lowv_t)((data >> 6) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set the battery voltage threshold for precharge to fast charge transition
 * @param dev Pointer to driver handle
 * @param threshold Battery low voltage threshold
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vbat_lowv(bq25798_t *dev, const bq25798_vbat_lowv_t threshold) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_PRECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    data = (data & 0x3F) | ((threshold & 0x03) << 6);

    return bq25798_write_register(dev, BQ25798_CMD_PRECHARGE_CONTROL, data);
}

/**
 * @brief Get the precharge current limit
 * @param dev Pointer to driver handle
 * @param precharge_lim Pointer to precharge current limit variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_precharge_lim_i(bq25798_t *dev, int *precharge_lim) {
    if (!dev || !dev->initialized || !precharge_lim) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_PRECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *precharge_lim = (data & 0x3F) * 40;

    return BQ25798_OK;
}

/**
 * @brief Set the precharge current limit
 * @param dev Pointer to driver handle
 * @param precharge_lim Precharge current limit
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_precharge_lim_i(bq25798_t *dev, const int precharge_lim) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    if (precharge_lim < 40 || precharge_lim > 2000) return BQ25798_ERR_INVALID_ARG;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_PRECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    const uint8_t val = (uint8_t)roundf((float)precharge_lim / 40);
    data = (data & 0xC0) | (val & 0x3F);

    return bq25798_write_register(dev, BQ25798_CMD_PRECHARGE_CONTROL, data);
}

/**
 * @brief Reset all settings to default values
 * @param dev Pointer to driver handle
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_reset_settings(bq25798_t *dev) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TERMINATION_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), 0x1, 1, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_TERMINATION_CONTROL, data);
}

/**
 * @brief Get the watchdog timer charge action setting
 * @param dev Pointer to driver handle
 * @param stopOnWDT Pointer to watchdog timer action variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_wd_chg(bq25798_t *dev, bool *stopOnWDT) {
    if (!dev || !dev->initialized || !stopOnWDT) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TERMINATION_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *stopOnWDT = (data >> 5) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set the watchdog timer charge action
 * @param dev Pointer to driver handle
 * @param stopOnWDT Stop charging on watchdog timer expiration
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_wd_chg(bq25798_t *dev, const bool stopOnWDT) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TERMINATION_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), stopOnWDT, 1, 5);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_TERMINATION_CONTROL, data);
}

/**
 * @brief Get the termination current
 * @param dev Pointer to driver handle
 * @param term_curr Pointer to termination current variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_termination_curr(bq25798_t *dev, int *term_curr) {
    if (!dev || !dev->initialized || !term_curr) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TERMINATION_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *term_curr = (data & 0x1F) * 40;

    return BQ25798_OK;
}

/**
 * @brief Set the termination current
 * @param dev Pointer to driver handle
 * @param term_curr Termination current
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_termination_curr(bq25798_t *dev, const int term_curr) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    if (term_curr < 40 || term_curr > 1000) return BQ25798_ERR_INVALID_ARG;

    const uint8_t val = (uint8_t)roundf((float)term_curr / 40);

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TERMINATION_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), val, 4, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_TERMINATION_CONTROL, data);
}

/**
 * @brief Get the battery cell count
 * @param dev Pointer to driver handle
 * @param cellCount Pointer to cell count variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_cell_count(bq25798_t *dev, bq25798_cell_count_t *cellCount) {
    if (!dev || !dev->initialized|| !cellCount) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_RECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *cellCount = (bq25798_cell_count_t)((data >> 6) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set the battery cell count
 * @param dev Pointer to driver handle
 * @param cellCount Battery cell count
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_cell_count(bq25798_t *dev, const bq25798_cell_count_t cellCount) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_RECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), cellCount, 2, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_RECHARGE_CONTROL, data);
}

/**
 * @brief Get the battery recharge deglitch time
 * @param dev Pointer to driver handle
 * @param deglitchTime Pointer to deglitch time variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_deglitch_time(bq25798_t *dev, bq25798_trechg_time_t *deglitchTime) {
    if (!dev || !dev->initialized || !deglitchTime) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_RECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *deglitchTime = (bq25798_trechg_time_t)((data >> 4) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set the battery recharge deglitch time
 * @param dev Pointer to driver handle
 * @param deglitchTime Battery recharge deglitch time
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_deglitch_time(bq25798_t *dev, const bq25798_trechg_time_t deglitchTime) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_RECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), deglitchTime, 2, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_RECHARGE_CONTROL, data);
}

/**
 * @brief Get the battery recharge threshold offset
 * @param dev Pointer to driver handle
 * @param thresholdOffset Pointer to threshold offset variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_recharge_threshold_offset(bq25798_t *dev, int *thresholdOffset) {
    if (!dev || !dev->initialized || !thresholdOffset) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_RECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *thresholdOffset = ((data & 0x0F) * 50) + 50;

    return BQ25798_OK;
}

/**
 * @brief Set the battery recharge threshold offset
 * @param dev Pointer to driver handle
 * @param thresholdOffset Battery recharge threshold offset (below VREG) in mV
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_recharge_threshold_offset(bq25798_t *dev, const int thresholdOffset) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    if (thresholdOffset < 50 || thresholdOffset > 800) return BQ25798_ERR_INVALID_ARG;

    const uint8_t val = (uint8_t)roundf(((float)thresholdOffset - 50) / 50);

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_RECHARGE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), val, 2, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_RECHARGE_CONTROL, data);
}

/**
 * @brief Get automatic battery discharge on OVP fault setting
 * @param dev Pointer to driver handle
 * @param auto_ibatdis Pointer to auto discharge setting variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_auto_ovp_batt_discharge(bq25798_t *dev, bool *auto_ibatdis) {
    if (!dev || !dev->initialized || !auto_ibatdis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *auto_ibatdis = (data >> 7) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set automatic battery discharge on OVP fault
 * @param dev Pointer to driver handle
 * @param auto_ibatdis Enable auto battery discharge during OVP fault
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_auto_ovp_batt_discharge(bq25798_t *dev, const bool auto_ibatdis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), auto_ibatdis, 1, 7);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, data);
}

/**
 * @brief Get forced battery discharge setting
 * @param dev Pointer to driver handle
 * @param force_ibatdis Pointer to forced discharge setting variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_force_ovp_batt_discharge(bq25798_t *dev, bool *force_ibatdis) {
    if (!dev || !dev->initialized || !force_ibatdis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *force_ibatdis = (data >> 6) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set forced battery discharge
 * @param dev Pointer to driver handle
 * @param force_ibatdis Force battery discharge current
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_force_ovp_batt_discharge(bq25798_t *dev, const bool force_ibatdis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), force_ibatdis, 1, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, data);
}

/**
 * @brief Get charging enable status
 * @param dev Pointer to driver handle
 * @param en_chg Pointer to charging enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_charge_enable(bq25798_t *dev, bool *en_chg) {
    if (!dev || !dev->initialized || !en_chg) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *en_chg = (data >> 5) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set charging enable
 * @param dev Pointer to driver handle
 * @param en_chg Enable charging
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_charge_enable(bq25798_t *dev, const bool en_chg) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_chg, 1, 5);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, data);
}

/**
 * @brief Get input current optimizer (ICO) enable status
 * @param dev Pointer to driver handle
 * @param en_ico Pointer to ICO enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ico_enable(bq25798_t *dev, bool *en_ico) {
    if (!dev || !dev->initialized || !en_ico) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *en_ico = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set input current optimizer (ICO) enable
 * @param dev Pointer to driver handle
 * @param en_ico Enable ICO
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ico_enable(bq25798_t *dev, const bool en_ico) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_ico, 1, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, data);
}

/**
 * @brief Get forced ICO enable status
 * @param dev Pointer to driver handle
 * @param force_ico Pointer to forced ICO enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_force_ico_enable(bq25798_t *dev, bool *force_ico) {
    if (!dev || !dev->initialized || !force_ico) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *force_ico = (data >> 3) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set forced ICO enable
 * @param dev Pointer to driver handle
 * @param force_ico Force start ICO
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_force_ico_enable(bq25798_t *dev, const bool force_ico) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), force_ico, 1, 3);

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, data);
}

/**
 * @brief Get high-impedance (HIZ) mode status
 * @param dev Pointer to driver handle
 * @param en_hiz Pointer to HIZ mode variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_hiz_mode(bq25798_t *dev, bool *en_hiz) {
    if (!dev || !dev->initialized || !en_hiz) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *en_hiz = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set high-impedance (HIZ) mode
 * @param dev Pointer to driver handle
 * @param en_hiz Enable HIZ mode
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_hiz_mode(bq25798_t *dev, const bool en_hiz) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_hiz, 1, 2);

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, data);
}

/**
 * @brief Get charge termination enable status
 * @param dev Pointer to driver handle
 * @param en_term Pointer to termination enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_termination_enable(bq25798_t *dev, bool *en_term) {
    if (!dev || !dev->initialized || !en_term) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *en_term = (data >> 1) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set charge termination enable
 * @param dev Pointer to driver handle
 * @param en_term Enable charge termination
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_termination_enable(bq25798_t *dev, const bool en_term) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_term, 1, 1);

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, data);
}

/**
 * @brief Get backup mode enable status
 * @param dev Pointer to driver handle
 * @param en_backup Pointer to backup mode enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_backup_mode_enable(bq25798_t *dev, bool *en_backup) {
    if (!dev || !dev->initialized || !en_backup) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *en_backup = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set backup mode enable
 * @param dev Pointer to driver handle
 * @param en_backup Enable backup mode
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_backup_mode_enable(bq25798_t *dev, const bool en_backup) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_backup, 1, 0);

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_0, data);
}

/**
 * @brief Get backup mode threshold
 * @param dev Pointer to driver handle
 * @param vbckup_threshold Pointer to backup threshold variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_backup_mode_threshold(bq25798_t *dev, bq25798_vbus_backup_t *vbckup_threshold) {
    if (!dev || !dev->initialized || !vbckup_threshold) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    *vbckup_threshold = (bq25798_vbus_backup_t)((data >> 6) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set backup mode threshold
 * @param dev Pointer to driver handle
 * @param vbckup_threshold Backup mode threshold
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_backup_mode_threshold(bq25798_t *dev, const bq25798_vbus_backup_t vbckup_threshold) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vbckup_threshold, 2, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, data);
}

/**
 * @brief Get VAC overvoltage protection threshold
 * @param dev Pointer to driver handle
 * @param vac_threshold Pointer to VAC OVP threshold variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac_ovp(bq25798_t *dev, bq25798_vac_ovp_t *vac_threshold) {
    if (!dev || !dev->initialized || !vac_threshold) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    *vac_threshold = (bq25798_vac_ovp_t)((data >> 4) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set VAC overvoltage protection threshold
 * @param dev Pointer to driver handle
 * @param vac_threshold VAC OVP threshold
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vac_ovp(bq25798_t *dev, const bq25798_vac_ovp_t vac_threshold) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vac_threshold, 2, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, data);
}

/**
 * @brief Reset the watchdog timer
 * @param dev Pointer to driver handle
 * @param wdt_rst Reset watchdog timer
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_reset_watchdog(bq25798_t *dev, const bool wdt_rst) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), wdt_rst, 1, 3);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, data);
}

/**
 * @brief Get watchdog timer setting
 * @param dev Pointer to driver handle
 * @param timer Pointer to watchdog timer variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_watchdog(bq25798_t *dev, bq25798_wdt_t *timer) {
    if (!dev || !dev->initialized || !timer) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    *timer = (bq25798_wdt_t)(data & 0x07);

    return BQ25798_OK;
}

/**
 * @brief Set watchdog timer
 * @param dev Pointer to driver handle
 * @param timer Watchdog timer setting
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_watchdog(bq25798_t *dev, const bq25798_wdt_t timer) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), timer, 3, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_1, data);
}

/**
 * @brief Get HVDCP 12V enable status
 * @param dev Pointer to driver handle
 * @param en_12v Pointer to 12V enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_hvdcp_12V_enable(bq25798_t *dev, bool *en_12v) {
    if (!dev || !dev->initialized || !en_12v) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    *en_12v = (data >> 5) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set HVDCP 12V enable
 * @param dev Pointer to driver handle
 * @param en_12v Enable 12V HVDCP
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_hvdcp_12V_enable(bq25798_t *dev, const bool en_12v) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_12v, 1, 5);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, data);
}

/**
 * @brief Get HVDCP 9V enable status
 * @param dev Pointer to driver handle
 * @param en_9v Pointer to 9V enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_hvdcp_9V_enable(bq25798_t *dev, bool *en_9v) {
    if (!dev || !dev->initialized || !en_9v) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    *en_9v = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set HVDCP 9V enable
 * @param dev Pointer to driver handle
 * @param en_9v Enable 9V HVDCP
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_hvdcp_9V_enable(bq25798_t *dev, const bool en_9v) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_9v, 1, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, data);
}

/**
 * @brief Get HVDCP enable status
 * @param dev Pointer to driver handle
 * @param hvdcp_en Pointer to HVDCP enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_hvdcp_enable(bq25798_t *dev, bool *hvdcp_en) {
    if (!dev || !dev->initialized || !hvdcp_en) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    *hvdcp_en = (data >> 3) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set HVDCP enable
 * @param dev Pointer to driver handle
 * @param hvdcp_en Enable high voltage DCP
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_hvdcp_enable(bq25798_t *dev, const bool hvdcp_en) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), hvdcp_en, 1, 3);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, data);
}

/**
 * @brief Get ship FET mode control
 * @param dev Pointer to driver handle
 * @param sfet_mode Pointer to SFET mode variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_sfet_mode(bq25798_t *dev, bq25798_sdrv_ctrl_t *sfet_mode) {
    if (!dev || !dev->initialized || !sfet_mode) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    *sfet_mode = (bq25798_sdrv_ctrl_t)((data >> 1) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set ship FET mode control
 * @param dev Pointer to driver handle
 * @param sfet_mode SFET mode control
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_sfet_mode(bq25798_t *dev, const bq25798_sdrv_ctrl_t sfet_mode) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), sfet_mode, 2, 1);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, data);
}

/**
 * @brief Get ship FET delay setting
 * @param dev Pointer to driver handle
 * @param sdrv_dly Pointer to SFET delay variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_sfet_delay(bq25798_t *dev, bool *sdrv_dly) {
    if (!dev || !dev->initialized || !sdrv_dly) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    *sdrv_dly = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set ship FET delay
 * @param dev Pointer to driver handle
 * @param sdrv_dly Add 10s delay for SFET actions
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_sfet_delay(bq25798_t *dev, const bool sdrv_dly) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), sdrv_dly, 1, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_2, data);
}

/**
 * @brief Get ACDRV disable status
 * @param dev Pointer to driver handle
 * @param dis_acdrv Pointer to ACDRV disable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_acdrv_disable(bq25798_t *dev, bool *dis_acdrv) {
    if (!dev || !dev->initialized || !dis_acdrv) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    *dis_acdrv = (data >> 7) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set ACDRV disable
 * @param dev Pointer to driver handle
 * @param dis_acdrv Disconnect ACDRV1 & ACDRV2
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_acdrv_disable(bq25798_t *dev, const bool dis_acdrv) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), dis_acdrv, 1, 7);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, data);
}

/**
 * @brief Get PFM disable in forward mode status
 * @param dev Pointer to driver handle
 * @param pfm_fwd_dis Pointer to PFM disable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_pfm_disable(bq25798_t *dev, bool *pfm_fwd_dis) {
    if (!dev || !dev->initialized || !pfm_fwd_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    *pfm_fwd_dis = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set PFM disable in forward mode
 * @param dev Pointer to driver handle
 * @param pfm_fwd_dis Disable PFM in forward mode
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_pfm_disable(bq25798_t *dev, const bool pfm_fwd_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), pfm_fwd_dis, 1, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, data);
}

/**
 * @brief Get ship mode wakeup delay
 * @param dev Pointer to driver handle
 * @param wkup_dly Pointer to wakeup delay variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ship_wakeup_delay(bq25798_t *dev, bq25798_wkup_dly_t *wkup_dly) {
    if (!dev || !dev->initialized || !wkup_dly) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    *wkup_dly = (bq25798_wkup_dly_t)((data >> 3) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set ship mode wakeup delay
 * @param dev Pointer to driver handle
 * @param wkup_dly Time to pull low the QON pin
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ship_wakeup_delay(bq25798_t *dev, const bq25798_wkup_dly_t wkup_dly) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), wkup_dly, 1, 3);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, data);
}

/**
 * @brief Get BATFET LDO disable status
 * @param dev Pointer to driver handle
 * @param dis_ldo Pointer to LDO disable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_batfet_ldo_disable(bq25798_t *dev, bool *dis_ldo) {
    if (!dev || !dev->initialized || !dis_ldo) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    *dis_ldo = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set BATFET LDO disable
 * @param dev Pointer to driver handle
 * @param dis_ldo Disable BATFET LDO in pre-charge stage
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_batfet_ldo_disable(bq25798_t *dev, const bool dis_ldo) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), dis_ldo, 1, 2);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, data);
}

/**
 * @brief Get OOA disable in forward mode status
 * @param dev Pointer to driver handle
 * @param dis_fwd_ooa Pointer to OOA disable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ooa_disable(bq25798_t *dev, bool *dis_fwd_ooa) {
    if (!dev || !dev->initialized || !dis_fwd_ooa) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    *dis_fwd_ooa = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set OOA disable in forward mode
 * @param dev Pointer to driver handle
 * @param dis_fwd_ooa Disable OOA in forward mode
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ooa_disable(bq25798_t *dev, const bool dis_fwd_ooa) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), dis_fwd_ooa, 1, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_3, data);
}

/**
 * @brief Get ACDRV2 enable status
 * @param dev Pointer to driver handle
 * @param en_acdrv2 Pointer to ACDRV2 enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_acdrv2_enable(bq25798_t *dev, bool *en_acdrv2) {
    if (!dev || !dev->initialized || !en_acdrv2) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    *en_acdrv2 = (data >> 7) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set ACDRV2 enable
 * @param dev Pointer to driver handle
 * @param en_acdrv2 Enable ACDRV2
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_acdrv2_enable(bq25798_t *dev, const bool en_acdrv2) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_acdrv2, 1, 7);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, data);
}

/**
 * @brief Get ACDRV1 enable status
 * @param dev Pointer to driver handle
 * @param en_acdrv1 Pointer to ACDRV1 enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_acdrv1_enable(bq25798_t *dev, bool *en_acdrv1) {
    if (!dev || !dev->initialized || !en_acdrv1) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    *en_acdrv1 = (data >> 6) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set ACDRV1 enable
 * @param dev Pointer to driver handle
 * @param en_acdrv1 Enable ACDRV1
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_acdrv1_enable(bq25798_t *dev, const bool en_acdrv1) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_acdrv1, 1, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, data);
}

/**
 * @brief Get PWM switching frequency
 * @param dev Pointer to driver handle
 * @param pwm_freq Pointer to PWM frequency variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_pwm_freq(bq25798_t *dev, bq25798_pwm_freq_t *pwm_freq) {
    if (!dev || !dev->initialized || !pwm_freq) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    *pwm_freq = (bq25798_pwm_freq_t)((data >> 5) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set PWM switching frequency
 * @param dev Pointer to driver handle
 * @param pwm_freq PWM switching frequency
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_pwm_freq(bq25798_t *dev, const bq25798_pwm_freq_t pwm_freq) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), pwm_freq, 1, 5);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, data);
}

/**
 * @brief Get STAT pin disable status
 * @param dev Pointer to driver handle
 * @param dis_stat Pointer to STAT pin disable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_stat_pin_disable(bq25798_t *dev, bool *dis_stat) {
    if (!dev || !dev->initialized || !dis_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    *dis_stat = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set STAT pin disable
 * @param dev Pointer to driver handle
 * @param dis_stat Disable the STAT pin output
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_stat_pin_disable(bq25798_t *dev, const bool dis_stat) {
    if (!dev) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), dis_stat, 1, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, data);
}

/**
 * @brief Get VSYS short protection disable status
 * @param dev Pointer to driver handle
 * @param dis_vsys_short Pointer to VSYS short disable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsys_short_disable(bq25798_t *dev, bool *dis_vsys_short) {
    if (!dev || !dev->initialized || !dis_vsys_short) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    *dis_vsys_short = (data >> 3) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set VSYS short protection disable
 * @param dev Pointer to driver handle
 * @param dis_vsys_short Disable forward mode VSYS short hiccup protection
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vsys_short_disable(bq25798_t *dev, const bool dis_vsys_short) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), dis_vsys_short, 1, 3);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, data);
}

/**
 * @brief Get forced VINDPM detection status
 * @param dev Pointer to driver handle
 * @param force_vindpm_det Pointer to forced VINDPM detection variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_force_vindpm_detect(bq25798_t *dev, bool *force_vindpm_det) {
    if (!dev || !dev->initialized || !force_vindpm_det) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    *force_vindpm_det = (data >> 1) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set forced VINDPM detection
 * @param dev Pointer to driver handle
 * @param force_vindpm_det Force VINDPM detection
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_force_vindpm_detect(bq25798_t *dev, const bool force_vindpm_det) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), force_vindpm_det, 1, 1);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, data);
}

/**
 * @brief Get IBUS OCP enable status
 * @param dev Pointer to driver handle
 * @param en_ibus_ocp Pointer to IBUS OCP enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibus_ocp_enable(bq25798_t *dev, bool *en_ibus_ocp) {
    if (!dev || !dev->initialized || !en_ibus_ocp) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    *en_ibus_ocp = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set IBUS OCP enable
 * @param dev Pointer to driver handle
 * @param en_ibus_ocp Enable IBUS_OCP in forward mode
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ibus_ocp_enable(bq25798_t *dev, const bool en_ibus_ocp) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_ibus_ocp, 1, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_4, data);
}

/**
 * @brief Get ship FET present status
 * @param dev Pointer to driver handle
 * @param sfet_present Pointer to ship FET present variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_sfet_present(bq25798_t *dev, bool *sfet_present) {
    if (!dev || !dev->initialized || !sfet_present) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    *sfet_present = (data >> 7) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set ship FET present
 * @param dev Pointer to driver handle
 * @param sfet_present Define if ship FET is populated or not
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_sfet_present(bq25798_t *dev, const bool sfet_present) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), sfet_present, 1, 7);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, data);
}

/**
 * @brief Get IBAT discharge current sensing enable status
 * @param dev Pointer to driver handle
 * @param en_ibat Pointer to IBAT enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibat_enable(bq25798_t *dev, bool *en_ibat) {
    if (!dev || !dev->initialized || !en_ibat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    *en_ibat = (data >> 5) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set IBAT discharge current sensing enable
 * @param dev Pointer to driver handle
 * @param en_ibat IBAT discharge current sensing
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ibat_enable(bq25798_t *dev, const bool en_ibat) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_ibat, 1, 5);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, data);
}

/**
 * @brief Get internal IINDPM enable status
 * @param dev Pointer to driver handle
 * @param en_iindpm Pointer to IINDPM enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_iindpm_enable(bq25798_t *dev, bool *en_iindpm) {
    if (!dev || !dev->initialized || !en_iindpm) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    *en_iindpm = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set internal IINDPM enable
 * @param dev Pointer to driver handle
 * @param en_iindpm Enable the internal IINDPM input current regulation
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_iindpm_enable(bq25798_t *dev, const bool en_iindpm) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_iindpm, 1, 2);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, data);
}

/**
 * @brief Get external ILIM enable status
 * @param dev Pointer to driver handle
 * @param en_extilim Pointer to external ILIM enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ext_ilim_enable(bq25798_t *dev, bool *en_extilim) {
    if (!dev || !dev->initialized || !en_extilim) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    *en_extilim = (data >> 1) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set external ILIM enable
 * @param dev Pointer to driver handle
 * @param en_extilim Enable the external ILIM_HIZ pin input current regulation
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ext_ilim_enable(bq25798_t *dev, const bool en_extilim) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_extilim, 1, 1);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, data);
}

/**
 * @brief Get battery discharge OCP enable status
 * @param dev Pointer to driver handle
 * @param en_batoc Pointer to battery OCP enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_bat_ocp_enable(bq25798_t *dev, bool *en_batoc) {
    if (!dev || !dev->initialized || !en_batoc) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    *en_batoc = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set battery discharge OCP enable
 * @param dev Pointer to driver handle
 * @param en_batoc Enable the battery discharging current OCP
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_bat_ocp_enable(bq25798_t *dev, const bool en_batoc) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_batoc, 1, 0);

    return bq25798_write_register(dev, BQ25798_CMD_CHARGER_CONTROL_5, data);
}

/**
 * @brief Get VOC percentage setting
 * @param dev Pointer to driver handle
 * @param voc_pct Pointer to VOC percentage variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_voc_pct(bq25798_t *dev, bq25798_voc_pct_t *voc_pct) {
    if (!dev || !dev->initialized || !voc_pct) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MPPT_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *voc_pct = (bq25798_voc_pct_t)((data >> 5) & 0x07);

    return BQ25798_OK;
}

/**
 * @brief Set VOC percentage
 * @param dev Pointer to driver handle
 * @param voc_pct Set VINDPM as a percentage of the VBUS open circuit voltage
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_voc_pct(bq25798_t *dev, const bq25798_voc_pct_t voc_pct) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MPPT_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), voc_pct, 3, 5);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_MPPT_CONTROL, data);
}

/**
 * @brief Get VOC delay setting
 * @param dev Pointer to driver handle
 * @param voc_dly Pointer to VOC delay variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_voc_delay(bq25798_t *dev, bq25798_voc_dly_t *voc_dly) {
    if (!dev || !dev->initialized || !voc_dly) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MPPT_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *voc_dly = (bq25798_voc_dly_t)((data >> 3) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set VOC delay
 * @param dev Pointer to driver handle
 * @param voc_dly Delay after converter stops switching before VOC is measured
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_voc_delay(bq25798_t *dev, const bq25798_voc_dly_t voc_dly) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MPPT_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), voc_dly, 2, 3);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_MPPT_CONTROL, data);
}

/**
 * @brief Get VOC measurement rate
 * @param dev Pointer to driver handle
 * @param voc_rate Pointer to VOC measurement rate variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_voc_measurement_rate(bq25798_t *dev, bq25798_voc_rate_t *voc_rate) {
    if (!dev || !dev->initialized || !voc_rate) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MPPT_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *voc_rate = (bq25798_voc_rate_t)((data >> 1) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set VOC measurement rate
 * @param dev Pointer to driver handle
 * @param voc_rate Time interval for VBUS open circuit voltage measurements
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_voc_measurement_rate(bq25798_t *dev, const bq25798_voc_rate_t voc_rate) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MPPT_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), voc_rate, 2, 1);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_MPPT_CONTROL, data);
}

/**
 * @brief Get MPPT enable status
 * @param dev Pointer to driver handle
 * @param en_mppt Pointer to MPPT enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_mppt_enable(bq25798_t *dev, bool *en_mppt) {
    if (!dev || !dev->initialized || !en_mppt) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MPPT_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *en_mppt = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set MPPT enable
 * @param dev Pointer to driver handle
 * @param en_mppt Enable the MPPT to measure the VBUS open circuit voltage
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_mppt_enable(bq25798_t *dev, const bool en_mppt) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_MPPT_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), en_mppt, 1, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_MPPT_CONTROL, data);
}

/**
 * @brief Get thermal regulation threshold
 * @param dev Pointer to driver handle
 * @param treg_threshold Pointer to thermal regulation threshold variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_thermal_reg_threshold(bq25798_t *dev, bq25798_treg_t *treg_threshold) {
    if (!dev || !dev->initialized || !treg_threshold) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *treg_threshold = (bq25798_treg_t)((data >> 6) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set thermal regulation threshold
 * @param dev Pointer to driver handle
 * @param treg_threshold Thermal regulation threshold
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_thermal_reg_threshold(bq25798_t *dev, const bq25798_treg_t treg_threshold) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), treg_threshold, 2, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, data);
}

/**
 * @brief Get thermal shutdown threshold
 * @param dev Pointer to driver handle
 * @param tshut_threshold Pointer to thermal shutdown threshold variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_thermal_shutdown_threshold(bq25798_t *dev, bq25798_tshut_t *tshut_threshold) {
    if (!dev || !dev->initialized || !tshut_threshold) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *tshut_threshold = (bq25798_tshut_t)((data >> 4) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set thermal shutdown threshold
 * @param dev Pointer to driver handle
 * @param tshut_threshold Thermal shutdown threshold
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_thermal_shutdown_threshold(bq25798_t *dev, const bq25798_tshut_t tshut_threshold) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), tshut_threshold, 2, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, data);
}

/**
 * @brief Get VBUS pulldown resistor enable status
 * @param dev Pointer to driver handle
 * @param vbus_pd_en Pointer to VBUS pulldown enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbus_pulldown(bq25798_t *dev, bool *vbus_pd_en) {
    if (!dev || !dev->initialized || !vbus_pd_en) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *vbus_pd_en = (data >> 3) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set VBUS pulldown resistor enable
 * @param dev Pointer to driver handle
 * @param vbus_pd_en Enable VBUS pull down resistor (6k Ohm)
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vbus_pulldown(bq25798_t *dev, const bool vbus_pd_en) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vbus_pd_en, 1, 3);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, data);
}

/**
 * @brief Get VAC1 pulldown resistor enable status
 * @param dev Pointer to driver handle
 * @param vac1_pd_en Pointer to VAC1 pulldown enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac1_pulldown(bq25798_t *dev, bool *vac1_pd_en) {
    if (!dev || !dev->initialized || !vac1_pd_en) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *vac1_pd_en = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set VAC1 pulldown resistor enable
 * @param dev Pointer to driver handle
 * @param vac1_pd_en Enable VAC1 pull down resistor
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vac1_pulldown(bq25798_t *dev, const bool vac1_pd_en) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vac1_pd_en, 1, 2);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, data);
}

/**
 * @brief Get VAC2 pulldown resistor enable status
 * @param dev Pointer to driver handle
 * @param vac2_pd_en Pointer to VAC2 pulldown enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac2_pulldown(bq25798_t *dev, bool *vac2_pd_en) {
    if (!dev || !dev->initialized || !vac2_pd_en) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *vac2_pd_en = (data >> 1) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set VAC2 pulldown resistor enable
 * @param dev Pointer to driver handle
 * @param vac2_pd_en Enable VAC2 pull down resistor
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vac2_pulldown(bq25798_t *dev, const bool vac2_pd_en) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vac2_pd_en, 1, 1);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, data);
}

/**
 * @brief Get backup ACFET1 enable status
 * @param dev Pointer to driver handle
 * @param bkup_acfet1_on Pointer to backup ACFET1 enable variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_backup_acfet1_on(bq25798_t *dev, bool *bkup_acfet1_on) {
    if (!dev || !dev->initialized || !bkup_acfet1_on) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *bkup_acfet1_on = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set backup ACFET1 enable
 * @param dev Pointer to driver handle
 * @param bkup_acfet1_on Turn on ACFET1 in backup mode
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_backup_acfet1_on(bq25798_t *dev, const bool bkup_acfet1_on) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), bkup_acfet1_on, 1, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_TEMPERATURE_CONTROL, data);
}

/**
 * @brief Get JEITA high temperature voltage setting
 * @param dev Pointer to driver handle
 * @param jeita_vset Pointer to JEITA voltage setting variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_jeita_high_temp_voltage_setting(bq25798_t *dev, bq25798_jeita_vset_t *jeita_vset) {
    if (!dev || !dev->initialized || !jeita_vset) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *jeita_vset = (bq25798_jeita_vset_t)((data >> 5) & 0x07);

    return BQ25798_OK;
}

/**
 * @brief Set JEITA high temperature voltage setting
 * @param dev Pointer to driver handle
 * @param jeita_vset JEITA high temperature range voltage setting
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_jeita_high_temp_voltage_setting(bq25798_t *dev, const bq25798_jeita_vset_t jeita_vset) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), jeita_vset, 3, 5);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_NTC_CONTROL_0, data);
}

/**
 * @brief Get JEITA high temperature current setting
 * @param dev Pointer to driver handle
 * @param jeita_iseth Pointer to JEITA current setting variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_jeita_high_temp_current_setting(bq25798_t *dev, bq25798_jeita_iseth_t *jeita_iseth) {
    if (!dev || !dev->initialized || !jeita_iseth) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *jeita_iseth = (bq25798_jeita_iseth_t)((data >> 3) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set JEITA high temperature current setting
 * @param dev Pointer to driver handle
 * @param jeita_iseth JEITA high temperature range current setting
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_jeita_high_temp_current_setting(bq25798_t *dev, const bq25798_jeita_iseth_t jeita_iseth) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), jeita_iseth, 2, 3);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_NTC_CONTROL_0, data);
}

/**
 * @brief Get JEITA low temperature current setting
 * @param dev Pointer to driver handle
 * @param jeita_isetc Pointer to JEITA low current setting variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_jeita_low_temp_current_setting(bq25798_t *dev, bq25798_jeita_isetc_t *jeita_isetc) {
    if (!dev || !dev->initialized || !jeita_isetc) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    *jeita_isetc = (bq25798_jeita_isetc_t)((data >> 1) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set JEITA low temperature current setting
 * @param dev Pointer to driver handle
 * @param jeita_isetc JEITA low temperature range current setting
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_jeita_low_temp_current_setting(bq25798_t *dev, const bq25798_jeita_isetc_t jeita_isetc) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), jeita_isetc, 2, 1);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_NTC_CONTROL_0, data);
}

/**
 * @brief Get TS cool temperature setting
 * @param dev Pointer to driver handle
 * @param ts_cool Pointer to TS cool setting variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_cool_setting(bq25798_t *dev, bq25798_ts_cool_t *ts_cool) {
    if (!dev || !dev->initialized || !ts_cool) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    *ts_cool = (bq25798_ts_cool_t)((data >> 6) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set TS cool temperature setting
 * @param dev Pointer to driver handle
 * @param ts_cool JEITA VT2 rising threshold
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ts_cool_setting(bq25798_t *dev, const bq25798_ts_cool_t ts_cool) {
    if (!dev) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), ts_cool, 2, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_NTC_CONTROL_1, data);
}

/**
 * @brief Get TS warm temperature setting
 * @param dev Pointer to driver handle
 * @param ts_warm Pointer to TS warm setting variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_warm_setting(bq25798_t *dev, bq25798_ts_warm_t *ts_warm) {
    if (!dev || !dev->initialized || !ts_warm) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    *ts_warm = (bq25798_ts_warm_t)((data >> 4) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set TS warm temperature setting
 * @param dev Pointer to driver handle
 * @param ts_warm JEITA VT3 falling threshold
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ts_warm_setting(bq25798_t *dev, const bq25798_ts_warm_t ts_warm) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), ts_warm, 2, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_NTC_CONTROL_1, data);
}

/**
 * @brief Get TS hot temperature setting
 * @param dev Pointer to driver handle
 * @param bhot Pointer to TS hot setting variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_hot_setting(bq25798_t *dev, bq25798_bhot_t *bhot) {
    if (!dev || !dev->initialized || !bhot) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    *bhot = (bq25798_bhot_t)((data >> 2) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Set TS hot temperature setting
 * @param dev Pointer to driver handle
 * @param bhot OTG mode TS HOT temperature threshold
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ts_hot_setting(bq25798_t *dev, const bq25798_bhot_t bhot) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), bhot, 2, 2);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_NTC_CONTROL_1, data);
}

/**
 * @brief Get TS cold temperature setting
 * @param dev Pointer to driver handle
 * @param bcold Pointer to TS cold setting variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_cold_setting(bq25798_t *dev, bq25798_bcold_t *bcold) {
    if (!dev || !dev->initialized || !bcold) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    *bcold = (bq25798_bcold_t)((data >> 1) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set TS cold temperature setting
 * @param dev Pointer to driver handle
 * @param bcold OTG mode TS COLD temperature threshold
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ts_cold_setting(bq25798_t *dev, const bq25798_bcold_t bcold) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), bcold, 1, 1);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_NTC_CONTROL_1, data);
}

/**
 * @brief Get TS ignore setting
 * @param dev Pointer to driver handle
 * @param ts_ignore Pointer to TS ignore variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_ignore(bq25798_t *dev, bool *ts_ignore) {
    if (!dev || !dev->initialized || !ts_ignore) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    *ts_ignore = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Set TS ignore setting
 * @param dev Pointer to driver handle
 * @param ts_ignore Ignore TS feedback
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ts_ignore(bq25798_t *dev, const bool ts_ignore) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_NTC_CONTROL_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), ts_ignore, 1, 0);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_NTC_CONTROL_1, data);
}

/**
 * @brief Get IINDPM status
 * @param dev Pointer to driver handle
 * @param iindpm_stat Pointer to IINDPM status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_iindpm_status(bq25798_t *dev, bool *iindpm_stat) {
    if (!dev || !dev->initialized || !iindpm_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *iindpm_stat = (bool)((data >> 7) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VINDPM status
 * @param dev Pointer to driver handle
 * @param vindpm_stat Pointer to VINDPM status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vindpm_status(bq25798_t *dev, bool *vindpm_stat) {
    if (!dev || !dev->initialized || !vindpm_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *vindpm_stat = (bool)((data >> 6) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get watchdog timer status
 * @param dev Pointer to driver handle
 * @param watchdog_stat Pointer to watchdog status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_watchdog_status(bq25798_t *dev, bool *watchdog_stat) {
    if (!dev || !dev->initialized || !watchdog_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *watchdog_stat = (bool)((data >> 5) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get power good status
 * @param dev Pointer to driver handle
 * @param pg_stat Pointer to power good status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_power_good_status(bq25798_t *dev, bool *pg_stat) {
    if (!dev || !dev->initialized || !pg_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *pg_stat = (bool)((data >> 3) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get AC2 present status
 * @param dev Pointer to driver handle
 * @param ac2_present_stat Pointer to AC2 present status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ac2_present_status(bq25798_t *dev, bool *ac2_present_stat) {
    if (!dev || !dev->initialized || !ac2_present_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *ac2_present_stat = (bool)((data >> 2) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get AC1 present status
 * @param dev Pointer to driver handle
 * @param ac1_present_stat Pointer to AC1 present status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ac1_present_status(bq25798_t *dev, bool *ac1_present_stat) {
    if (!dev || !dev->initialized || !ac1_present_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *ac1_present_stat = (bool)((data >> 1) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VBUS present status
 * @param dev Pointer to driver handle
 * @param vbus_present_stat Pointer to VBUS present status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbus_present_status(bq25798_t *dev, bool *vbus_present_stat) {
    if (!dev || !dev->initialized || !vbus_present_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *vbus_present_stat = (bool)(data & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get charger status
 * @param dev Pointer to driver handle
 * @param chg_stat Pointer to charger status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_charger_status(bq25798_t *dev, bq25798_chg_stat_t *chg_stat) {
    if (!dev || !dev->initialized || !chg_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_1, &data);
    if (status != BQ25798_OK) return status;

    *chg_stat = (bq25798_chg_stat_t)((data >> 5) & 0x07);

    return BQ25798_OK;
}

/**
 * @brief Get VBUS status
 * @param dev Pointer to driver handle
 * @param vbus_stat Pointer to VBUS status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbus_status(bq25798_t *dev, bq25798_vbus_stat_t *vbus_stat) {
    if (!dev || !dev->initialized || !vbus_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_1, &data);
    if (status != BQ25798_OK) return status;

    *vbus_stat = (bq25798_vbus_stat_t)((data >> 1) & 0x0F);

    return BQ25798_OK;
}

/**
 * @brief Get BC1.2 detection done status
 * @param dev Pointer to driver handle
 * @param bc_1_2_done_stat Pointer to BC1.2 done status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_bc_1_2_done_status(bq25798_t *dev, bool *bc_1_2_done_stat) {
    if (!dev || !dev->initialized || !bc_1_2_done_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_1, &data);
    if (status != BQ25798_OK) return status;

    *bc_1_2_done_stat = (bool)(data & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get ICO status
 * @param dev Pointer to driver handle
 * @param ico_stat Pointer to ICO status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ico_status(bq25798_t *dev, bq25798_ico_stat_t *ico_stat) {
    if (!dev || !dev->initialized || !ico_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_2, &data);
    if (status != BQ25798_OK) return status;

    *ico_stat = (bq25798_ico_stat_t)((data >> 6) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Get thermal regulation status
 * @param dev Pointer to driver handle
 * @param treg_stat Pointer to thermal regulation status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_treg_status(bq25798_t *dev, bool *treg_stat) {
    if (!dev || !dev->initialized || !treg_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_2, &data);
    if (status != BQ25798_OK) return status;

    *treg_stat = (bool)((data >> 2) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get D+/D- detection status
 * @param dev Pointer to driver handle
 * @param dpdm_stat Pointer to D+/D- detection status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_dpdm_status(bq25798_t *dev, bool *dpdm_stat) {
    if (!dev || !dev->initialized || !dpdm_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_2, &data);
    if (status != BQ25798_OK) return status;

    *dpdm_stat = (bool)((data >> 1) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get battery present status
 * @param dev Pointer to driver handle
 * @param vbat_present_stat Pointer to battery present status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbat_present_status(bq25798_t *dev, bool *vbat_present_stat) {
    if (!dev || !dev->initialized || !vbat_present_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_2, &data);
    if (status != BQ25798_OK) return status;

    *vbat_present_stat = (bool)(data & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get ACFET2-RBFET2 status
 * @param dev Pointer to driver handle
 * @param acrb2_stat Pointer to ACRB2 status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_acrb2_status(bq25798_t *dev, bool *acrb2_stat) {
    if (!dev || !dev->initialized || !acrb2_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_3, &data);
    if (status != BQ25798_OK) return status;

    *acrb2_stat = (bool)((data >> 7) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get ACFET1-RBFET1 status
 * @param dev Pointer to driver handle
 * @param acrb1_stat Pointer to ACRB1 status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_acrb1_status(bq25798_t *dev, bool *acrb1_stat) {
    if (!dev || !dev->initialized || !acrb1_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_3, &data);
    if (status != BQ25798_OK) return status;

    *acrb1_stat = (bool)((data >> 6) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get ADC conversion done status
 * @param dev Pointer to driver handle
 * @param adc_done_stat Pointer to ADC done status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_adc_done_status(bq25798_t *dev, bool *adc_done_stat) {
    if (!dev || !dev->initialized || !adc_done_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_3, &data);
    if (status != BQ25798_OK) return status;

    *adc_done_stat = (bool)((data >> 5) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VSYS regulation status
 * @param dev Pointer to driver handle
 * @param vsys_stat Pointer to VSYS status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsys_status(bq25798_t *dev, bool *vsys_stat) {
    if (!dev || !dev->initialized || !vsys_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_3, &data);
    if (status != BQ25798_OK) return status;

    *vsys_stat = (bool)((data >> 4) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get charge timer status
 * @param dev Pointer to driver handle
 * @param chg_tmr_stat Pointer to charge timer status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_charge_timer_status(bq25798_t *dev, bool *chg_tmr_stat) {
    if (!dev || !dev->initialized || !chg_tmr_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_3, &data);
    if (status != BQ25798_OK) return status;

    *chg_tmr_stat = (bool)((data >> 3) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get trickle charge timer status
 * @param dev Pointer to driver handle
 * @param trichg_tmr_stat Pointer to trickle timer status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_trickle_timer_status(bq25798_t *dev, bool *trichg_tmr_stat) {
    if (!dev || !dev->initialized || !trichg_tmr_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_3, &data);
    if (status != BQ25798_OK) return status;

    *trichg_tmr_stat = (bool)((data >> 2) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get precharge timer status
 * @param dev Pointer to driver handle
 * @param prechg_tmr_stat Pointer to precharge timer status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_precharge_timer_status(bq25798_t *dev, bool *prechg_tmr_stat) {
    if (!dev || !dev->initialized || !prechg_tmr_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_3, &data);
    if (status != BQ25798_OK) return status;

    *prechg_tmr_stat = (bool)((data >> 1) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VBAT too low for OTG status
 * @param dev Pointer to driver handle
 * @param vbat_otg_low_stat Pointer to VBAT OTG low status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbat_otg_low_status(bq25798_t *dev, bool *vbat_otg_low_stat) {
    if (!dev || !dev->initialized || !vbat_otg_low_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_4, &data);
    if (status != BQ25798_OK) return status;

    *vbat_otg_low_stat = (bool)((data >> 4) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get TS cold temperature status
 * @param dev Pointer to driver handle
 * @param ts_cold_stat Pointer to TS cold status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_cold_status(bq25798_t *dev, bool *ts_cold_stat) {
    if (!dev || !dev->initialized || !ts_cold_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_4, &data);
    if (status != BQ25798_OK) return status;

    *ts_cold_stat = (bool)((data >> 3) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get TS cool temperature status
 * @param dev Pointer to driver handle
 * @param ts_cool_stat Pointer to TS cool status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_cool_status(bq25798_t *dev, bool *ts_cool_stat) {
    if (!dev || !dev->initialized || !ts_cool_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_4, &data);
    if (status != BQ25798_OK) return status;

    *ts_cool_stat = (bool)((data >> 2) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get TS warm temperature status
 * @param dev Pointer to driver handle
 * @param ts_warm_stat Pointer to TS warm status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_warm_status(bq25798_t *dev, bool *ts_warm_stat) {
    if (!dev || !dev->initialized || !ts_warm_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_4, &data);
    if (status != BQ25798_OK) return status;

    *ts_warm_stat = (bool)((data >> 1) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get TS hot temperature status
 * @param dev Pointer to driver handle
 * @param ts_hot_stat Pointer to TS hot status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_hot_status(bq25798_t *dev, bool *ts_hot_stat) {
    if (!dev || !dev->initialized || !ts_hot_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_4, &data);
    if (status != BQ25798_OK) return status;

    *ts_hot_stat = (bool)(data & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get IBAT regulation status
 * @param dev Pointer to driver handle
 * @param ibat_reg_stat IBAT regulation status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibat_regulator_status(bq25798_t *dev, bool *ibat_reg_stat) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *ibat_reg_stat = (bool)((data >> 7) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VBUS overvoltage status
 * @param dev Pointer to driver handle
 * @param vbus_ovp_stat Pointer to VBUS OVP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbus_ovp_status(bq25798_t *dev, bool *vbus_ovp_stat) {
    if (!dev || !dev->initialized || !vbus_ovp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *vbus_ovp_stat = (bool)((data >> 6) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VBAT overvoltage status
 * @param dev Pointer to driver handle
 * @param vbat_ovp_stat Pointer to VBAT OVP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbat_ovp_status(bq25798_t *dev, bool *vbat_ovp_stat) {
    if (!dev || !dev->initialized || !vbat_ovp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *vbat_ovp_stat = (bool)((data >> 5) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get IBUS overcurrent status
 * @param dev Pointer to driver handle
 * @param ibus_ocp_stat Pointer to IBUS OCP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibus_ocp_status(bq25798_t *dev, bool *ibus_ocp_stat) {
    if (!dev || !dev->initialized || !ibus_ocp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *ibus_ocp_stat = (bool)((data >> 4) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get IBAT overcurrent status
 * @param dev Pointer to driver handle
 * @param ibat_ocp_stat Pointer to IBAT OCP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibat_ocp_status(bq25798_t *dev, bool *ibat_ocp_stat) {
    if (!dev || !dev->initialized || !ibat_ocp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *ibat_ocp_stat = (bool)((data >> 3) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get converter overcurrent status
 * @param dev Pointer to driver handle
 * @param conv_ocp_stat Pointer to converter OCP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_converter_ocp_status(bq25798_t *dev, bool *conv_ocp_stat) {
    if (!dev || !dev->initialized || !conv_ocp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *conv_ocp_stat = (bool)((data >> 2) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VAC2 overvoltage status
 * @param dev Pointer to driver handle
 * @param vac2_ovp_stat Pointer to VAC2 OVP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac2_ovp_status(bq25798_t *dev, bool *vac2_ovp_stat) {
    if (!dev || !dev->initialized || !vac2_ovp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *vac2_ovp_stat = (bool)((data >> 1) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VAC1 overvoltage status
 * @param dev Pointer to driver handle
 * @param vac1_ovp_stat Pointer to VAC1 OVP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac1_ovp_status(bq25798_t *dev, bool *vac1_ovp_stat) {
    if (!dev || !dev->initialized || !vac1_ovp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_0, &data);
    if (status != BQ25798_OK) return status;

    *vac1_ovp_stat = (bool)(data & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VSYS short circuit status
 * @param dev Pointer to driver handle
 * @param vsys_short_stat Pointer to VSYS short status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsys_short_status(bq25798_t *dev, bool *vsys_short_stat) {
    if (!dev || !dev->initialized || !vsys_short_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_1, &data);
    if (status != BQ25798_OK) return status;

    *vsys_short_stat = (bool)((data >> 7) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VSYS overvoltage status
 * @param dev Pointer to driver handle
 * @param vsys_ovp_stat Pointer to VSYS OVP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsys_ovp_status(bq25798_t *dev, bool *vsys_ovp_stat) {
    if (!dev || !dev->initialized || !vsys_ovp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_1, &data);
    if (status != BQ25798_OK) return status;

    *vsys_ovp_stat = (bool)((data >> 6) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get OTG overvoltage status
 * @param dev Pointer to driver handle
 * @param otg_ovp_stat Pointer to OTG OVP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_otg_ovp_status(bq25798_t *dev, bool *otg_ovp_stat) {
    if (!dev || !dev->initialized || !otg_ovp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_1, &data);
    if (status != BQ25798_OK) return status;

    *otg_ovp_stat = (bool)((data >> 5) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get OTG undervoltage status
 * @param dev Pointer to driver handle
 * @param otg_uvp_stat Pointer to OTG UVP status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_otg_uvp_status(bq25798_t *dev, bool *otg_uvp_stat) {
    if (!dev || !dev->initialized || !otg_uvp_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_1, &data);
    if (status != BQ25798_OK) return status;

    *otg_uvp_stat = (bool)((data >> 4) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get thermal shutdown status
 * @param dev Pointer to driver handle
 * @param tshut_stat Pointer to thermal shutdown status variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_thermal_shutdown_status(bq25798_t *dev, bool *tshut_stat) {
    if (!dev || !dev->initialized || !tshut_stat) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_1, &data);
    if (status != BQ25798_OK) return status;

    *tshut_stat = (bool)((data >> 2) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Check charger statuses
 * @param dev Pointer to device handle
 * @param charger_status Pointer to statuses
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_check_charger_status(bq25798_t *dev, bq25798_charger_status_t *charger_status) {
    if (!dev || !dev->initialized || !charger_status) return BQ25798_ERR_NULL;

    uint8_t statuses[5];

    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_0, &statuses[0]);
    if (status != BQ25798_OK) return status;

    status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_1, &statuses[1]);
    if (status != BQ25798_OK) return status;

    status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_2, &statuses[2]);
    if (status != BQ25798_OK) return status;

    status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_3, &statuses[3]);
    if (status != BQ25798_OK) return status;

    status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_STATUS_4, &statuses[4]);
    if (status != BQ25798_OK) return status;

    charger_status->iindpm_stat = (bool)((statuses[0] >> 7) & 0x01);
    charger_status->vindpm_stat = (bool)((statuses[0] >> 6) & 0x01);
    charger_status->watchdog_stat = (bool)((statuses[0] >> 5) & 0x01);
    charger_status->pg_stat = (bool)((statuses[0] >> 3) & 0x01);
    charger_status->ac2_present_stat = (bool)((statuses[0] >> 2) & 0x01);
    charger_status->ac1_present_stat = (bool)((statuses[0] >> 1) & 0x01);
    charger_status->vbus_present_stat = (bool)(statuses[0] & 0x01);

    charger_status->chg_stat = (bq25798_chg_stat_t)((statuses[1] >> 5) & 0x07);
    charger_status->vbus_stat = (bq25798_vbus_stat_t)((statuses[1] >> 1) & 0x0F);
    charger_status->bc_1_2_done_stat = (bool)(statuses[1] & 0x01);

    charger_status->ico_stat = (bq25798_ico_stat_t)((statuses[2] >> 6) & 0x03);
    charger_status->treg_stat = (bool)((statuses[2] >> 2) & 0x01);
    charger_status->dpdm_stat = (bool)((statuses[2] >> 1) & 0x01);
    charger_status->vbat_present_stat = (bool)(statuses[2] & 0x01);

    charger_status->acrb2_stat = (bool)((statuses[3] >> 7) & 0x07);
    charger_status->acrb1_stat = (bool)((statuses[3] >> 6) & 0x01);
    charger_status->adc_done_stat = (bool)((statuses[3] >> 5) & 0x01);
    charger_status->vsys_stat = (bool)((statuses[3] >> 4) & 0x01);
    charger_status->chg_tmr_stat = (bool)((statuses[3] >> 3) & 0x01);
    charger_status->trichg_tmr_stat = (bool)((statuses[3] >> 2) & 0x01);
    charger_status->prechg_tmr_stat = (bool)((statuses[3] >> 1) & 0x01);

    charger_status->vbat_otg_low_stat = (bool)((statuses[4] >> 4) & 0x01);
    charger_status->ts_cold_stat = (bool)((statuses[4] >> 3) & 0x01);
    charger_status->ts_cool_stat = (bool)((statuses[4] >> 2) & 0x01);
    charger_status->ts_warm_stat = (bool)((statuses[4] >> 1) & 0x01);
    charger_status->ts_hot_stat = (bool)(statuses[4] & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Check fault statuses
 * @param dev Pointer to device handle
 * @param fault_status Pointer to statuses
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_check_fault_status(bq25798_t *dev, bq25798_fault_status_t *fault_status) {
    if (!dev || !dev->initialized || !fault_status) return BQ25798_ERR_NULL;

    uint8_t statuses[2];

    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_0, &statuses[0]);
    if (status != BQ25798_OK) return status;

    status = bq25798_read_register(dev, BQ25798_CMD_FAULT_STATUS_1, &statuses[1]);
    if (status != BQ25798_OK) return status;

    fault_status->ibat_reg_stat = (bool)((statuses[0] >> 7) & 0x01);
    fault_status->vbus_ovp_stat = (bool)((statuses[0] >> 6) & 0x01);
    fault_status->vbat_ovp_stat = (bool)((statuses[0] >> 5) & 0x01);
    fault_status->ibus_ocp_stat = (bool)((statuses[0] >> 4) & 0x01);
    fault_status->ibat_ocp_stat = (bool)((statuses[0] >> 3) & 0x01);
    fault_status->conv_ocp_stat = (bool)((statuses[0] >> 2) & 0x01);
    fault_status->vac2_ovp_stat = (bool)((statuses[0] >> 1) & 0x01);
    fault_status->vac1_ovp_stat = (bool)(statuses[0] & 0x01);

    fault_status->vsys_short_stat = (bool)(statuses[1] >> 7) & 0x01;
    fault_status->vsys_ovp_stat = (bool)(statuses[1] >> 6) & 0x01;
    fault_status->otg_ovp_stat = (bool)(statuses[1] >> 5) & 0x01;
    fault_status->otg_ovp_stat = (bool)(statuses[1] >> 4) & 0x01;
    fault_status->tshut_stat = (bool)(statuses[1] & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get IINDPM flag
 * @param dev Pointer to driver handle
 * @param iindpm_flag Pointer to IINDPM flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_iindpm_flag(bq25798_t *dev, bool *iindpm_flag) {
    if (!dev || !dev->initialized || !iindpm_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *iindpm_flag = (data >> 7) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VINDPM flag
 * @param dev Pointer to driver handle
 * @param vindpm_flag Pointer to VINDPM flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vindpm_flag(bq25798_t *dev, bool *vindpm_flag) {
    if (!dev || !dev->initialized || !vindpm_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *vindpm_flag = (data >> 6) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get watchdog timer flag
 * @param dev Pointer to driver handle
 * @param watchdog_flag Pointer to watchdog flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_watchdog_flag(bq25798_t *dev, bool *watchdog_flag) {
    if (!dev || !dev->initialized || !watchdog_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *watchdog_flag = (data >> 5) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get poor source detection flag
 * @param dev Pointer to driver handle
 * @param poor_src_flag Pointer to poor source flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_poor_src_flag(bq25798_t *dev, bool *poor_src_flag) {
    if (!dev || !dev->initialized || !poor_src_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *poor_src_flag = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get power good flag
 * @param dev Pointer to driver handle
 * @param pg_flag Pointer to power good flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_power_good_flag(bq25798_t *dev, bool *pg_flag) {
    if (!dev || !dev->initialized || !pg_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *pg_flag = (data >> 3) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get AC2 present flag
 * @param dev Pointer to driver handle
 * @param ac2_present_flag Pointer to AC2 present flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ac2_present_flag(bq25798_t *dev, bool *ac2_present_flag) {
    if (!dev || !dev->initialized || !ac2_present_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *ac2_present_flag = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get AC1 present flag
 * @param dev Pointer to driver handle
 * @param ac1_present_flag Pointer to AC1 present flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ac1_present_flag(bq25798_t *dev, bool *ac1_present_flag) {
    if (!dev || !dev->initialized || !ac1_present_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *ac1_present_flag = (data >> 1) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VBUS present flag
 * @param dev Pointer to driver handle
 * @param vbus_present_flag Pointer to VBUS present flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbus_present_flag(bq25798_t *dev, bool *vbus_present_flag) {
    if (!dev || !dev->initialized || !vbus_present_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *vbus_present_flag = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get charge status flag
 * @param dev Pointer to driver handle
 * @param charge_flag Pointer to charge flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_charge_flag(bq25798_t *dev, bool *charge_flag) {
    if (!dev || !dev->initialized || !charge_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *charge_flag = (data >> 7) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get ICO flag
 * @param dev Pointer to driver handle
 * @param ico_flag Pointer to ICO flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ico_flag(bq25798_t *dev, bool *ico_flag) {
    if (!dev || !dev->initialized || !ico_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *ico_flag = (data >> 6) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VBUS status flag
 * @param dev Pointer to driver handle
 * @param vbus_flag Pointer to VBUS flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbus_flag(bq25798_t *dev, bool *vbus_flag) {
    if (!dev || !dev->initialized || !vbus_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *vbus_flag = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get thermal regulation flag
 * @param dev Pointer to driver handle
 * @param treg_flag Pointer to thermal regulation flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_regulator_temp_flag(bq25798_t *dev, bool *treg_flag) {
    if (!dev || !dev->initialized || !treg_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *treg_flag = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get battery present flag
 * @param dev Pointer to driver handle
 * @param vbat_present_flag Pointer to battery present flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbat_present_flag(bq25798_t *dev, bool *vbat_present_flag) {
    if (!dev || !dev->initialized || !vbat_present_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *vbat_present_flag = (data >> 1) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get BC1.2 detection done flag
 * @param dev Pointer to driver handle
 * @param bc_1_2_done_flag Pointer to BC1.2 done flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_bc_1_2_done_flag(bq25798_t *dev, bool *bc_1_2_done_flag) {
    if (!dev || !dev->initialized || !bc_1_2_done_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *bc_1_2_done_flag = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get D+/D- detection done flag
 * @param dev Pointer to driver handle
 * @param dpdm_done_flag Pointer to D+/D- done flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_dpdm_done_flag(bq25798_t *dev, bool *dpdm_done_flag) {
    if (!dev || !dev->initialized || !dpdm_done_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_2, &data);
    if (status != BQ25798_OK) return status;

    *dpdm_done_flag = (data >> 6) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get ADC conversion done flag
 * @param dev Pointer to driver handle
 * @param adc_done_flag Pointer to ADC done flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_adc_done_flag(bq25798_t *dev, bool *adc_done_flag) {
    if (!dev || !dev->initialized || !adc_done_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_2, &data);
    if (status != BQ25798_OK) return status;

    *adc_done_flag = (data >> 5) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VSYS regulation flag
 * @param dev Pointer to driver handle
 * @param vsys_flag Pointer to VSYS flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsys_flag(bq25798_t *dev, bool *vsys_flag) {
    if (!dev || !dev->initialized || !vsys_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_2, &data);
    if (status != BQ25798_OK) return status;

    *vsys_flag = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get charge timer flag
 * @param dev Pointer to driver handle
 * @param chg_tmr_flag Pointer to charge timer flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_charge_timer_flag(bq25798_t *dev, bool *chg_tmr_flag) {
    if (!dev || !dev->initialized || !chg_tmr_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_2, &data);
    if (status != BQ25798_OK) return status;

    *chg_tmr_flag = (data >> 3) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get trickle charge timer flag
 * @param dev Pointer to driver handle
 * @param trichg_tmr_flag Pointer to trickle timer flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_trickle_timer_flag(bq25798_t *dev, bool *trichg_tmr_flag) {
    if (!dev || !dev->initialized || !trichg_tmr_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_2, &data);
    if (status != BQ25798_OK) return status;

    *trichg_tmr_flag = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get precharge timer flag
 * @param dev Pointer to driver handle
 * @param prechg_tmr_flag Pointer to precharge timer flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_precharge_timer_flag(bq25798_t *dev, bool *prechg_tmr_flag) {
    if (!dev || !dev->initialized || !prechg_tmr_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_2, &data);
    if (status != BQ25798_OK) return status;

    *prechg_tmr_flag = (data >> 1) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get top-off timer flag
 * @param dev Pointer to driver handle
 * @param topoff_tmr_flag Pointer to top-off timer flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_topoff_timer_flag(bq25798_t *dev, bool *topoff_tmr_flag) {
    if (!dev || !dev->initialized || !topoff_tmr_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_2, &data);
    if (status != BQ25798_OK) return status;

    *topoff_tmr_flag = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VBAT too low for OTG flag
 * @param dev Pointer to driver handle
 * @param vbat_otg_low_flag Pointer to VBAT OTG low flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbat_otg_low_flag(bq25798_t *dev, bool *vbat_otg_low_flag) {
    if (!dev || !dev->initialized || !vbat_otg_low_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_3, &data);
    if (status != BQ25798_OK) return status;

    *vbat_otg_low_flag = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get TS cold temperature flag
 * @param dev Pointer to driver handle
 * @param ts_cold_flag Pointer to TS cold flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_cold_flag(bq25798_t *dev, bool *ts_cold_flag) {
    if (!dev || !dev->initialized || !ts_cold_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_3, &data);
    if (status != BQ25798_OK) return status;

    *ts_cold_flag = (data >> 3) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get TS cool temperature flag
 * @param dev Pointer to driver handle
 * @param ts_cool_flag Pointer to TS cool flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_cool_flag(bq25798_t *dev, bool *ts_cool_flag) {
    if (!dev || !dev->initialized || !ts_cool_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_3, &data);
    if (status != BQ25798_OK) return status;

    *ts_cool_flag = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get TS warm temperature flag
 * @param dev Pointer to driver handle
 * @param ts_warm_flag Pointer to TS warm flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_warm_flag(bq25798_t *dev, bool *ts_warm_flag) {
    if (!dev || !dev->initialized || !ts_warm_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_3, &data);
    if (status != BQ25798_OK) return status;

    *ts_warm_flag = (data >> 1) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get TS hot temperature flag
 * @param dev Pointer to driver handle
 * @param ts_hot_flag Pointer to TS hot flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_hot_flag(bq25798_t *dev, bool *ts_hot_flag) {
    if (!dev || !dev->initialized || !ts_hot_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_3, &data);
    if (status != BQ25798_OK) return status;

    *ts_hot_flag = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get IBAT regulation flag
 * @param dev Pointer to driver handle
 * @param ibat_reg_flag Pointer to IBAT regulation flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibat_reg_flag(bq25798_t *dev, bool *ibat_reg_flag) {
    if (!dev || !dev->initialized || !ibat_reg_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *ibat_reg_flag = (data >> 7) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VBUS overvoltage flag
 * @param dev Pointer to driver handle
 * @param vbus_ovp_flag Pointer to VBUS OVP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbus_ovp_flag(bq25798_t *dev, bool *vbus_ovp_flag) {
    if (!dev || !dev->initialized || !vbus_ovp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *vbus_ovp_flag = (data >> 6) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VBAT overvoltage flag
 * @param dev Pointer to driver handle
 * @param vbat_ovp_flag Pointer to VBAT OVP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbat_ovp_flag(bq25798_t *dev, bool *vbat_ovp_flag) {
    if (!dev || !dev->initialized || !vbat_ovp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *vbat_ovp_flag = (data >> 5) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get IBUS overcurrent flag
 * @param dev Pointer to driver handle
 * @param ibus_ocp_flag Pointer to IBUS OCP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibus_ocp_flag(bq25798_t *dev, bool *ibus_ocp_flag) {
    if (!dev || !dev->initialized || !ibus_ocp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *ibus_ocp_flag = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get IBAT overcurrent flag
 * @param dev Pointer to driver handle
 * @param ibat_ocp_flag Pointer to IBAT OCP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibat_ocp_flag(bq25798_t *dev, bool *ibat_ocp_flag) {
    if (!dev || !dev->initialized || !ibat_ocp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *ibat_ocp_flag = (data >> 3) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get converter overcurrent flag
 * @param dev Pointer to driver handle
 * @param conv_ocp_flag Pointer to converter OCP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_converter_ocp_flag(bq25798_t *dev, bool *conv_ocp_flag) {
    if (!dev || !dev->initialized || !conv_ocp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *conv_ocp_flag = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VAC2 overvoltage flag
 * @param dev Pointer to driver handle
 * @param vac2_ovp_flag Pointer to VAC2 OVP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac2_ovp_flag(bq25798_t *dev, bool *vac2_ovp_flag) {
    if (!dev || !dev->initialized || !vac2_ovp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *vac2_ovp_flag = (data >> 1) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VAC1 overvoltage flag
 * @param dev Pointer to driver handle
 * @param vac1_ovp_flag Pointer to VAC1 OVP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac1_ovp_flag(bq25798_t *dev, bool *vac1_ovp_flag) {
    if (!dev || !dev->initialized || !vac1_ovp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_0, &data);
    if (status != BQ25798_OK) return status;

    *vac1_ovp_flag = data & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VSYS short circuit flag
 * @param dev Pointer to driver handle
 * @param vsys_short_flag Pointer to VSYS short flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsys_short_flag(bq25798_t *dev, bool *vsys_short_flag) {
    if (!dev || !dev->initialized || !vsys_short_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *vsys_short_flag = (data >> 7) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get VSYS overvoltage flag
 * @param dev Pointer to driver handle
 * @param vsys_ovp_flag Pointer to VSYS OVP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsys_ovp_flag(bq25798_t *dev, bool *vsys_ovp_flag) {
    if (!dev || !dev->initialized || !vsys_ovp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *vsys_ovp_flag = (data >> 6) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get OTG overvoltage flag
 * @param dev Pointer to driver handle
 * @param otg_ovp_flag Pointer to OTG OVP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_otg_ovp_flag(bq25798_t *dev, bool *otg_ovp_flag) {
    if (!dev || !dev->initialized || !otg_ovp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *otg_ovp_flag = (data >> 5) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get OTG undervoltage flag
 * @param dev Pointer to driver handle
 * @param otg_uvp_flag Pointer to OTG UVP flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_otg_uvp_flag(bq25798_t *dev, bool *otg_uvp_flag) {
    if (!dev || !dev->initialized || !otg_uvp_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *otg_uvp_flag = (data >> 4) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Get thermal shutdown flag
 * @param dev Pointer to driver handle
 * @param tshut_flag Pointer to thermal shutdown flag variable
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_thermal_shutdown_flag(bq25798_t *dev, bool *tshut_flag) {
    if (!dev || !dev->initialized || !tshut_flag) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_1, &data);
    if (status != BQ25798_OK) return status;

    *tshut_flag = (data >> 2) & 0x01;

    return BQ25798_OK;
}

/**
 * @brief Check charger flags
 * @param dev Pointer to driver handle
 * @param charger_flags Pointer to flags
 * @return
 */
bq25798_status_t bq25798_check_charger_flags(bq25798_t *dev, uint32_t *charger_flags) {
    if (!dev || !dev->initialized || !charger_flags) return BQ25798_ERR_NULL;

    uint8_t flags[4];

    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_0, &flags[0]);
    if (status != BQ25798_OK) return status;

    status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_1, &flags[1]);
    if (status != BQ25798_OK) return status;

    status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_2, &flags[2]);
    if (status != BQ25798_OK) return status;

    status = bq25798_read_register(dev, BQ25798_CMD_CHARGER_FLAG_3, &flags[3]);
    if (status != BQ25798_OK) return status;

    *charger_flags =
        ((uint32_t)flags[0]) << 18 |
        ((uint32_t)((flags[1] >> 6) & 0x03)) << 16 |
        ((uint32_t)((flags[1] >> 4) & 0x01)) << 15 |
        ((uint32_t)(flags[1] & 0x07)) << 12 |
        ((uint32_t)(flags[2] & 0x7F)) << 5 |
        (uint32_t)(flags[3] & 0x0F);

    return BQ25798_OK;
}

/**
 * @brief Check fault flags
 * @param dev Pointer to driver handle
 * @param fault_flags Pointer to flags
 * @return
 */
bq25798_status_t bq25798_check_fault_flags(bq25798_t *dev, uint16_t *fault_flags) {
    if (!dev || !dev->initialized || !fault_flags) return BQ25798_ERR_NULL;

    uint8_t flags[2];

    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_0, &flags[0]);
    if (status != BQ25798_OK) return status;

    status = bq25798_read_register(dev, BQ25798_CMD_FAULT_FLAG_1, &flags[1]);
    if (status != BQ25798_OK) return status;

    *fault_flags =
        ((uint16_t)flags[0]) << 5 |
        ((uint16_t)((flags[1] >> 4) & 0x0F)) << 1 |
        ((uint16_t)(flags[1] >> 2) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get ADC Enable
 * @param dev Pointer to driver handle
 * @param adc_enable Pointer to ADC enable status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_adc_enable(bq25798_t *dev, bool *adc_enable) {
    if (!dev || !dev->initialized || !adc_enable) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *adc_enable = (bool)((data >> 7) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set ADC Enable
 * @param dev Pointer to driver handle
 * @param adc_enable ADC enable toggle
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_adc_enable(bq25798_t *dev, const bool adc_enable) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), adc_enable, 1, 7);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_CONTROL, data);
}

/**
 * @brief Get ADC conversion rate
 * @param dev Pointer to driver handle
 * @param adc_rate Pointer to ADC conversion rate status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_adc_rate(bq25798_t *dev, bool *adc_rate) {
    if (!dev || !dev->initialized || !adc_rate) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *adc_rate = (bool)((data >> 6) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set ADC conversion rate
 * @param dev Pointer to driver handle
 * @param adc_rate ADC conversion rate
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_adc_rate(bq25798_t *dev, const bool adc_rate) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), adc_rate, 1, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_CONTROL, data);
}

/**
 * @brief Get ADC resolution
 * @param dev Pointer to driver handle
 * @param resolution Pointer to ADC resolution
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_adc_resolution(bq25798_t *dev, bq25798_adc_speed_t *resolution) {
    if (!dev || !dev->initialized || !resolution) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *resolution = (bq25798_adc_speed_t)((data >> 4) & 0x03);

    return BQ25798_OK;
}

/**
 * @brief Get ADC resolution
 * @param dev Pointer to driver handle
 * @param resolution ADC resolution
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_adc_resolution(bq25798_t *dev, const bq25798_adc_speed_t resolution) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), resolution, 2, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_CONTROL, data);
}

/**
 * @brief Get ADC average control
 * @param dev Pointer to driver handle
 * @param avg Pointer to ADC average control
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_adc_averageing_ctrl(bq25798_t *dev, bool *avg) {
    if (!dev || !dev->initialized || !avg) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *avg = (bool)((data >> 3) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set ADC average control
 * @param dev Pointer to driver handle
 * @param avg ADC average control
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_adc_averageing_ctrl(bq25798_t *dev, const bool avg) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), avg, 1, 3);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_CONTROL, data);
}

/**
 * @brief Get ADC average initial value control
 * @param dev Pointer to driver handle
 * @param avg_init Pointer to ADC average initial value control
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_adc_avg_init_val(bq25798_t *dev, bool *avg_init) {
    if (!dev || !dev->initialized || !avg_init) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    *avg_init = (bool)((data >> 2) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set ADC average initial value control
 * @param dev Pointer to driver handle
 * @param avg_init ADC average initial value control
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_adc_avg_init_val(bq25798_t *dev, const bool avg_init) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_CONTROL, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), avg_init, 1, 2);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_CONTROL, data);
}

/**
 * @brief Get IBUS ADC status
 * @param dev Pointer to driver handle
 * @param ibus_dis Pointer to IBUS ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibus_adc_disable(bq25798_t *dev, bool *ibus_dis) {
    if (!dev || !dev->initialized || !ibus_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    *ibus_dis = (bool)((data >> 7) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set IBUS ADC status
 * @param dev Pointer to driver handle
 * @param ibus_dis IBUS ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ibus_adc_disable(bq25798_t *dev, const bool ibus_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), ibus_dis, 1, 7);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, data);
}

/**
 * @brief Get IBAT ADC status
 * @param dev Pointer to driver handle
 * @param ibat_dis Pointer to IBAT ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibat_adc_disable(bq25798_t *dev, bool *ibat_dis) {
    if (!dev || !dev->initialized || !ibat_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    *ibat_dis = (bool)((data >> 6) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set IBAT ADC status
 * @param dev Pointer to driver handle
 * @param ibat_dis IBAT ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ibat_adc_disable(bq25798_t *dev, const bool ibat_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), ibat_dis, 1, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, data);
}

/**
 * @brief Get VBUS ADC status
 * @param dev Pointer to driver handle
 * @param vbus_dis Pointer to VBUS ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbus_adc_disable(bq25798_t *dev, bool *vbus_dis) {
    if (!dev || !dev->initialized || !vbus_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    *vbus_dis = (bool)((data >> 5) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set VBUS ADC status
 * @param dev Pointer to driver handle
 * @param vbus_dis VBUS ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vbus_adc_disable(bq25798_t *dev, const bool vbus_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vbus_dis, 1, 5);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, data);
}

/**
 * @brief Get VBAT ADC status
 * @param dev Pointer to driver handle
 * @param vbat_dis Pointer to VBAT ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbat_adc_disable(bq25798_t *dev, bool *vbat_dis) {
    if (!dev || !dev->initialized || !vbat_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    *vbat_dis = (bool)((data >> 4) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set VBAT ADC status
 * @param dev Pointer to driver handle
 * @param vbat_dis VBAT ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vbat_adc_disable(bq25798_t *dev, const bool vbat_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vbat_dis, 1, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, data);
}

/**
 * @brief Get VSYS ADC status
 * @param dev Pointer to driver handle
 * @param vsys_dis Pointer to VSYS ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsys_adc_disable(bq25798_t *dev, bool *vsys_dis) {
    if (!dev || !dev->initialized || !vsys_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    *vsys_dis = (bool)((data >> 3) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set VSYS ADC status
 * @param dev Pointer to driver handle
 * @param vsys_dis VSYS ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vsys_adc_disable(bq25798_t *dev, const bool vsys_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vsys_dis, 1, 3);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, data);
}

/**
 * @brief Get TS ADC status
 * @param dev Pointer to driver handle
 * @param ts_dis Pointer to TS ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_adc_disable(bq25798_t *dev, bool *ts_dis) {
    if (!dev || !dev->initialized || !ts_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    *ts_dis = (bool)((data >> 2) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set TS ADC status
 * @param dev Pointer to driver handle
 * @param ts_dis TS ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_ts_adc_disable(bq25798_t *dev, const bool ts_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), ts_dis, 1, 2);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, data);
}

/**
 * @brief Get TDIE ADC status
 * @param dev Pointer to driver handle
 * @param tdie_dis Pointer to TDIE ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_tdie_adc_disable(bq25798_t *dev, bool *tdie_dis) {
    if (!dev || !dev->initialized || !tdie_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    *tdie_dis = (bool)((data >> 1) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set TDIE ADC status
 * @param dev Pointer to driver handle
 * @param tdie_dis TDIE ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_tdie_adc_disable(bq25798_t *dev, const bool tdie_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), tdie_dis, 1, 1);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_0, data);
}

/**
 * @brief Get D+ ADC status
 * @param dev Pointer to driver handle
 * @param dp_dis Pointer to D+ ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_dp_adc_disable(bq25798_t *dev, bool *dp_dis) {
    if (!dev || !dev->initialized || !dp_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, &data);
    if (status != BQ25798_OK) return status;

    *dp_dis = (bool)((data >> 7) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set D+ ADC status
 * @param dev Pointer to driver handle
 * @param dp_dis D+ ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_dp_adc_disable(bq25798_t *dev, const bool dp_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), dp_dis, 1, 7);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, data);
}

/**
 * @brief Get D- ADC status
 * @param dev Pointer to driver handle
 * @param dm_dis Pointer to D- ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_dm_adc_disable(bq25798_t *dev, bool *dm_dis) {
    if (!dev || !dev->initialized || !dm_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, &data);
    if (status != BQ25798_OK) return status;

    *dm_dis = (bool)((data >> 6) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set D- ADC status
 * @param dev Pointer to driver handle
 * @param dm_dis D- ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_dm_adc_disable(bq25798_t *dev, const bool dm_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), dm_dis, 1, 6);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, data);
}

/**
 * @brief Get VAC2 ADC status
 * @param dev Pointer to driver handle
 * @param vac2_dis Pointer to VAC2 ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac2_adc_disable(bq25798_t *dev, bool *vac2_dis) {
    if (!dev || !dev->initialized || !vac2_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, &data);
    if (status != BQ25798_OK) return status;

    *vac2_dis = (bool)((data >> 5) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Set VAC2 ADC status
 * @param dev Pointer to driver handle
 * @param vac2_dis VAC2 ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vac2_adc_disable(bq25798_t *dev, const bool vac2_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vac2_dis, 1, 5);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, data);
}

/**
 * @brief Get VAC1 ADC status
 * @param dev Pointer to driver handle
 * @param vac1_dis Pointer to VAC1 ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac1_adc_disable(bq25798_t *dev, bool *vac1_dis) {
    if (!dev || !dev->initialized || !vac1_dis) return BQ25798_ERR_NULL;

    uint8_t data;
    const bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, &data);
    if (status != BQ25798_OK) return status;

    *vac1_dis = (bool)((data >> 4) & 0x01);

    return BQ25798_OK;
}

/**
 * @brief Get VAC1 ADC status
 * @param dev Pointer to driver handle
 * @param vac1_dis Pointer to VAC1 ADC status
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_set_vac1_adc_disable(bq25798_t *dev, const bool vac1_dis) {
    if (!dev || !dev->initialized) return BQ25798_ERR_NULL;

    uint8_t data;
    bq25798_status_t status = bq25798_read_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, &data);
    if (status != BQ25798_OK) return status;

    status = bq25798_register_bits(&data, sizeof(data), vac1_dis, 1, 4);
    if (status != BQ25798_OK) return status;

    return bq25798_write_register(dev, BQ25798_CMD_ADC_FUNCTION_DISABLE_1, data);
}

/**
 * @brief Get the measurement for IBUS
 * @param dev Pointer to driver handle
 * @param ibus Pointer to IBUS measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibus_measurement(bq25798_t *dev, int *ibus) {
    if (!dev || !dev->initialized || !ibus) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_IBUS_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    *ibus = (int16_t)((data[0] << 8) | data[1]);

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for IBAT
 * @param dev Pointer to driver handle
 * @param ibat Pointer to IBAT measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ibat_measurement(bq25798_t *dev, int *ibat) {
    if (!dev || !dev->initialized || !ibat) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_IBUS_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    *ibat = (int16_t)((data[0] << 8) | data[1]);

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for VBUS
 * @param dev Pointer to driver handle
 * @param vbus Pointer to VBUS measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbus_measurement(bq25798_t *dev, int *vbus) {
    if (!dev || !dev->initialized || !vbus) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_IBUS_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    *vbus = (uint16_t)((data[0] << 8) | data[1]);

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for VAC1
 * @param dev Pointer to driver handle
 * @param vac1 Pointer to VAC1 measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac1_measurement(bq25798_t *dev, int *vac1) {
    if (!dev || !dev->initialized || !vac1) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_VAC1_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    *vac1 = (uint16_t)((data[0] << 8) | data[1]);

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for VAC2
 * @param dev Pointer to driver handle
 * @param vac2 Pointer to VAC2 measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vac2_measurement(bq25798_t *dev, int *vac2) {
    if (!dev || !dev->initialized || !vac2) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_VAC2_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    *vac2 = (uint16_t)((data[0] << 8) | data[1]);

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for VBAT
 * @param dev Pointer to driver handle
 * @param vbat Pointer to VBAT measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vbat_measurement(bq25798_t *dev, int *vbat) {
    if (!dev || !dev->initialized || !vbat) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_VBAT_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    *vbat = (uint16_t)((data[0] << 8) | data[1]);

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for VSYS
 * @param dev Pointer to driver handle
 * @param vsys Pointer to VSYS measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_vsys_measurement(bq25798_t *dev, int *vsys) {
    if (!dev || !dev->initialized || !vsys) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_VSYS_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    *vsys = (uint16_t)((data[0] << 8) | data[1]);

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for TS
 * @param dev Pointer to driver handle
 * @param ts Pointer to TS measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_ts_measurement(bq25798_t *dev, float *ts) {
    if (!dev || !dev->initialized || !ts) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_IBUS_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    const uint16_t dataCombined = (uint16_t)((data[0] << 8) | data[1]);

    *ts = (float)dataCombined * 0.0976563f;

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for TDIE
 * @param dev Pointer to driver handle
 * @param tdie Pointer to TDIE measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_tdie_measurement(bq25798_t *dev, float *tdie) {
    if (!dev || !dev->initialized || !tdie) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_IBUS_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    const int16_t dataCombined = (int16_t)((data[0] << 8) | data[1]);

    *tdie = (float)dataCombined * 0.5f;

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for D+
 * @param dev Pointer to driver handle
 * @param dp Pointer to D+ measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_dp_measurement(bq25798_t *dev, int *dp) {
    if (!dev || !dev->initialized || !dp) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_IBUS_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    *dp = (uint16_t)((data[0] << 8) | data[1]);

    return BQ25798_OK;
}

/**
 * @brief Get the measurement for D-
 * @param dev Pointer to driver handle
 * @param dm Pointer to D- measurement
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_dm_measurement(bq25798_t *dev, int *dm) {
    if (!dev || !dev->initialized || !dm) return BQ25798_ERR_NULL;

    uint8_t data[2];
    const bq25798_status_t status = bq25798_read_registers(dev, BQ25798_CMD_IBUS_ADC, data, sizeof(data));
    if (status != BQ25798_OK) return status;

    *dm = (uint16_t)((data[0] << 8) | data[1]);

    return BQ25798_OK;
}

/**
 * @brief Get the all of the ADC measurements
 * @param dev Pointer to driver handle
 * @param measurements Pointer to measurement struct
 * @return BQ25798_OK on success, or an error code.
 */
bq25798_status_t bq25798_get_adc_mesurements(bq25798_t *dev, bq25798_measurements_t *measurements) {
    if (!dev || !dev->initialized || !measurements) return BQ25798_ERR_NULL;

    bq25798_status_t status = bq25798_set_adc_enable(dev, true);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_ibus_measurement(dev, &measurements->ibus);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_ibat_measurement(dev, &measurements->ibat);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_vbus_measurement(dev, &measurements->vbus);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_vac1_measurement(dev, &measurements->vac1);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_vac2_measurement(dev, &measurements->vac2);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_vbat_measurement(dev, &measurements->vbat);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_vsys_measurement(dev, &measurements->vsys);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_ts_measurement(dev, &measurements->ts);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_tdie_measurement(dev, &measurements->tdie);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_dp_measurement(dev, &measurements->ibus);
    if (status != BQ25798_OK) return status;

    status = bq25798_get_dm_measurement(dev, &measurements->ibus);
    if (status != BQ25798_OK) return status;

    return BQ25798_OK;
}