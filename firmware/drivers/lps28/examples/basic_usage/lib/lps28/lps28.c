/**
 * @file lps28.c
 * @brief LPS28DFW Pressure and Temperature Sensor Driver Implementation
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-15
 */

#include <stdbool.h>
#include "lps28.h"

// Register addresses
#define LPS28_WHO_AM_I_REG      0x0F
#define LPS28_CTRL_REG1         0x10
#define LPS28_CTRL_REG2         0x11
#define LPS28_CTRL_REG3         0x12
#define LPS28_CTRL_REG4         0x13
#define LPS28_FIFO_CTRL         0x14
#define LPS28_FIFO_WTM          0x15
#define LPS28_REF_P_L           0x16
#define LPS28_REF_P_H           0x17
#define LPS28_I3C_IF_CTRL       0x19
#define LPS28_RPDS_L            0x1A
#define LPS28_RPDS_H            0x1B
#define LPS28_INT_SOURCE        0x24
#define LPS28_FIFO_STATUS1      0x25
#define LPS28_FIFO_STATUS2      0x26
#define LPS28_STATUS            0x27
#define LPS28_PRESS_OUT_XL      0x28
#define LPS28_PRESS_OUT_L       0x29
#define LPS28_PRESS_OUT_H       0x2A
#define LPS28_TEMP_OUT_L        0x2B
#define LPS28_TEMP_OUT_H        0x2C

// WHO_AM_I value
#define LPS28_WHO_AM_I_VALUE    0xB4

// CTRL_REG1 bits
#define LPS28_CTRL_REG1_ODR_MASK    0x78
#define LPS28_CTRL_REG1_ODR_SHIFT   3
#define LPS28_CTRL_REG1_AVG_MASK    0x06
#define LPS28_CTRL_REG1_AVG_SHIFT   1

// CTRL_REG2 bits
#define LPS28_CTRL_REG2_BOOT        0x80
#define LPS28_CTRL_REG2_RESET       0x40
#define LPS28_CTRL_REG2_ONESHOT     0x01

// STATUS bits
#define LPS28_STATUS_T_OR           0x20
#define LPS28_STATUS_P_OR           0x10
#define LPS28_STATUS_T_DA           0x02
#define LPS28_STATUS_P_DA           0x01

// Helper function to write a register
static lps28_status_t lps28_write_register(lps28_t *dev, const uint8_t reg, const uint8_t value) {
    if (!dev || !dev->initialized) return LPS28_ERR_NULL;
    
    const uint8_t data[2] = {reg, value};
    return dev->io.i2c_write(dev->i2c_address, data, 2) == 0 ? LPS28_OK : LPS28_ERR_I2C;
}

// Helper function to read a register
static lps28_status_t lps28_read_register(lps28_t *dev, const uint8_t reg, uint8_t *value) {
    if (!dev || !dev->initialized || !value) return LPS28_ERR_NULL;
    
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return LPS28_ERR_I2C;
    
    return dev->io.i2c_read(dev->i2c_address, value, 1) == 0 ? LPS28_OK : LPS28_ERR_I2C;
}

// Helper function to read multiple registers
static lps28_status_t lps28_read_registers(lps28_t *dev, const uint8_t reg, uint8_t *data, const uint8_t len) {
    if (!dev || !dev->initialized || !data) return LPS28_ERR_NULL;
    
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return LPS28_ERR_I2C;
    
    return dev->io.i2c_read(dev->i2c_address, data, len) == 0 ? LPS28_OK : LPS28_ERR_I2C;
}

/**
 * @brief Convert status error into string
 * @param status Status error
 * @return String describing status error
 */
const char* lps28_stat_error(const lps28_status_t status) {
    switch (status) {
    case LPS28_OK:              return "OK";
    case LPS28_ERR_I2C:         return "I2C communication failed";
    case LPS28_ERR_TIMEOUT:     return "Timeout occurred";
    case LPS28_ERR_NULL:        return "Null pointer";
    case LPS28_ERR_INVALID_ARG: return "Invalid argument";
    case LPS28_ERR_WHO_AM_I:    return "WHO_AM_I verification failed";
    default:                    return "Unknown error";
    }
}

/**
 * @brief Initialize the LPS28DFW driver
 * @param dev Pointer to driver handle
 * @param address I2C address (0 to use default)
 * @param io Interface structure with platform-specific functions
 * @return lps28_status_t Error code
 */
lps28_status_t lps28_init(lps28_t *dev, const uint8_t address, const lps28_interface_t io) {
    if (!dev || !io.i2c_write || !io.i2c_read || !io.delay_ms) return LPS28_ERR_NULL;

    dev->i2c_address = address ? address : LPS28_I2C_ADDR;
    dev->io = io;
    dev->initialized = false;

    // Verify WHO_AM_I
    uint8_t who_am_i;
    const lps28_status_t status = lps28_who_am_i(dev, &who_am_i);
    if (status != LPS28_OK) return status;
    
    if (who_am_i != LPS28_WHO_AM_I_VALUE) return LPS28_ERR_WHO_AM_I;

    dev->initialized = true;
    
    return LPS28_OK;
}

/**
 * @brief Read the WHO_AM_I register to verify sensor identity.
 *
 * @param dev Pointer to initialized driver struct.
 * @param who_am_i Output pointer for WHO_AM_I register value.
 * @return LPS28_OK on success, or an error code.
 */
lps28_status_t lps28_who_am_i(lps28_t *dev, uint8_t *who_am_i) {
    if (!dev || !who_am_i) return LPS28_ERR_NULL;
    
    const uint8_t reg = LPS28_WHO_AM_I_REG;
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return LPS28_ERR_I2C;
    
    return dev->io.i2c_read(dev->i2c_address, who_am_i, 1) == 0 ? LPS28_OK : LPS28_ERR_I2C;
}

/**
 * @brief Configure CTRL_REG1 register.
 *
 * @param dev Pointer to initialized driver struct.
 * @param odr Output data rate setting.
 * @param avg Average configuration setting.

 * @return LPS28_OK on success, or an error code.
 */
lps28_status_t lps28_ctrl_reg1(lps28_t *dev, const lps28_odr_t odr, const lps28_avg_t avg) {
    if (!dev || !dev->initialized) return LPS28_ERR_NULL;
    
    if (odr > LPS28_ODR_200_HZ || avg > LPS28_AVG_32) return LPS28_ERR_INVALID_ARG;
    
    const uint8_t reg_value = ((odr << LPS28_CTRL_REG1_ODR_SHIFT) & LPS28_CTRL_REG1_ODR_MASK) | ((avg << LPS28_CTRL_REG1_AVG_SHIFT) & LPS28_CTRL_REG1_AVG_MASK);
    
    return lps28_write_register(dev, LPS28_CTRL_REG1, reg_value);
}

/**
 * @brief Configure CTRL_REG2 register.
 *
 * @param dev Pointer to initialized driver struct.
 * @param boot Enable boot mode.
 * @param reset Enable software reset.
 * @param oneshot Enable one-shot mode.
 * @return LPS28_OK on success, or an error code.
 */
lps28_status_t lps28_ctrl_reg2(lps28_t *dev, const bool boot, const bool reset, const bool oneshot) {
    if (!dev || !dev->initialized) return LPS28_ERR_NULL;
    
    uint8_t reg_value = 0;
    if (boot) reg_value |= LPS28_CTRL_REG2_BOOT;
    if (reset) reg_value |= LPS28_CTRL_REG2_RESET;
    if (oneshot) reg_value |= LPS28_CTRL_REG2_ONESHOT;
    
    return lps28_write_register(dev, LPS28_CTRL_REG2, reg_value);
}

/**
 * @brief Set reference pressure for differential pressure measurement.
 *
 * @param dev Pointer to initialized driver struct.
 * @param ref_pressure Reference pressure in hPa.
 * @return LPS28_OK on success, or an error code.
 */
lps28_status_t lps28_reference_pressure(lps28_t *dev, const float ref_pressure) {
    if (!dev || !dev->initialized) return LPS28_ERR_NULL;
    
    // Convert pressure from hPa to raw 16-bit value
    // LPS28DFW has 24-bit pressure data, but reference is 16-bit
    const int16_t ref_raw = (int16_t)(ref_pressure * 4096.0f / 1260.0f);
    
    const uint8_t ref_l = (uint8_t)(ref_raw & 0xFF);
    const uint8_t ref_h = (uint8_t)((ref_raw >> 8) & 0xFF);
    
    const lps28_status_t status = lps28_write_register(dev, LPS28_REF_P_L, ref_l);
    if (status != LPS28_OK) return status;
    
    return lps28_write_register(dev, LPS28_REF_P_H, ref_h);
}

/**
 * @brief Set pressure offset for calibration.
 *
 * @param dev Pointer to initialized driver struct.
 * @param offset Pressure offset in hPa.
 * @return LPS28_OK on success, or an error code.
 */
lps28_status_t lps28_pressure_offset(lps28_t *dev, const float offset) {
    if (!dev || !dev->initialized) return LPS28_ERR_NULL;
    
    // Convert offset from hPa to raw 16-bit value
    const int16_t offset_raw = (int16_t)(offset * 4096.0f / 1260.0f);
    
    const uint8_t offset_l = (uint8_t)(offset_raw & 0xFF);
    const uint8_t offset_h = (uint8_t)((offset_raw >> 8) & 0xFF);
    
    const lps28_status_t status = lps28_write_register(dev, LPS28_RPDS_L, offset_l);
    if (status != LPS28_OK) return status;
    
    return lps28_write_register(dev, LPS28_RPDS_H, offset_h);
}

/**
 * @brief Read the status register.
 *
 * @param dev Pointer to initialized driver struct.
 * @param status Output pointer for status register data.
 * @return LPS28_OK on success, or an error code.
 */
lps28_status_t lps28_status(lps28_t *dev, lps28_status_reg_t *status) {
    if (!dev || !dev->initialized || !status) return LPS28_ERR_NULL;
    
    uint8_t status_reg;
    lps28_status_t result = lps28_read_register(dev, LPS28_STATUS, &status_reg);
    if (result != LPS28_OK) return result;
    
    status->temperature_overrun = (status_reg & LPS28_STATUS_T_OR) ? true : false;
    status->pressure_overrun = (status_reg & LPS28_STATUS_P_OR) ? true : false;
    status->temperature_data_available = (status_reg & LPS28_STATUS_T_DA) ? true : false;
    status->pressure_data_available = (status_reg & LPS28_STATUS_P_DA) ? true : false;
    
    return LPS28_OK;
}

lps28_status_t lps28_read_pressure(lps28_t *dev, float *pressure) {
    if (!dev || !dev->initialized || !pressure) return LPS28_ERR_NULL;
    
    uint8_t data[3];
    lps28_status_t status = lps28_read_registers(dev, LPS28_PRESS_OUT_XL, data, 3);
    if (status != LPS28_OK) return status;
    
    // Combine 24-bit pressure data (XL, L, H)
    int32_t raw_pressure = (int32_t)data[0] | ((int32_t)data[1] << 8) | ((int32_t)data[2] << 16);
    
    // Sign extend if negative (24-bit to 32-bit)
    if (raw_pressure & 0x800000) {
        raw_pressure |= 0xFF000000;
    }
    
    // Convert to hPa (4096 LSB/hPa)
    *pressure = (float)raw_pressure / 4096.0f;
    
    return LPS28_OK;
}

lps28_status_t lps28_read_temperature(lps28_t *dev, float *temperature) {
    if (!dev || !dev->initialized || !temperature) return LPS28_ERR_NULL;
    
    uint8_t data[2];
    lps28_status_t status = lps28_read_registers(dev, LPS28_TEMP_OUT_L, data, 2);
    if (status != LPS28_OK) return status;
    
    // Combine 16-bit temperature data (L, H)
    int16_t raw_temp = (int16_t)((data[1] << 8) | data[0]);
    
    // Convert to Celsius (100 LSB/°C)
    *temperature = (float)raw_temp / 100.0f;
    
    return LPS28_OK;
}

const char* lps28_stat_error(lps28_status_t status) {
    switch (status) {
        case LPS28_OK:              return "OK";
        case LPS28_ERR_I2C:         return "I2C communication failed";
        case LPS28_ERR_TIMEOUT:     return "Timeout occurred";
        case LPS28_ERR_NULL:        return "Null pointer";
        case LPS28_ERR_INVALID_ARG: return "Invalid argument";
        case LPS28_ERR_WHO_AM_I:    return "WHO_AM_I verification failed";
        default:                    return "Unknown error";
    }
}