/**
* @file cc2dxx.c
 * @brief CD2Dxx Humidity and Temperature Sensor Driver Implementation
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-04
 */

#include <stddef.h>
#include "cc2dxx.h"

/**
 * @brief Helper function to write to a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Value to write to a register
 * @return
 */
static cc2dxx_status_t cc2dxx_write_register(const cc2dxx_t *dev, const uint8_t reg, const uint8_t value) {
    const uint8_t data[2] = {reg, value};
    return dev->io.i2c_write(dev->i2c_address, data, 2) == 0 ? CC2DXX_OK : CC2DXX_ERR_I2C;
}

/**
 * @brief Helper function to read data from a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Pointer to data variable
 * @return
 */
static cc2dxx_status_t cc2dxx_read_register(const cc2dxx_t *dev, const uint8_t reg, uint8_t *value) {
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return CC2DXX_ERR_I2C;

    return dev->io.i2c_read(dev->i2c_address, value, 1) == 0 ? CC2DXX_OK : CC2DXX_ERR_I2C;
}

/**
 * @brief Helper function to write multiple bytes to a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param data Pointer to data array
 * @param length Length of data array
 * @return
 */
static cc2dxx_status_t cc2dxx_write_registers(const cc2dxx_t *dev, const uint8_t reg, const uint8_t *data, const size_t length) {
    uint8_t buffer[length + 1];
    buffer[0] = reg;
    for (size_t i = 0; i < length; i++) {
        buffer[i + 1] = data[i];
    }

    return dev->io.i2c_write(dev->i2c_address, buffer, length + 1) == 0 ? CC2DXX_OK : CC2DXX_ERR_I2C;
}

/**
 * @brief Helper function to read multiple bytes from a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param data Pointer to data array
 * @param length Length of data array
 * @return
 */
static cc2dxx_status_t cc2dxx_read_registers(const cc2dxx_t *dev, const uint8_t reg, uint8_t *data, const size_t length) {
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return CC2DXX_ERR_I2C;

    return dev->io.i2c_read(dev->i2c_address, data, length) == 0 ? CC2DXX_OK : CC2DXX_ERR_I2C;
}

/**
 * @brief Helper function to insert bits into a register
 * @param reg Pointer to the register array
 * @param reg_len Length of the register array
 * @param bits Bits to insert into the register
 * @param bits_len Length of the bits to insert into the register
 * @param shift Number of bits to shift the data into the register
 * @return cc2dxx_status_t Status code
 */
static cc2dxx_status_t cc2dxx_register_bits(uint8_t *reg, const size_t reg_len, uint32_t bits, const uint32_t bits_len, const uint32_t shift) {
    if (!reg || !reg_len || !bits_len) return CC2DXX_ERR_NULL;
    if (shift + bits_len > reg_len * 8) return CC2DXX_ERR_INVALID_ARG;

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

    return CC2DXX_OK;
}

/**
 * @brief Convert status error messages to human-readable strings
 * @param status Status to be converted to a string
 * @return String from status
 */
const char* cc2dxx_stat_error(const cc2dxx_t status) {
    switch (status) {
        case CC2DXX_OK:                return "OK";
        case CC2DXX_ERR_I2C:           return "I2C communication failed";
        case CC2DXX_ERR_TIMEOUT:       return "Timeout occurred";
        case CC2DXX_ERR_NULL:          return "Null pointer";
        case CC2DXX_ERR_INVALID_ARG:   return "Invalid argument";
        default:                        return "Unknown error";
    }
}

/**
 * @brief Initialize the CC2Dxx driver
 * @param dev Pointer to the driver handle
 * @param address I2C address (0 to use default)
 * @param io Interface structure with platform-specific functions
 * @return cc2dxx_status_t Status code
 */
cc2dxx_status_t cc2dxx_init(cc2dxx_t *dev, const uint8_t address, const cc2dxx_interface_t io) {
    if (!dev || !io.i2c_write || !io.i2c_read || !io.delay_ms) return CC2DXX_ERR_NULL;

    dev->i2c_address = address ? address : CC2DXX_I2C_DEFAULT_ADDRESS;
    dev->io = io;
    dev->initialized = false;

    if (!dev->io.i2c_write(dev->i2c_address, 0x00, 1) != 0) return CC2DXX_ERR_I2C;

    dev->initialized = true;

    return CC2DXX_OK;
}

/**
 * @brief Get the status of the measurements
 * @param dev Pointer to the driver handle
 * @param measurements Pointer to the measurements struct
 * @return cc2dxx_status_t Status code
 */
cc2dxx_status_t cc2dxx_get_status(cc2dxx_t *dev, cc2dxx_measurements_t *measurements) {
    if (!dev || !measurements) return CC2DXX_ERR_NULL;

    uint8_t data;

    const cc2dxx_status_t status = cc2dxx_read_register(dev, 0x00, &data);
    if (status != CC2DXX_OK) return status;

    measurements->status = (cc2dxx_status_bits)(data >> 6);

    return CC2DXX_OK;
}

/**
 * @brief Get the humidity
 * @param dev Pointer to the driver handle
 * @param measurements Pointer to the measurements struct
 * @return cc2dxx_status_t Status code
 */
cc2dxx_status_t cc2dxx_get_humidity(cc2dxx_t *dev, cc2dxx_measurements_t *measurements) {
    if (!dev || !measurements) return CC2DXX_ERR_NULL;

    uint8_t data[2];

    const cc2dxx_status_t status = cc2dxx_read_registers(dev, 0x00, data, sizeof(data));
    if (status != CC2DXX_OK) return status;

    const uint16_t dataCombined = (uint16_t)(((data[0] & 0x3F) << 8) | data[1]);

    measurements->humidity = (float)((dataCombined * 100.0) / 16384.0);

    return CC2DXX_OK;
}

/**
 * @brief Get the temperature
 * @param dev Pointer to the driver handle
 * @param measurements Pointer to the measurements struct
 * @return cc2dxx_status_t Status code
 */
cc2dxx_status_t cc2dxx_get_temperature(cc2dxx_t *dev, cc2dxx_measurements_t *measurements) {
    if (!dev || !measurements) return CC2DXX_ERR_NULL;

    uint8_t data[4];

    const cc2dxx_status_t status = cc2dxx_read_registers(dev, 0x00, data, sizeof(data));
    if (status != CC2DXX_OK) return status;

    const uint16_t temperatureCombined = (uint16_t)((data[2] << 6) | (data[3] >> 2));

    measurements->temperature = (float)((temperatureCombined / 16384.0) * 165.0 - 40.0);

    return CC2DXX_OK;
}

/**
 * @brief Get measurements
 * @param dev Pointer to the driver handle
 * @param measurements Pointer to the measurements struct
 * @return cc2dxx_status_t Status code
 */
cc2dxx_status_t cc2dxx_get_measurements(cc2dxx_t *dev, cc2dxx_measurements_t *measurements) {
    if (!dev || !measurements) return CC2DXX_ERR_NULL;

    uint8_t data[4];

    const cc2dxx_status_t status = cc2dxx_read_registers(dev, 0x00, data, sizeof(data));
    if (status != CC2DXX_OK) return status;

    measurements->status = (cc2dxx_status_bits)(data[0] >> 6);

    const uint16_t humidityCombined = (uint16_t)(((data[0] & 0x3F) << 8) | data[1]);
    const uint16_t temperatureCombined = (uint16_t)((data[2] << 6) | (data[3] >> 2));

    measurements->humidity = (float)((humidityCombined * 100.0) / 16384.0);
    measurements->temperature = (float)((temperatureCombined / 16384.0) * 165.0 - 40.0);

    return CC2DXX_OK;
}