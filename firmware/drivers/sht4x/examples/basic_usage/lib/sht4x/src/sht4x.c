/*!
 * @file sht4x.c
 * @brief SHT4x Temperature and Humidity Sensor Driver Implementation
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-12
 */

#include <stdbool.h>
#include <stddef.h>
#include "sht4x.h"

// Registers
#define SHT4X_CMD_MEASURE_HIGH_PRECISION    0xFD
#define SHT4X_CMD_HEATER_200MW_1S           0x39
#define SHT4X_CMD_HEATER_200MW_100MS        0x32
#define SHT4X_CMD_HEATER_110MW_1S           0x2F
#define SHT4X_CMD_HEATER_110MW_100MS        0x24
#define SHT4X_CMD_HEATER_20MW_1S            0x1E
#define SHT4X_CMD_HEATER_20MW_100MS         0x15

#define SHT4X_CMD_READ_SERIAL               0x89
#define SHT4X_CMD_SOFT_RESET                0x94


// Implementation of functions

/**
 * @brief Helper function to write to a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Value to write to a register
 * @return
 */
static sht4x_status_t sht4x_write_register(const sht4x_t *dev, const uint8_t reg, const uint8_t value) {
    const uint8_t data[2] = {reg, value};
    return dev->io.i2c_write(dev->i2c_address, data, 2) == 0 ? SHT4X_OK : SHT4X_ERR_I2C;
}

/**
 * @brief Helper function to read data from a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Pointer to data variable
 * @return
 */
static sht4x_status_t sht4x_read_register(const sht4x_t *dev, const uint8_t reg, uint8_t *value) {
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return SHT4X_ERR_I2C;

    return dev->io.i2c_read(dev->i2c_address, value, 1) == 0 ? SHT4X_OK : SHT4X_ERR_I2C;
}

/**
 * @brief Helper function to read multiple bytes from a register
 * @param dev Pointer to driver handle
 * @param cmd Target register
 * @param data Pointer to data array
 * @param length Length of data array
 * @param delay_ms Measurement delay (if needed)
 * @return
 */
static sht4x_status_t sht4x_command_read(const sht4x_t *dev, const uint8_t cmd, uint8_t *data, const size_t length, const uint32_t delay_ms) {
    // Send command
    const sht4x_status_t status = sht4x_send_command(dev, cmd);
    if (status != SHT4X_OK) return status;

    // Wait for processing if delay specified
    if (delay_ms > 0) {
        dev->io.delay_ms(delay_ms);
    }

    // Read response
    return dev->io.i2c_read(dev->i2c_address, data, length) == 0 ? SHT4X_OK : SHT4X_ERR_I2C;
}

/**
 * @brief CRC check
 * @param data Bytes
 * @param len Number of bytes
 * @return crc
 */
static uint16_t sht4x_crc(const uint8_t *data, const uint8_t len) {
    uint8_t crc = 0xFF; // Initialize the CRC variable
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Human-readable description of an SHT4x status code.
 *
 * @param status The status code to translate.
 * @return A string describing the error.
 */
const char* sht4x_stat_error(const sht4x_status_t status) {
    switch (status) {
    case SHT4X_OK:              return "OK";
    case SHT4X_ERR_I2C:         return "I2C communication failed";
    case SHT4X_ERR_TIMEOUT:     return "Timeout occurred";
    case SHT4X_ERR_NULL:        return "Null pointer";
    case SHT4X_ERR_CRC:         return "CRC check failed";
    case SHT4X_ERR_INVALID_ARG: return "Invalid argument";
    default:                    return "Unknown error";
    }
}

/**
 * @brief Initialize the SHT4x driver
 * @param dev Pointer to driver handle
 * @param address I2C address (0 to use default)
 * @param io Interface structure with platform-specific functions
 * @return sht4x_status_t Error code
 */
sht4x_status_t sht4x_init(sht4x_t *dev, const uint8_t address, const sht4x_interface_t io) {
    if (!dev || !io.i2c_write || !io.i2c_read || !io.delay_ms) return SHT4X_ERR_NULL;

    dev->i2c_address = address ? address : SHT4X_I2C_ADDR; // Check if the address matches, if not, set it to the default I2C address
    dev->io = io;
    dev->initialized = false;
    dev->serial_number = 0;

    const sht4x_status_t status = sht4x_read_serial_number(dev);
    if (status != SHT4X_OK) return status;

    dev->initialized = true;
    
    return SHT4X_OK;
}

/**
 * @brief Perform a soft reset of the sensor.
 *
 * @param dev Pointer to initialized driver struct.
 * @return SHT4X_OK on success, or an error code.
 */
sht4x_status_t sht4x_soft_reset(sht4x_t *dev) {
    if (!dev || !dev->initialized) return SHT4X_ERR_NULL;
    return sht4x_send_command(dev, SHT4X_CMD_SOFT_RESET);
}

/**
 * @brief Read and store the serial number of the sensor.
 *
 * @param dev Pointer to initialized driver struct.
 * @return SHT4X_OK on success, or an error code.
 */
sht4x_status_t sht4x_read_serial_number(sht4x_t *dev) {
    if (!dev) return SHT4X_ERR_NULL;

    uint8_t data[6]; // 2 bytes + CRC, 2 bytes + CRC
    const sht4x_status_t status = sht4x_command_read(dev, SHT4X_CMD_READ_SERIAL, data, sizeof(data), 1);
    if (status != SHT4X_OK) return status;

    // CRC Check
    if (sht4x_crc(&data[0], 2) != data[2] || sht4x_crc(&data[3], 2) != data[5]) return SHT4X_ERR_CRC;

    dev->serial_number = (uint32_t)data[0] << 24 | (uint32_t)data[1] << 16 |
                        (uint32_t)data[3] << 8 | (uint32_t)data[4]; // bit-shift the data to form the serial number

    return SHT4X_OK;
}

/**
 * @brief Pulse the heater on the sensor.
 *
 * @param dev Pointer to initialized driver struct.
 * @param level The desired heater setting.
 * @return SHT4X_OK on success, or an error code.
 */
sht4x_status_t sht4x_pulse_heater(sht4x_t *dev, const sht4x_heater_t level) {
    if (!dev || !dev->initialized) return SHT4X_ERR_NULL;

    uint8_t cmd;
    switch (level) {
        case SHT4X_HEATER_20MW_100MS:   cmd = SHT4X_CMD_HEATER_20MW_100MS; break;
        case SHT4X_HEATER_20MW_1S:      cmd = SHT4X_CMD_HEATER_20MW_1S; break;
        case SHT4X_HEATER_110MW_100MS:  cmd = SHT4X_CMD_HEATER_110MW_100MS; break;
        case SHT4X_HEATER_110MW_1S:     cmd = SHT4X_CMD_HEATER_110MW_1S; break;
        case SHT4X_HEATER_200MW_100MS:  cmd = SHT4X_CMD_HEATER_200MW_100MS; break;
        case SHT4X_HEATER_200MW_1S:     cmd = SHT4X_CMD_HEATER_200MW_1S; break;
    default:
        return SHT4X_ERR_INVALID_ARG;
    }

    return sht4x_send_command(dev, cmd);
}

/**
 * @brief Read temperature and humidity from the sensor.
 *
 * @param dev Pointer to initialized driver struct.
 * @param measurements Output pointer for temperature in degrees Celsius & relative humidity in %RH.
 * @return SHT4X_OK on success, or an error code.
 */
sht4x_status_t sht4x_read_measurements(sht4x_t *dev, sht4x_measurements_t *measurements) {
    if (!dev || !measurements) return SHT4X_ERR_NULL;

    uint8_t data[6];
    const sht4x_status_t status = sht4x_command_read(dev, SHT4X_CMD_MEASURE_HIGH_PRECISION, data, sizeof(data), 10);
    if (status != SHT4X_OK) return status;

    // CRC Check
    if (sht4x_crc(&data[0], 2) != data[2] || sht4x_crc(&data[3], 2) != data[5]) return SHT4X_ERR_CRC;

    const uint16_t raw_temp = (data[0] << 8) | data[1];
    const uint16_t raw_rh = (data[3] << 8) | data[4];

    // Convert raw values to temperature and humidity
    measurements->temperature_c = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    measurements->humidity_rh = 100.0f * ((float)raw_rh / 65535.0f);

    return SHT4X_OK;
}