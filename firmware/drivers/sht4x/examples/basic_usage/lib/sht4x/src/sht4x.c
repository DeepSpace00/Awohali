/*!
 * @file sht4x.c
 * @brief SHT4x Temperature and Humidity Sensor Driver Implementation
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-12
 */

#include <sht4x.h>

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
 * @brief  Adafruit Unified Sensor interface for the humidity sensor component
 * of SHT4x
 *
 */
static uint16_t sht4x_crc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF; // Initialize the CRC variable
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }

    return crc;
}

sht4x_status_t sht4x_init(sht4x_t *dev, uint8_t address, sht4x_interface_t io) {
    if (!dev || !io.i2c_write || !io.i2c_read || !io.delay_ms) return SHT4X_ERR_NULL;

    dev->i2c_address = address ? address : SHT4X_I2C_ADDR; // Check if the address matches, if not, set it to the default I2C address
    dev->io = io;
    dev->initialized = false;
    dev->serial_number = 0;

    sht4x_status_t status = sht4x_read_serial_number(dev);
    if (status != SHT4X_OK) return status;

    dev->initialized = true;
    
    return SHT4X_OK;
}

sht4x_status_t sht4x_soft_reset(sht4x_t *dev) {
    if (!dev || !dev->initialized) return SHT4X_ERR_NULL;

    uint8_t cmd = SHT4X_CMD_SOFT_RESET;

    return dev->io.i2c_write(dev->i2c_address, &cmd, 1) == 0 ? SHT4X_OK : SHT4X_ERR_I2C;
}

sht4x_status_t sht4x_read_serial_number(sht4x_t *dev) {
    if (!dev) return SHT4X_ERR_NULL;

    uint8_t cmd = SHT4X_CMD_READ_SERIAL;
    if (dev->io.i2c_write(dev->i2c_address, &cmd, 1) != 0) return SHT4X_ERR_I2C;

    uint8_t buffer[6]; // 2 bytes + CRC, 2 bytes + CRC 
    if (dev->io.i2c_read(dev->i2c_address, buffer, 6) != 0) return SHT4X_ERR_I2C;

    // CRC Check
    if (sht4x_crc(&buffer[0], 2) != buffer[2] || sht4x_crc(&buffer[3], 2) != buffer[5]) return SHT4X_ERR_I2C; // Check if the CRC matches

    dev->serial_number = (uint32_t)buffer[0] << 24 | (uint32_t)buffer[1] << 16 | (uint32_t)buffer[3] << 8 | (uint32_t)buffer[4]; // bit-shift the data to form the serial number

    return SHT4X_OK;
}

sht4x_status_t sht4x_pulse_heater(sht4x_t *dev, sht4x_heater_t level) {
    if (!dev || !dev->initialized) return SHT4X_ERR_NULL;

    uint8_t cmd = SHT4X_CMD_HEATER_20MW_100MS;

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

    if (dev->io.i2c_write(dev->i2c_address, &cmd, 1) != 0) return SHT4X_ERR_I2C;

    return SHT4X_OK;
}

sht4x_status_t sht4x_read_measurement(sht4x_t *dev, sht4x_measurement_t *measurement) {
    if (!dev || !measurement) return SHT4X_ERR_NULL;

    uint8_t cmd = SHT4X_CMD_MEASURE_HIGH_PRECISION;
    if (dev->io.i2c_write(dev->i2c_address, &cmd, 1) != 0) return SHT4X_ERR_I2C;
    dev->io.delay_ms(10); // Max measurement duration

    uint8_t buffer[6];
    if (dev->io.i2c_read(dev->i2c_address, buffer, 6) != 0) return SHT4X_ERR_I2C;

    // CRC Check
    if (sht4x_crc(&buffer[0], 2) != buffer[2] || sht4x_crc(&buffer[3], 2) != buffer[5]) return SHT4X_ERR_I2C;

    uint16_t raw_temp = (buffer[0] << 8) | buffer[1];
    uint16_t raw_rh = (buffer[3] << 8) | buffer[4];

    // Convert raw values to temperature and humidity
    measurement->temperature_c = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    measurement->humidity_rh = 100.0f * ((float)raw_rh / 65535.0f);

    return SHT4X_OK;
}

/*sht4x_status_t sht4x_read_serial_number_2(uint32_t *serial_number) {
    if (!serial_number) return SHT4X_ERR_NULL;
    
    uint8_t cmd = SHT4X_CMD_READ_SERIAL;
    if (platform_i2c_write(SHT4X_I2C_ADDR, &cmd, 1) != SHT4X_OK) {
        return SHT4X_ERR_I2C;
    }
    
    if (platform_delay_ms(1) != SHT4X_OK) {
        return SHT4X_ERR_I2C;
    }
    
    uint8_t data[6];
    if (platform_i2c_read(SHT4X_I2C_ADDR, data, 6) != SHT4X_OK) {
        return SHT4X_ERR_I2C;
    }
    
    // Verify CRC
    if (sht4x_crc(&data[0], 2) != data[2] || sht4x_crc(&data[3], 2) != data[5]) {
        return SHT4X_ERR_I2C;
    }
    
    *serial_number = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | 
                     ((uint32_t)data[3] << 8) | data[4];
    
    return SHT4X_OK;
}*/

const char* sht4x_stat_error(sht4x_status_t status) {
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