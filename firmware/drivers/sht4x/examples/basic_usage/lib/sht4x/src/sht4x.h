/*!
 * @file sht4x.h
 * @brief SHT4x Temperature and Humidity Sensor Driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-14
 * 
 * This driver supports the SHT40, SHT41, SHT45 sensors from Sensirion
 */

#ifndef SHT4X_H
#define SHT4X_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SHT4X_I2C_ADDR 0x44

/**
 * @brief SHT4x sensor driver return status codes.
 */
typedef enum {
    SHT4X_OK = 0,
    SHT4X_ERR_I2C = -1,
    SHT4X_ERR_TIMEOUT = -2,
    SHT4X_ERR_NULL = -3,
    SHT4X_ERR_CRC = -4,
    SHT4X_ERR_INVALID_ARG = -5
} sht4x_status_t;

/**
 * @brief Heater configuration options for the SHT4x sensor.
 */
typedef enum {
    SHT4X_HEATER_20MW_100MS,
    SHT4X_HEATER_20MW_1S,
    SHT4X_HEATER_110MW_100MS,
    SHT4X_HEATER_110MW_1S,
    SHT4X_HEATER_200MW_100MS,
    SHT4X_HEATER_200MW_1S
} sht4x_heater_t;

/**
 * @brief Platform interface abstraction for SHT4x driver.
 *
 * The user must provide these functions to enable hardware-specific I2C and timing.
 */
typedef struct {
    int (*i2c_write)(uint8_t dev_addr, const uint8_t *data, uint16_t len);
    int (*i2c_read)(uint8_t dev_addr, uint8_t *data, uint16_t len);
    void (*delay_ms)(uint32_t ms);
} sht4x_interface_t;

/**
 * @brief SHT4x driver instance.
 */
typedef struct {
    uint8_t i2c_address;
    uint32_t serial_number;
    sht4x_interface_t io;
    bool initialized;
} sht4x_t;

/**
 * @brief Humidity and Temperature measurements from the SHT4x sensors
 */
typedef struct {
    float temperature_c;
    float humidity_rh;
} sht4x_measurements_t;


// Public function prototypes

const char* sht4x_stat_error(sht4x_status_t status);

sht4x_status_t sht4x_init(sht4x_t *dev, uint8_t address, sht4x_interface_t io);

sht4x_status_t sht4x_soft_reset(sht4x_t *dev);

sht4x_status_t sht4x_read_serial_number(sht4x_t *dev);

sht4x_status_t sht4x_pulse_heater(sht4x_t *dev, sht4x_heater_t level);

sht4x_status_t sht4x_read_measurements(sht4x_t *dev, sht4x_measurements_t *measurements);

#ifdef __cplusplus
}
#endif

#endif // SHT4X_H