/**
 * @file cc2dxx.h
 * @brief CC2Dxx Humidity and Temperature Sensor Driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-04
 *
 * This driver supports the CC2Dxx sensor from Telaire
 */

#ifndef CC2DXX_H
#define CC2DXX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
// extern "C" {
#endif

#define CC2DXX_I2C_DEFAULT_ADDRESS 0x28  ///< Default I2C address

/**
 * @brief CC2Dxx IC driver return status codes.
 */
typedef enum {
 CC2DXX_OK = 0,
 CC2DXX_ERR_I2C = -1,
 CC2DXX_ERR_TIMEOUT = -2,
 CC2DXX_ERR_NULL = -3,
 CC2DXX_ERR_INVALID_ARG = -4,
} cc2dxx_status_t;

typedef enum {
 CC2DXX_STATUS_GOOD_DATA = 0x00,  ///< Valid data: Data that has not been fetched since the last measurement cycle
 CC2DXX_STATUS_STALE_DATA = 0x01, ///< Stale data: Data that has been already been fetched since the last measurement cycle
 CC2DXX_STATUS_CMD_MODE = 0x03    ///< Command mode: The ChipCap 2 is in Command Mode
} cc2dxx_status_bits;

/**
 * @brief Platform interface abstraction for CC2Dxx driver.
 *
 * The user must provide these functions to enable hardware-specific I2C and timing.
 */
typedef struct {
 int (*i2c_write)(uint8_t dev_addr, const uint8_t *data, uint16_t len);
 int (*i2c_read)(uint8_t dev_addr, uint8_t *data, uint16_t len);
 void (*delay_ms)(uint32_t ms);
} cc2dxx_interface_t;

/**
 * @brief CD2Dxx driver instance.
 */
typedef struct {
 uint8_t i2c_address;
 cc2dxx_interface_t io;
 bool initialized;
} cc2dxx_t;

typedef struct {
 cc2dxx_status_bits status;
 float humidity;
 float temperature;
} cc2dxx_measurements_t;

/**
 * @brief Human-readable description of an CD2Dxx status code.
 *
 * @param status The status code to translate.
 * @return A string describing the error.
 */
const char* cd2dxx_stat_error(cc2dxx_status_t status);

cc2dxx_status_t cd2dxx_init(cc2dxx_t *dev, uint8_t address, cc2dxx_interface_t io);

cc2dxx_status_t cc2dxx_get_status(cc2dxx_t *dev, cc2dxx_measurements_t *measurements);

cc2dxx_status_t cc2dxx_get_humidity(cc2dxx_t *dev, cc2dxx_measurements_t *measurements);

cc2dxx_status_t cc2dxx_get_temperature(cc2dxx_t *dev, cc2dxx_measurements_t *measurements);

cc2dxx_status_t cc2dxx_get_measurements(cc2dxx_t *dev, cc2dxx_measurements_t *measurements);

#ifdef __cplusplus
// }
#endif

#endif //CC2DXX_H
