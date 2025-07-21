/*!
 * @file bq25798_platform.h
 * @brief Platform abstraction layer for BQ25798 I2C driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-18
 */

#ifndef BQ25798_PLATFORM_H
#define BQ25798_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Platform-specific I2C functions
int platform_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len);
int platform_i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t len);
void platform_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif // BQ25798_PLATFORM_H