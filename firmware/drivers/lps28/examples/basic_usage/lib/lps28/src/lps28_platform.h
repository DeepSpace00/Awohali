/*!
 * @file lps28_platform.h
 * @brief Platform abstraction layer for LPS28DFW I2C driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-14
 */

#ifndef LPS28_PLATFORM_H
#define LPS28_PLATFORM_H

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

#endif // LPS28_PLATFORM_H