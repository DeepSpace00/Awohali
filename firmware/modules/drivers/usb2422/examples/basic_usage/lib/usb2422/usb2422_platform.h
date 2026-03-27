/**
 * @file usb2422_platform.h
 * @brief Platform abstraction layer for USB2422 I2C driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-06
 */

#ifndef USB2422_PLATFORM_H
#define USB2422_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

    // Platform-specific I2C functions
    int platform_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len);
    int platform_i2c_read(uint8_t dev_addr, const uint8_t reg, uint8_t *data, uint16_t len);
    void platform_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif // USB2422_PLATFORM_H