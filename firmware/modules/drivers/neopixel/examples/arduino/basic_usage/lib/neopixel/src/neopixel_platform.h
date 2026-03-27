/**
 * @file neopixel_platform.h
 * @brief Platform abstraction layer for NeoPixel driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-04
 */

#ifndef NEOPIXEL_PLATFORM_H
#define NEOPIXEL_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Platform-specific functions
int platform_gpio_set_high(uint32_t pin);
int platform_gpio_set_low(uint32_t pin);
void platform_delay_us(uint32_t us);
uint32_t platform_get_us_tick(void);
void platform_disable_interrupts(void);
void platform_enable_interrupts(void);

#ifdef __cplusplus
}
#endif

#endif // NEOPIXEL_PLATFORM_H