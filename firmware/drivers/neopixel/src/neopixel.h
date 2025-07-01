#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <stdint.h>
#include <stdbool.h>

// Public function prototypes
void neopixel_init(void);
void neopixel_set_rgb(uint8_t red, uint8_t green, uint8_t blue);
void neopixel_show(void);

// Platform-specific functions
void neopixel_gpio_set(bool level);     // Set data pin high or low
void neopixel_delay_ns(uint32_t ns);    // Delay for a number of nanoseconds

#endif // NEOPIXEL_H