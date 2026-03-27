/*!
 * @file neopixel_platform_arduino.cpp
 * @brief Platform abstraction layer implementation for Arduino
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-04
 * 
 * This implementation uses Arduino framework functions
 */

#include <Arduino.h>

extern "C" {
#include "neopixel_platform.h"
}

extern "C" {

static bool interrupts_enabled = true;

int platform_gpio_set_high(uint32_t pin) {
    digitalWrite(pin, HIGH);
    return 0;
}

int platform_gpio_set_low(uint32_t pin) {
    digitalWrite(pin, LOW);
    return 0;
}

void platform_delay_us(uint32_t us) {
    if (us == 0) return;
    
    if (us == 1) {
        // For 1us delays, use delayMicroseconds which is more accurate
        // but for NeoPixel timing, we often need sub-microsecond precision
        // This is a rough approximation - fine-tune based on your board
        #if defined(__AVR__)
            // AVR: Use inline assembly for precise timing
            __asm__ __volatile__ (
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                ::
            );
        #elif defined(ESP32) || defined(ESP8266)
            // ESP: Use built-in delayMicroseconds for short delays
            delayMicroseconds(1);
        #else
            // Generic: Use delayMicroseconds
            delayMicroseconds(1);
        #endif
    } else {
        delayMicroseconds(us);
    }
}

uint32_t platform_get_us_tick(void) {
    return micros();
}

void platform_disable_interrupts(void) {
    interrupts_enabled = true;  // Remember the previous state
    noInterrupts();
}

void platform_enable_interrupts(void) {
    if (interrupts_enabled) {
        interrupts();
    }
}

}