/*!
 * @file neopixel_platform_arduino.cpp
 * @brief Platform implementation for Arduino/STM32 Feather boards
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-05
 *
 * Supports both Arduino AVR and STM32 Feather boards with proper timing
 */

#include <Arduino.h>

extern "C" {
#include "neopixel_platform.h"
#include "neopixel.h"
}

// Detect board type and set up timing
#if defined(STM32_CORE_VERSION) || defined(STM32F405xx) || defined(STM32F4xx)
    #define IS_STM32_FEATHER 1
    // STM32F405 @ 168MHz
    #define CPU_CYCLES_PER_US (SystemCoreClock / 1000000)
#elif defined(__AVR__)
    #define IS_AVR_ARDUINO 1
    // AVR @ 16MHz
    #define CPU_CYCLES_PER_US (F_CPU / 1000000)
#elif defined(ESP32) || defined(ESP8266)
    #define IS_ESP_BOARD 1
    #define CPU_CYCLES_PER_US (F_CPU / 1000000)
#else
    #define IS_GENERIC_ARDUINO 1
    #define CPU_CYCLES_PER_US 16  // Assume 16MHz
#endif

extern "C" {

// Platform interface functions
int platform_gpio_set_high(const uint32_t pin) {
    digitalWrite(pin, HIGH);
    return 0;
}

int platform_gpio_set_low(const uint32_t pin) {
    digitalWrite(pin, LOW);
    return 0;
}

void platform_delay_us(const uint32_t us) {
    delayMicroseconds(us);
}

uint32_t platform_get_us_tick(void) {
    return micros();
}

void platform_disable_interrupts(void) {
    noInterrupts();
}

void platform_enable_interrupts(void) {
    interrupts();
}

// Optimized NeoPixel show function with proper timing for each platform
neopixel_status_t platform_neopixel_show_optimized(neopixel_t *dev) {
    if (!dev || !dev->initialized) {
        return NEOPIXEL_ERR_NOT_INITIALIZED;
    }

    // Wait for latch time from previous transmission
    bool ready;
    neopixel_status_t status = neopixel_can_show(dev, &ready);
    if (status != NEOPIXEL_OK) return status;
    while (!ready) {
        status = neopixel_can_show(dev, &ready);
        if (status != NEOPIXEL_OK) return status;
    }

    uint8_t *ptr = dev->pixels;
    uint16_t numBytes = dev->num_bytes;
    uint8_t pin = dev->pin;

    noInterrupts();

#if defined(IS_STM32_FEATHER)
    // STM32F405 Feather @ 168MHz with DWT timing
    // Enable DWT if not already enabled
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    for (uint16_t i = 0; i < numBytes; i++) {
        uint8_t byte = ptr[i];
        for (uint8_t bit = 0x80; bit; bit >>= 1) {
            uint32_t start = DWT->CYCCNT;

            if (byte & bit) {
                // Send '1': ~850ns high, ~400ns low
                digitalWrite(pin, HIGH);
                while((DWT->CYCCNT - start) < (CPU_CYCLES_PER_US * 850 / 1000)){} // 850ns

                start = DWT->CYCCNT;
                digitalWrite(pin, LOW);
                while((DWT->CYCCNT - start) < (CPU_CYCLES_PER_US * 400 / 1000)){} // 400ns
            } else {
                // Send '0': ~400ns high, ~850ns low
                digitalWrite(pin, HIGH);
                while((DWT->CYCCNT - start) < (CPU_CYCLES_PER_US * 400 / 1000)){} // 400ns

                start = DWT->CYCCNT;
                digitalWrite(pin, LOW);
                while((DWT->CYCCNT - start) < (CPU_CYCLES_PER_US * 850 / 1000)); // 850ns
            }
        }
    }

#elif defined(IS_AVR_ARDUINO)
    // Arduino Uno/Nano @ 16MHz with inline assembly
    volatile uint8_t *port = portOutputRegister(digitalPinToPort(pin));
    uint8_t pinMask = digitalPinToBitMask(pin);
    uint8_t hiVal = *port | pinMask;
    uint8_t loVal = *port & ~pinMask;

    volatile uint16_t i = numBytes;
    volatile uint8_t *p = ptr, b = *p++, bit = 0x80;

    // Hand-tuned assembly for 16MHz AVR
    for(;;) {
        *port = hiVal;
        asm volatile(
            "mov  __tmp_reg__,  %[lo]"    "\n\t"
            "sbrc %[byte],  7"            "\n\t"
            "mov  __tmp_reg__,  %[hi]"    "\n\t"
            "dec  %[bitcount]"            "\n\t"
            "out  %[portaddr], __tmp_reg__" "\n\t"
            "mov  __tmp_reg__,  %[lo]"    "\n\t"
            "breq nextbyte%="             "\n\t"
            "rol  %[byte]"                "\n\t"
            "rjmp .+0"                    "\n\t"
            "nop"                         "\n\t"
            "out  %[portaddr],  %[lo]"    "\n\t"
            "nop"                         "\n\t"
            "rjmp .+0"                    "\n\t"
            "rjmp head%="                 "\n\t"
            "nextbyte%=:"                 "\n\t"
            "ldi  %[bitcount],  8"        "\n\t"
            "ld   %[byte], %a[ptr]+"      "\n\t"
            "out  %[portaddr], %[lo]"     "\n\t"
            "nop"                         "\n\t"
            "sbiw %[count], 1"            "\n\t"
            "brne head%="                 "\n\t"
            : [portaddr] "+e" (port),
              [byte]     "+r" (b),
              [bitcount] "+r" (bit),
              [count]    "+w" (i)
            : [ptr]      "e"  (p),
              [hi]       "r"  (hiVal),
              [lo]       "r"  (loVal)
        );
        break;
        head%=:;
    }

#elif defined(IS_ESP_BOARD)
    // ESP32/ESP8266 with cycle counter
    for (uint16_t i = 0; i < numBytes; i++) {
        uint8_t byte = ptr[i];
        for (uint8_t bit = 0x80; bit; bit >>= 1) {
            uint32_t start = ESP.getCycleCount();

            digitalWrite(pin, HIGH);

            if (byte & bit) {
                // 1 bit: ~800ns high, ~450ns low
                while((ESP.getCycleCount() - start) < (CPU_CYCLES_PER_US * 800 / 1000));
            } else {
                // 0 bit: ~400ns high, ~850ns low
                while((ESP.getCycleCount() - start) < (CPU_CYCLES_PER_US * 400 / 1000));
            }

            digitalWrite(pin, LOW);
            while((ESP.getCycleCount() - start) < (CPU_CYCLES_PER_US * 1250 / 1000)); // 1.25us total
        }
    }

#else
    // Generic Arduino - less precise timing
    for (uint16_t i = 0; i < numBytes; i++) {
        uint8_t byte = ptr[i];
        for (uint8_t bit = 0x80; bit; bit >>= 1) {
            if (byte & bit) {
                // Send '1'
                digitalWrite(pin, HIGH);
                delayMicroseconds(1);  // Approximate
                digitalWrite(pin, LOW);
                delayMicroseconds(1);
            } else {
                // Send '0'
                digitalWrite(pin, HIGH);
                delayMicroseconds(1);
                digitalWrite(pin, LOW);
                delayMicroseconds(1);
            }
        }
    }
#endif

    interrupts();

    // Record end time for latch timing
    dev->end_time = micros();

    return NEOPIXEL_OK;
}

} // extern "C"