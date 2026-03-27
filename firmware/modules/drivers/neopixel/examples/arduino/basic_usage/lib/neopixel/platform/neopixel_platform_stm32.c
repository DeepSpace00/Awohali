/*!
 * @file neopixel_platform_stm32.c
 * @brief Platform abstraction layer implementation for STM32 (STM32CubeMX HAL)
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-04
 * 
 * This implementation uses STM32 HAL and DWT for precise timing.
 * Requires DWT (Data Watchpoint and Trace) unit for microsecond timing.
 */

#include "neopixel_platform.h"

#ifdef STM32_HAL_H  // STM32 HAL detected

#include "main.h"  // Include your main.h for GPIO definitions

// DWT (Data Watchpoint and Trace) registers for precise timing
#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DEM_CR      (*(volatile uint32_t*)0xE000EDFC)
#define DEM_CR_TRCENA   (1 << 24)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

// Global variables for interrupt state
static uint32_t interrupt_state = 0;

// Initialize DWT counter for microsecond timing
static void dwt_init(void) {
    static bool dwt_initialized = false;
    if (!dwt_initialized) {
        DEM_CR |= DEM_CR_TRCENA;
        DWT_CYCCNT = 0;
        DWT_CTRL |= DWT_CTRL_CYCCNTENA;
        dwt_initialized = true;
    }
}

// Convert pin number to GPIO port and pin
// This is a simplified mapping - adjust based on your pin configuration
static GPIO_TypeDef* pin_to_port(uint32_t pin) {
    // Example mapping - customize based on your board
    if (pin >= 0 && pin <= 15) return GPIOA;
    else if (pin >= 16 && pin <= 31) return GPIOB;
    else if (pin >= 32 && pin <= 47) return GPIOC;
    else if (pin >= 48 && pin <= 63) return GPIOD;
    else if (pin >= 64 && pin <= 79) return GPIOE;
    else if (pin >= 80 && pin <= 95) return GPIOF;
    else if (pin >= 96 && pin <= 111) return GPIOG;
    else if (pin >= 112 && pin <= 127) return GPIOH;
    return GPIOA; // fallback
}

static uint16_t pin_to_pin_mask(uint32_t pin) {
    return 1 << (pin & 0x0F);
}

// High-precision NeoPixel bit transmission using inline assembly
// This is optimized for STM32F4 at 168MHz - adjust timing for other MCUs
static inline void send_neopixel_bit_800khz(GPIO_TypeDef* port, uint16_t pin_mask, uint8_t bit) {
    if (bit) {
        // Send '1': ~850ns high, ~400ns low (total ~1.25us)
        port->BSRR = pin_mask;  // Set pin high
        // Delay for ~850ns (adjust NOPs based on your MCU frequency)
        __asm volatile (
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
        );
        port->BSRR = (uint32_t)pin_mask << 16;  // Set pin low
        // Delay for ~400ns
        __asm volatile (
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; "
        );
    } else {
        // Send '0': ~400ns high, ~850ns low (total ~1.25us)
        port->BSRR = pin_mask;  // Set pin high
        // Delay for ~400ns
        __asm volatile (
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; "
        );
        port->BSRR = (uint32_t)pin_mask << 16;  // Set pin low
        // Delay for ~850ns
        __asm volatile (
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
            "nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; "
        );
    }
}

int platform_gpio_set_high(uint32_t pin) {
    GPIO_TypeDef* port = pin_to_port(pin);
    uint16_t pin_mask = pin_to_pin_mask(pin);
    
    port->BSRR = pin_mask;
    return 0;
}

int platform_gpio_set_low(uint32_t pin) {
    GPIO_TypeDef* port = pin_to_port(pin);
    uint16_t pin_mask = pin_to_pin_mask(pin);
    
    port->BSRR = (uint32_t)pin_mask << 16;
    return 0;
}

void platform_delay_us(uint32_t us) {
    dwt_init();
    
    uint32_t start_tick = DWT_CYCCNT;
    uint32_t delay_ticks = us * (SystemCoreClock / 1000000);
    
    while ((DWT_CYCCNT - start_tick) < delay_ticks) {
        // Wait
    }
}

uint32_t platform_get_us_tick(void) {
    dwt_init();
    return DWT_CYCCNT / (SystemCoreClock / 1000000);
}

void platform_disable_interrupts(void) {
    interrupt_state = __get_PRIMASK();
    __disable_irq();
}

void platform_enable_interrupts(void) {
    if (!interrupt_state) {
        __enable_irq();
    }
}

// Optimized show function for STM32 - bypasses the generic bit-by-bit approach
// for better timing precision. This should be called directly from neopixel_show()
// when STM32 platform is detected.
void platform_neopixel_show_optimized(uint32_t pin, uint8_t* pixels, uint16_t num_bytes, bool is_800khz) {
    GPIO_TypeDef* port = pin_to_port(pin);
    uint16_t pin_mask = pin_to_pin_mask(pin);
    
    platform_disable_interrupts();
    
    if (is_800khz) {
        for (uint16_t i = 0; i < num_bytes; i++) {
            uint8_t byte = pixels[i];
            for (uint8_t bit = 0x80; bit; bit >>= 1) {
                send_neopixel_bit_800khz(port, pin_mask, byte & bit);
            }
        }
    } else {
        // 400KHz implementation - similar but with different timing
        for (uint16_t i = 0; i < num_bytes; i++) {
            uint8_t byte = pixels[i];
            for (uint8_t bit = 0x80; bit; bit >>= 1) {
                if (byte & bit) {
                    // Send '1': ~1.2us high, ~1.3us low
                    port->BSRR = pin_mask;
                    platform_delay_us(1);  // Approximate - fine-tune as needed
                    port->BSRR = (uint32_t)pin_mask << 16;
                    platform_delay_us(1);
                } else {
                    // Send '0': ~0.5us high, ~2.0us low
                    port->BSRR = pin_mask;
                    platform_delay_us(1);
                    port->BSRR = (uint32_t)pin_mask << 16;
                    platform_delay_us(2);
                }
            }
        }
    }
    
    platform_enable_interrupts();
}

#endif  // STM32_HAL_H