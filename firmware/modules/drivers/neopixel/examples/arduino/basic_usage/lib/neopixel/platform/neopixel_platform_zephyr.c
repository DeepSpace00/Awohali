/*!
 * @file neopixel_platform_zephyr.c
 * @brief Platform abstraction layer implementation for Zephyr RTOS
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-04
 * 
 * This implementation uses Zephyr RTOS GPIO and timing APIs
 */

#include "neopixel_platform.h"

#ifdef CONFIG_ZEPHYR_KERNEL  // Zephyr RTOS detected

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>

// GPIO device - adjust based on your board's device tree
#define GPIO_NODE DT_NODELABEL(gpio0)
static const struct device *gpio_dev = DEVICE_DT_GET(GPIO_NODE);

// Global variables for interrupt state
static unsigned int interrupt_key = 0;

int platform_gpio_set_high(uint32_t pin) {
    if (!device_is_ready(gpio_dev)) {
        return -1;
    }
    
    return gpio_pin_set(gpio_dev, pin, 1);
}

int platform_gpio_set_low(uint32_t pin) {
    if (!device_is_ready(gpio_dev)) {
        return -1;
    }
    
    return gpio_pin_set(gpio_dev, pin, 0);
}

void platform_delay_us(uint32_t us) {
    if (us == 0) return;
    
    // For very short delays, use busy wait
    if (us < 10) {
        // Busy wait for sub-10us delays
        uint32_t start = k_cycle_get_32();
        uint32_t cycles = us * sys_clock_hw_cycles_per_sec() / 1000000;
        while ((k_cycle_get_32() - start) < cycles) {
            // Busy wait
        }
    } else {
        // Use k_usleep for longer delays
        k_usleep(us);
    }
}

uint32_t platform_get_us_tick(void) {
    return k_cycle_get_32() / (sys_clock_hw_cycles_per_sec() / 1000000);
}

void platform_disable_interrupts(void) {
    interrupt_key = irq_lock();
}

void platform_enable_interrupts(void) {
    irq_unlock(interrupt_key);
}

// Initialize GPIO pin for NeoPixel output
int platform_neopixel_gpio_init(uint32_t pin) {
    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return -1;
    }
    
    int ret = gpio_pin_configure(gpio_dev, pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("Failed to configure GPIO pin %d: %d\n", pin, ret);
        return ret;
    }
    
    return 0;
}

// Optimized NeoPixel bit transmission for Zephyr
// This function provides better timing control than the generic implementation
void platform_neopixel_send_bit_optimized(uint32_t pin, uint8_t bit, bool is_800khz) {
    if (is_800khz) {
        if (bit) {
            // Send '1': ~850ns high, ~400ns low
            gpio_pin_set_raw(gpio_dev, pin, 1);
            // Busy wait for precise timing - adjust based on your MCU speed
            for (volatile int i = 0; i < 100; i++) __asm__("nop");
            gpio_pin_set_raw(gpio_dev, pin, 0);
            for (volatile int i = 0; i < 50; i++) __asm__("nop");
        } else {
            // Send '0': ~400ns high, ~850ns low
            gpio_pin_set_raw(gpio_dev, pin, 1);
            for (volatile int i = 0; i < 50; i++) __asm__("nop");
            gpio_pin_set_raw(gpio_dev, pin, 0);
            for (volatile int i = 0; i < 100; i++) __asm__("nop");
        }
    } else {
        // 400 KHz timing
        if (bit) {
            // Send '1': ~1.2us high, ~1.3us low
            gpio_pin_set_raw(gpio_dev, pin, 1);
            platform_delay_us(1);
            gpio_pin_set_raw(gpio_dev, pin, 0);
            platform_delay_us(1);
        } else {
            // Send '0': ~0.5us high, ~2.0us low
            gpio_pin_set_raw(gpio_dev, pin, 1);
            platform_delay_us(1);
            gpio_pin_set_raw(gpio_dev, pin, 0);
            platform_delay_us(2);
        }
    }
}

#endif  // CONFIG_ZEPHYR_KERNEL