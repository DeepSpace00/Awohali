/*!
 * @file example_zephyr.c
 * @brief Zephyr RTOS example for NeoPixel driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-04
 * 
 * This example demonstrates NeoPixel usage with Zephyr RTOS
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "neopixel.h"
#include "neopixel_platform.h"

#define LED_COUNT   60     // Number of NeoPixels in strip
#define BRIGHTNESS  128    // Set brightness (0-255)
#define NEOPIXEL_PIN 2     // GPIO pin connected to NeoPixel data input

// NeoPixel driver instance
static neopixel_t strip;

// Function prototypes
void colorWipe(uint32_t color, uint32_t wait);
void theaterChase(uint32_t color, uint32_t wait);
void rainbowCycle(uint8_t wait);
uint32_t wheel(uint8_t wheelPos);

int main(void) {
    printk("NeoPixel Driver Test with Zephyr RTOS\n");
    
    // Initialize GPIO pin
    int ret = platform_neopixel_gpio_init(NEOPIXEL_PIN);
    if (ret < 0) {
        printk("Failed to initialize GPIO pin %d\n", NEOPIXEL_PIN);
        return ret;
    }
    
    // Configure platform interface
    neopixel_interface_t interface = {
        .gpio_set_high = platform_gpio_set_high,
        .gpio_set_low = platform_gpio_set_low,
        .delay_us = platform_delay_us,
        .get_us_tick = platform_get_us_tick,
        .disable_interrupts = platform_disable_interrupts,
        .enable_interrupts = platform_enable_interrupts
    };
    
    // Initialize NeoPixel strip
    neopixel_status_t status = neopixel_init(&strip, NEOPIXEL_PIN, LED_COUNT, 
                                            NEO_GRB + NEO_KHZ800, interface);
    if (status != NEOPIXEL_OK) {
        printk("Failed to initialize NeoPixel: %s\n", neopixel_status_str(status));
        return -1;
    }
    
    // Set brightness
    neopixel_set_brightness(&strip, BRIGHTNESS);
    
    printk("NeoPixel initialized successfully!\n");
    printk("Strip has %d LEDs\n", LED_COUNT);
    
    // Clear all pixels
    neopixel_clear(&strip);
    neopixel_show(&strip);
    
    while (1) {
        // Demo 1: Color wipe
        printk("Color wipe demo...\n");
        colorWipe(neopixel_color_rgb(255, 0, 0), 50);    // Red
        colorWipe(neopixel_color_rgb(0, 255, 0), 50);    // Green  
        colorWipe(neopixel_color_rgb(0, 0, 255), 50);    // Blue
        colorWipe(neopixel_color_rgb(0, 0, 0), 50);      // Off
        
        // Demo 2: Rainbow
        printk("Rainbow demo...\n");
        neopixel_rainbow(&strip, 0, 1, 255, 255);
        neopixel_show(&strip);
        k_msleep(1000);
        
        // Demo 3: Rainbow cycle
        printk("Rainbow cycle demo...\n");
        for (int i = 0; i < 5; i++) {  // 5 cycles
            rainbowCycle(20);
        }
        
        // Demo 4: Theater chase
        printk("Theater chase demo...\n");
        theaterChase(neopixel_color_rgb(127, 127, 127), 100); // White
        theaterChase(neopixel_color_rgb(127, 0, 0), 100);     // Red
        theaterChase(neopixel_color_rgb(0, 0, 127), 100);     // Blue
        
        k_msleep(2000);
    }
    
    return 0;
}

// Fill strip pixels one after another with a color
void colorWipe(uint32_t color, uint32_t wait) {
    for (int i = 0; i < LED_COUNT; i++) {
        neopixel_set_pixel_color_packed(&strip, i, color);
        neopixel_show(&strip);
        k_msleep(wait);
    }
}

// Input a value 0 to 255 to get a color value.
// Colors transition r - g - b - back to r.
uint32_t wheel(uint8_t wheelPos) {
    wheelPos = 255 - wheelPos;
    if (wheelPos < 85) {
        return neopixel_color_rgb(255 - wheelPos * 3, 0, wheelPos * 3);
    }
    if (wheelPos < 170) {
        wheelPos -= 85;
        return neopixel_color_rgb(0, wheelPos * 3, 255 - wheelPos * 3);
    }
    wheelPos -= 170;
    return neopixel_color_rgb(wheelPos * 3, 255 - wheelPos * 3, 0);
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
    uint16_t i, j;
    
    for (j = 0; j < 256; j++) {
        for (i = 0; i < LED_COUNT; i++) {
            uint32_t color = wheel(((i * 256 / LED_COUNT) + j) & 255);
            neopixel_set_pixel_color_packed(&strip, i, color);
        }
        neopixel_show(&strip);
        k_msleep(wait);
    }
}

// Theater-style crawling lights
void theaterChase(uint32_t color, uint32_t wait) {
    for (int a = 0; a < 10; a++) {  // Repeat 10 times
        for (int b = 0; b < 3; b++) { // 'b' counts from 0 to 2
            neopixel_clear(&strip);   // Turn all pixels off
            for (int c = b; c < LED_COUNT; c += 3) {
                neopixel_set_pixel_color_packed(&strip, c, color); // Turn every third pixel on
            }
            neopixel_show(&strip);
            k_msleep(wait);
        }
    }
}