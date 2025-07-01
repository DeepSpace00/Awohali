#include "neopixel.h"

static uint8_t pixel_data[3] = {0}; // LED order: GRB

void neopixel_init(void) {
    // Nothing needed
}

void neopixel_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    pixel_data[0] = green;
    pixel_data[1] = red;
    pixel_data[2] = blue;
}

static void send_byte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (byte & 0x80) {
            // Send '1'
            neopixel_gpio_set(true);
            neopixel_delay_ns(800);
            neopixel_gpio_set(false);
            neopixel_delay_ns(450);
        }
        else {
            // Send '0'
            neopixel_gpio_set(true);
            neopixel_delay_ns(400);
            neopixel_gpio_set(false);
            neopixel_delay_ns(850);
        }
        byte <<= 1;
    }
}

void neopixel_show(void)
{
    for (uint8_t i = 0; i < 3; i++)
        send_byte(pixel_data[i]);

    // Reset pulse
    neopixel_gpio_set(false);
    neopixel_delay_ns(50000);
}