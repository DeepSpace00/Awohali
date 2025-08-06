/*!
 * @file neopixel.c
 * @brief Platform-agnostic NeoPixel Driver Implementation
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-05
 */

#include "neopixel.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Gamma correction lookup table (2.6 gamma)
static const uint8_t gamma_table[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,
    1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,
    3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,
    6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,   9,   10,  10,  10,
    11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,
    17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,  24,  25,
    25,  26,  27,  27,  28,  29,  29,  30,  31,  31,  32,  33,  34,  34,  35,
    36,  37,  38,  38,  39,  40,  41,  42,  42,  43,  44,  45,  46,  47,  48,
    49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
    64,  65,  66,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,  80,  81,
    82,  84,  85,  86,  88,  89,  90,  92,  93,  94,  96,  97,  99,  100, 102,
    103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 119, 120, 122, 124, 125,
    127, 129, 130, 132, 134, 136, 137, 139, 141, 143, 145, 146, 148, 150, 152,
    154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 182,
    184, 186, 188, 191, 193, 195, 197, 199, 202, 204, 206, 209, 211, 213, 215,
    218, 220, 223, 225, 227, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252,
    255
};

const char* neopixel_status_str(const neopixel_status_t status) {
    switch (status) {
        case NEOPIXEL_OK:              return "OK";
        case NEOPIXEL_ERR_NULL:        return "Null pointer";
        case NEOPIXEL_ERR_INVALID_ARG: return "Invalid argument";
        case NEOPIXEL_ERR_NOT_INITIALIZED: return "Not initialized";
        case NEOPIXEL_ERR_PLATFORM:   return "Platform error";
        default:                       return "Unknown error";
    }
}

neopixel_status_t neopixel_init(neopixel_t *dev, const uint32_t pin, const uint16_t num_leds,
                               const neopixel_type_t type, const neopixel_interface_t io) {
    if (!dev || !io.gpio_set_high || !io.gpio_set_low || !io.delay_us ||
        !io.get_us_tick || !io.disable_interrupts || !io.enable_interrupts) {
        return NEOPIXEL_ERR_NULL;
    }

    if (num_leds == 0) {
        return NEOPIXEL_ERR_INVALID_ARG;
    }

    // Parse color order and speed from type
    dev->w_offset = (type >> 6) & 0x03;
    dev->r_offset = (type >> 4) & 0x03;
    dev->g_offset = (type >> 2) & 0x03;
    dev->b_offset = type & 0x03;
    dev->is_800khz = !(type & NEO_KHZ400);
    dev->is_rgbw = (dev->w_offset != dev->r_offset);

    // Initialize device parameters
    dev->pin = pin;
    dev->num_leds = num_leds;
    dev->num_bytes = num_leds * (dev->is_rgbw ? 4 : 3);
    dev->brightness = 0; // No brightness scaling initially (0 means full brightness)
    dev->initialized = false;
    dev->end_time = 0;
    dev->io = io;

    // Allocate pixel buffer
    dev->pixels = (uint8_t*)malloc(dev->num_bytes);
    if (!dev->pixels) {
        return NEOPIXEL_ERR_PLATFORM;
    }

    // Clear pixel buffer
    memset(dev->pixels, 0, dev->num_bytes);

    dev->initialized = true;
    return NEOPIXEL_OK;
}

// Platform-optimized show function - implemented in platform-specific files
neopixel_status_t neopixel_show(neopixel_t *dev) {
    if (!dev || !dev->initialized) {
        return NEOPIXEL_ERR_NOT_INITIALIZED;
    }

    // Call platform-specific optimized implementation
    // This will be implemented in neopixel_platform_*.cpp
    extern neopixel_status_t platform_neopixel_show_optimized;
    return platform_neopixel_show_optimized(dev);
}

neopixel_status_t neopixel_set_pixel_color_rgbw(neopixel_t *dev, const uint16_t pixel_index,
                                                uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    if (!dev || !dev->initialized) {
        return NEOPIXEL_ERR_NOT_INITIALIZED;
    }

    if (pixel_index >= dev->num_leds) {
        return NEOPIXEL_ERR_INVALID_ARG;
    }

    // Apply brightness scaling if set
    if (dev->brightness) {
        r = (r * dev->brightness) >> 8;
        g = (g * dev->brightness) >> 8;
        b = (b * dev->brightness) >> 8;
        w = (w * dev->brightness) >> 8;
    }

    uint8_t *p;
    if (dev->is_rgbw) {
        p = &dev->pixels[pixel_index * 4];
        p[dev->w_offset] = w;
    } else {
        p = &dev->pixels[pixel_index * 3];
        // Ignore white component for RGB strips
    }

    p[dev->r_offset] = r;
    p[dev->g_offset] = g;
    p[dev->b_offset] = b;

    return NEOPIXEL_OK;
}

neopixel_status_t neopixel_set_pixel_color_rgb(neopixel_t *dev, const uint16_t pixel_index,
                                               const uint8_t r, const uint8_t g, const uint8_t b) {
    return neopixel_set_pixel_color_rgbw(dev, pixel_index, r, g, b, 0);
}

neopixel_status_t neopixel_set_pixel_color_packed(neopixel_t *dev, const uint16_t pixel_index,
                                                  const uint32_t color) {
    const uint8_t r = (uint8_t)(color >> 16);
    const uint8_t g = (uint8_t)(color >> 8);
    const uint8_t b = (uint8_t)color;
    const uint8_t w = (uint8_t)(color >> 24);

    return neopixel_set_pixel_color_rgbw(dev, pixel_index, r, g, b, w);
}

neopixel_status_t neopixel_set_pixel_color(neopixel_t *dev, const uint16_t pixel_index,
                                           const neopixel_color_t color) {
    return neopixel_set_pixel_color_rgbw(dev, pixel_index, color.r, color.g, color.b, color.w);
}

neopixel_status_t neopixel_get_pixel_color(neopixel_t *dev, const uint16_t pixel_index,
                                           neopixel_color_t *color) {
    if (!dev || !dev->initialized || !color) {
        return NEOPIXEL_ERR_NULL;
    }

    if (pixel_index >= dev->num_leds) {
        return NEOPIXEL_ERR_INVALID_ARG;
    }

    uint8_t *p;

    if (dev->is_rgbw) {
        p = &dev->pixels[pixel_index * 4];
        color->w = p[dev->w_offset];
    } else {
        p = &dev->pixels[pixel_index * 3];
        color->w = 0;
    }

    color->r = p[dev->r_offset];
    color->g = p[dev->g_offset];
    color->b = p[dev->b_offset];

    // If brightness scaling was applied, attempt to scale back
    if (dev->brightness) {
        color->r = (color->r << 8) / dev->brightness;
        color->g = (color->g << 8) / dev->brightness;
        color->b = (color->b << 8) / dev->brightness;
        if (dev->is_rgbw) {
            color->w = (color->w << 8) / dev->brightness;
        }
    }

    return NEOPIXEL_OK;
}

neopixel_status_t neopixel_fill(neopixel_t *dev, const neopixel_color_t color,
                               const uint16_t first, const uint16_t count) {
    if (!dev || !dev->initialized) {
        return NEOPIXEL_ERR_NOT_INITIALIZED;
    }

    if (first >= dev->num_leds) {
        return NEOPIXEL_ERR_INVALID_ARG;
    }

    uint16_t end = (count == 0) ? dev->num_leds : first + count;
    if (end > dev->num_leds) {
        end = dev->num_leds;
    }

    for (uint16_t i = first; i < end; i++) {
        const neopixel_status_t status = neopixel_set_pixel_color(dev, i, color);
        if (status != NEOPIXEL_OK) return status;
    }

    return NEOPIXEL_OK;
}

neopixel_status_t neopixel_fill_packed(neopixel_t *dev, const uint32_t packed_color,
                                      const uint16_t first, const uint16_t count) {
    const neopixel_color_t color = {
        .r = (uint8_t)(packed_color >> 16),
        .g = (uint8_t)(packed_color >> 8),
        .b = (uint8_t)packed_color,
        .w = (uint8_t)(packed_color >> 24)
    };

    return neopixel_fill(dev, color, first, count);
}

neopixel_status_t neopixel_clear(neopixel_t *dev) {
    if (!dev || !dev->initialized) {
        return NEOPIXEL_ERR_NOT_INITIALIZED;
    }

    memset(dev->pixels, 0, dev->num_bytes);
    return NEOPIXEL_OK;
}

neopixel_status_t neopixel_set_brightness(neopixel_t *dev, const uint8_t brightness) {
    if (!dev || !dev->initialized) {
        return NEOPIXEL_ERR_NOT_INITIALIZED;
    }

    const uint8_t new_brightness = brightness + 1; // Internal storage is +1
    if (new_brightness != dev->brightness) {
        // Re-scale existing data in RAM
        const uint8_t old_brightness = dev->brightness ? dev->brightness - 1 : 255;
        uint16_t scale;

        if (old_brightness == 0) {
            scale = 0; // Avoid division by zero
        } else if (brightness == 255) {
            scale = 65535 / old_brightness;
        } else {
            scale = (((uint16_t)new_brightness << 8) - 1) / old_brightness;
        }

        for (uint16_t i = 0; i < dev->num_bytes; i++) {
            const uint8_t c = dev->pixels[i];
            dev->pixels[i] = (c * scale) >> 8;
        }

        dev->brightness = new_brightness;
    }

    return NEOPIXEL_OK;
}

neopixel_status_t neopixel_get_brightness(neopixel_t *dev, uint8_t *brightness) {
    if (!dev || !dev->initialized || !brightness) {
        return NEOPIXEL_ERR_NULL;
    }

    *brightness = dev->brightness ? dev->brightness - 1 : 255;
    return NEOPIXEL_OK;
}

neopixel_status_t neopixel_can_show(neopixel_t *dev, bool *ready) {
    if (!dev || !dev->initialized || !ready) {
        return NEOPIXEL_ERR_NULL;
    }

    const uint32_t now = dev->io.get_us_tick();

    // Handle timer rollover
    if (dev->end_time > now) {
        dev->end_time = now;
    }

    // NeoPixels need ~300us latch time
    *ready = (now - dev->end_time) >= 300;
    return NEOPIXEL_OK;
}

neopixel_status_t neopixel_rainbow(neopixel_t *dev, const uint16_t first_hue, const int8_t reps,
                                  const uint8_t saturation, const uint8_t brightness) {
    if (!dev || !dev->initialized) {
        return NEOPIXEL_ERR_NOT_INITIALIZED;
    }

    for (uint16_t i = 0; i < dev->num_leds; i++) {
        const uint16_t hue = first_hue + (i * reps * 65536) / dev->num_leds;
        uint32_t color = neopixel_color_hsv(hue, saturation, brightness);
        color = neopixel_gamma32(color);
        const neopixel_status_t status = neopixel_set_pixel_color_packed(dev, i, color);
        if (status != NEOPIXEL_OK) return status;
    }

    return NEOPIXEL_OK;
}

// Utility functions

uint32_t neopixel_color_rgb(const uint8_t r, const uint8_t g, const uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint32_t neopixel_color_rgbw(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t w) {
    return ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint32_t neopixel_color_hsv(uint16_t hue, const uint8_t sat, const uint8_t val) {
    uint8_t r, g, b;

    // Remap 0-65535 to 0-1529
    hue = (hue * 1530L + 32768) / 65536;
    if (hue >= 1530) hue = 1529;

    // Convert hue to R,G,B
    if (hue < 510) { // Red to Green-1
        b = 0;
        if (hue < 255) { // Red to Yellow-1
            r = 255;
            g = hue;
        } else { // Yellow to Green-1
            r = 510 - hue;
            g = 255;
        }
    } else if (hue < 1020) { // Green to Blue-1
        r = 0;
        if (hue < 765) { // Green to Cyan-1
            g = 255;
            b = hue - 510;
        } else { // Cyan to Blue-1
            g = 1020 - hue;
            b = 255;
        }
    } else if (hue < 1530) { // Blue to Red-1
        g = 0;
        if (hue < 1275) { // Blue to Magenta-1
            r = hue - 1020;
            b = 255;
        } else { // Magenta to Red-1
            r = 255;
            b = 1530 - hue;
        }
    } else { // Last 0.5 Red
        r = 255;
        g = b = 0;
    }

    // Apply saturation and value
    const uint32_t v1 = 1 + val;  // 1 to 256
    const uint16_t s1 = 1 + sat;  // 1 to 256
    const uint8_t s2 = 255 - sat; // 255 to 0

    return ((((((r * s1) >> 8) + s2) * v1) & 0xff00) << 8) |
           (((((g * s1) >> 8) + s2) * v1) & 0xff00) |
           (((((b * s1) >> 8) + s2) * v1) >> 8);
}

uint8_t neopixel_gamma8(const uint8_t x) {
    return gamma_table[x];
}

uint32_t neopixel_gamma32(uint32_t color) {
    uint8_t *components = (uint8_t*)&color;
    for (uint8_t i = 0; i < 4; i++) {
        components[i] = neopixel_gamma8(components[i]);
    }
    return color;
}

neopixel_type_t neopixel_str_to_type(const char *order_str) {
    if (!order_str) return NEO_GRB; // Default

    int8_t r = 0, g = 0, b = 0, w = -1;
    char c;

    for (int8_t i = 0; (c = tolower(order_str[i])) != '\0'; i++) {
        if (c == 'r') r = i;
        else if (c == 'g') g = i;
        else if (c == 'b') b = i;
        else if (c == 'w') w = i;
    }

    r &= 3;
    g &= 3;
    b &= 3;

    if (w < 0) w = r; // If 'w' not specified, duplicate r bits
    w &= 3;

    return (w << 6) | (r << 4) | (g << 2) | b;
}