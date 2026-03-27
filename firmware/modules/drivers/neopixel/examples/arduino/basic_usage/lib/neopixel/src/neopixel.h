/*!
 * @file neopixel.h
 * @brief Platform-agnostic NeoPixel (WS2812/WS2811) LED Strip Driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-04
 * 
 * This driver supports WS2812, WS2812B, WS2811, SK6812 and compatible LEDs
 * Based on Adafruit NeoPixel library but made platform-agnostic
 */

#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Color order definitions - bits encode byte offsets for R,G,B,W
// Format: 0bWWRRGGBB for RGBW, 0bRRRRGGBB for RGB
#define NEO_RGB  ((0 << 6) | (0 << 4) | (1 << 2) | (2))  // R,G,B
#define NEO_RBG  ((0 << 6) | (0 << 4) | (2 << 2) | (1))  // R,B,G
#define NEO_GRB  ((1 << 6) | (1 << 4) | (0 << 2) | (2))  // G,R,B (most common)
#define NEO_GBR  ((2 << 6) | (2 << 4) | (0 << 2) | (1))  // G,B,R
#define NEO_BRG  ((1 << 6) | (1 << 4) | (2 << 2) | (0))  // B,R,G
#define NEO_BGR  ((2 << 6) | (2 << 4) | (1 << 2) | (0))  // B,G,R

// RGBW variants
#define NEO_WRGB ((0 << 6) | (1 << 4) | (2 << 2) | (3))  // W,R,G,B
#define NEO_WRBG ((0 << 6) | (1 << 4) | (3 << 2) | (2))  // W,R,B,G
#define NEO_WGRB ((0 << 6) | (2 << 4) | (1 << 2) | (3))  // W,G,R,B
#define NEO_WGBR ((0 << 6) | (3 << 4) | (1 << 2) | (2))  // W,G,B,R
#define NEO_WBRG ((0 << 6) | (2 << 4) | (3 << 2) | (1))  // W,B,R,G
#define NEO_WBGR ((0 << 6) | (3 << 4) | (2 << 2) | (1))  // W,B,G,R

#define NEO_RWGB ((1 << 6) | (0 << 4) | (2 << 2) | (3))  // R,W,G,B
#define NEO_RWBG ((1 << 6) | (0 << 4) | (3 << 2) | (2))  // R,W,B,G
#define NEO_RGWB ((2 << 6) | (0 << 4) | (1 << 2) | (3))  // R,G,W,B
#define NEO_RGBW ((3 << 6) | (0 << 4) | (1 << 2) | (2))  // R,G,B,W
#define NEO_RBWG ((2 << 6) | (0 << 4) | (3 << 2) | (1))  // R,B,W,G
#define NEO_RBGW ((3 << 6) | (0 << 4) | (2 << 2) | (1))  // R,B,G,W

#define NEO_GWRB ((1 << 6) | (2 << 4) | (0 << 2) | (3))  // G,W,R,B
#define NEO_GWBR ((1 << 6) | (3 << 4) | (0 << 2) | (2))  // G,W,B,R
#define NEO_GRWB ((2 << 6) | (1 << 4) | (0 << 2) | (3))  // G,R,W,B
#define NEO_GRBW ((3 << 6) | (1 << 4) | (0 << 2) | (2))  // G,R,B,W
#define NEO_GBWR ((2 << 6) | (3 << 4) | (0 << 2) | (1))  // G,B,W,R
#define NEO_GBRW ((3 << 6) | (2 << 4) | (0 << 2) | (1))  // G,B,R,W

#define NEO_BWRG ((1 << 6) | (2 << 4) | (3 << 2) | (0))  // B,W,R,G
#define NEO_BWGR ((1 << 6) | (3 << 4) | (2 << 2) | (0))  // B,W,G,R
#define NEO_BRWG ((2 << 6) | (1 << 4) | (3 << 2) | (0))  // B,R,W,G
#define NEO_BRGW ((3 << 6) | (1 << 4) | (2 << 2) | (0))  // B,R,G,W
#define NEO_BGWR ((2 << 6) | (3 << 4) | (1 << 2) | (0))  // B,G,W,R
#define NEO_BGRW ((3 << 6) | (2 << 4) | (1 << 2) | (0))  // B,G,R,W

// Speed definitions
#define NEO_KHZ800 0x0000  // 800 KHz datastream (default)
#define NEO_KHZ400 0x0100  // 400 KHz datastream (older v1 pixels)

typedef uint16_t neopixel_type_t;

/**
 * @brief NeoPixel driver return status codes
 */
typedef enum {
    NEOPIXEL_OK = 0,
    NEOPIXEL_ERR_NULL = -1,
    NEOPIXEL_ERR_INVALID_ARG = -2,
    NEOPIXEL_ERR_NOT_INITIALIZED = -3,
    NEOPIXEL_ERR_PLATFORM = -4
} neopixel_status_t;

/**
 * @brief Platform interface abstraction for NeoPixel driver
 */
typedef struct {
    int (*gpio_set_high)(uint32_t pin);
    int (*gpio_set_low)(uint32_t pin);
    void (*delay_us)(uint32_t us);
    uint32_t (*get_us_tick)(void);
    void (*disable_interrupts)(void);
    void (*enable_interrupts)(void);
} neopixel_interface_t;

/**
 * @brief NeoPixel color structure
 */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t w;  // White component (0 if RGB-only)
} neopixel_color_t;

/**
 * @brief NeoPixel driver instance
 */
typedef struct {
    uint32_t pin;
    uint16_t num_leds;
    uint16_t num_bytes;
    uint8_t *pixels;
    uint8_t brightness;
    uint8_t r_offset;
    uint8_t g_offset;
    uint8_t b_offset;
    uint8_t w_offset;
    bool is_800khz;
    bool is_rgbw;
    bool initialized;
    uint32_t end_time;
    neopixel_interface_t io;
} neopixel_t;

/**
 * @brief Human-readable description of a NeoPixel status code
 * @param status The status code to translate
 * @return A string describing the status
 */
const char* neopixel_status_str(neopixel_status_t status);

/**
 * @brief Initialize the NeoPixel driver
 * @param dev Pointer to driver handle
 * @param pin GPIO pin number for data output
 * @param num_leds Number of LEDs in the strip
 * @param type Color order and speed (e.g., NEO_GRB + NEO_KHZ800)
 * @param io Platform interface functions
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_init(neopixel_t *dev, uint32_t pin, uint16_t num_leds, 
                               neopixel_type_t type, neopixel_interface_t io);

/**
 * @brief Transmit pixel data to the LED strip
 * @param dev Pointer to initialized driver struct
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_show(neopixel_t *dev);

/**
 * @brief Set a pixel color using separate R,G,B components
 * @param dev Pointer to initialized driver struct
 * @param pixel_index Pixel index (0 = first pixel)
 * @param r Red brightness (0-255)
 * @param g Green brightness (0-255)
 * @param b Blue brightness (0-255)
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_set_pixel_color_rgb(neopixel_t *dev, uint16_t pixel_index,
                                               uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set a pixel color using separate R,G,B,W components
 * @param dev Pointer to initialized driver struct
 * @param pixel_index Pixel index (0 = first pixel)
 * @param r Red brightness (0-255)
 * @param g Green brightness (0-255)
 * @param b Blue brightness (0-255)
 * @param w White brightness (0-255, ignored for RGB strips)
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_set_pixel_color_rgbw(neopixel_t *dev, uint16_t pixel_index,
                                                uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/**
 * @brief Set a pixel color using a packed 32-bit color value
 * @param dev Pointer to initialized driver struct
 * @param pixel_index Pixel index (0 = first pixel)
 * @param color Pointer to color struct
 * @param color Packed color (0xWWRRGGBB or 0x00RRGGBB)
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_set_pixel_color_packed(neopixel_t *dev, uint16_t pixel_index,
                                                  uint32_t color);

/**
 * @brief Set pixel color using neopixel_color_t structure
 * @param dev Pointer to initialized driver struct
 * @param pixel_index Pixel index (0 = first pixel)
 * @param color Color structure
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_set_pixel_color(neopixel_t *dev, uint16_t pixel_index, 
                                           neopixel_color_t color);

/**
 * @brief Get a pixel's current color
 * @param dev Pointer to initialized driver struct
 * @param pixel_index Pixel index (0 = first pixel)
 * @param color Pointer to store the color
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_get_pixel_color(neopixel_t *dev, uint16_t pixel_index, 
                                           neopixel_color_t *color);

/**
 * @brief Fill all or part of the strip with a color
 * @param dev Pointer to initialized driver struct
 * @param color Color to fill with
 * @param first First pixel index to fill (0 if unspecified)
 * @param count Number of pixels to fill (0 = fill to end)
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_fill(neopixel_t *dev, neopixel_color_t color, 
                               uint16_t first, uint16_t count);

/**
 * @brief Fill entire strip with a packed color
 * @param dev Pointer to initialized driver struct
 * @param packed_color Packed color value (0xWWRRGGBB or 0x00RRGGBB)
 * @param first First pixel index to fill (0 if unspecified)
 * @param count Number of pixels to fill (0 = fill to end)
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_fill_packed(neopixel_t *dev, uint32_t packed_color, 
                                      uint16_t first, uint16_t count);

/**
 * @brief Clear all pixels (set to black/off)
 * @param dev Pointer to initialized driver struct
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_clear(neopixel_t *dev);

/**
 * @brief Set global brightness for the strip
 * @param dev Pointer to initialized driver struct
 * @param brightness Brightness level (0-255)
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_set_brightness(neopixel_t *dev, uint8_t brightness);

/**
 * @brief Get current brightness setting
 * @param dev Pointer to initialized driver struct
 * @param brightness Pointer to store brightness value
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_get_brightness(neopixel_t *dev, uint8_t *brightness);

/**
 * @brief Check if strip is ready for new data transmission
 * @param dev Pointer to initialized driver struct
 * @param ready Pointer to store ready status
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_can_show(neopixel_t *dev, bool *ready);

/**
 * @brief Create rainbow pattern across the strip
 * @param dev Pointer to initialized driver struct
 * @param first_hue Starting hue (0-65535)
 * @param reps Number of rainbow cycles (1 = one full rainbow)
 * @param saturation Color saturation (0-255, default 255)
 * @param brightness Color brightness (0-255, default 255)
 * @return NEOPIXEL_OK on success, or an error code
 */
neopixel_status_t neopixel_rainbow(neopixel_t *dev, uint16_t first_hue, int8_t reps, 
                                  uint8_t saturation, uint8_t brightness);

// Utility functions for color manipulation

/**
 * @brief Create packed RGB color from components
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @return Packed 32-bit color
 */
uint32_t neopixel_color_rgb(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Create packed RGBW color from components
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @param w White component (0-255)
 * @return Packed 32-bit color
 */
uint32_t neopixel_color_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/**
 * @brief Convert HSV to RGB color
 * @param hue Hue (0-65535, represents full color wheel)
 * @param sat Saturation (0-255)
 * @param val Value/brightness (0-255)
 * @return Packed 32-bit RGB color
 */
uint32_t neopixel_color_hsv(uint16_t hue, uint8_t sat, uint8_t val);

/**
 * @brief Apply gamma correction to an 8-bit color component
 * @param x Input color component (0-255)
 * @return Gamma-corrected component (0-255)
 */
uint8_t neopixel_gamma8(uint8_t x);

/**
 * @brief Apply gamma correction to a packed 32-bit color
 * @param color Input packed color
 * @return Gamma-corrected packed color
 */
uint32_t neopixel_gamma32(uint32_t color);

/**
 * @brief Convert color order string to neopixel_type_t constant
 * @param order_str Color order string (e.g., "GRB", "RGBW")
 * @return NeoPixel type constant
 */
neopixel_type_t neopixel_str_to_type(const char *order_str);

#ifdef __cplusplus
}
#endif

#endif // NEOPIXEL_H