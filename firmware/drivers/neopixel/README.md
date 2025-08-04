# Platform-Agnostic NeoPixel Driver

A C driver for WS2812/WS2811/SK6812 NeoPixel LED strips that can be used across Arduino, STM32CubeIDE, and Zephyr RTOS platforms.

## Features

- **Platform Agnostic**: Works with Arduino, STM32 HAL, and Zephyr RTOS
- **Comprehensive API**: Full feature set including RGB/RGBW support, brightness control, and utility functions
- **Precise Timing**: Optimized timing implementations for each platform
- **Color Effects**: Built-in rainbow, color wipe, and theater chase effects
- **Memory Efficient**: Dynamic memory allocation with proper cleanup
- **HSV Support**: Convert HSV colors to RGB
- **Gamma Correction**: Built-in gamma correction for better color appearance

## Supported Devices

- WS2812/WS2812B (most common)
- WS2811
- SK6812 (RGBW)
- Any compatible addressable LED strip

## File Structure

```
neopixel/
├── neopixel.h                    # Main driver header
├── neopixel.c                    # Main driver implementation
├── neopixel_platform.h           # Platform abstraction header
├── platforms/
│   ├── neopixel_platform_arduino.cpp    # Arduino implementation
│   ├── neopixel_platform_stm32.c        # STM32 HAL implementation
│   └── neopixel_platform_zephyr.c       # Zephyr RTOS implementation
└── examples/
    ├── example_arduino.cpp       # Arduino example
    ├── example_stm32.c          # STM32 example
    └── example_zephyr.c         # Zephyr example
```

## Platform Setup

### Arduino
1. Copy `neopixel.h`, `neopixel.c`, and `neopixel_platform_arduino.cpp` to your project
2. Configure the GPIO pin in your sketch
3. See `example_arduino.cpp` for usage

### STM32 (CubeIDE/HAL)
1. Copy `neopixel.h`, `neopixel.c`, and `neopixel_platform_stm32.c` to your project
2. Configure a GPIO pin as output in CubeMX
3. Adjust the pin mapping in `neopixel_platform_stm32.c`
4. Enable DWT (Debug Watchpoint and Trace) for precise timing
5. See `example_stm32.c` for usage

**Important STM32 Notes:**
- The timing is optimized for STM32F4 at 168MHz - adjust NOPs for other speeds
- Make sure to enable DWT in debug configuration
- Consider using DMA for better performance with large LED strips

### Zephyr RTOS
1. Copy `neopixel.h`, `neopixel.c`, and `neopixel_platform_zephyr.c` to your project
2. Configure GPIO in your device tree
3. Adjust `GPIO_NODE` in the platform file to match your board
4. See `example_zephyr.c` for usage

## Basic Usage

```c
#include "neopixel.h"
#include "neopixel_platform.h"

// Create driver instance
neopixel_t strip;

// Configure platform interface
neopixel_interface_t interface = {
    .gpio_set_high = platform_gpio_set_high,
    .gpio_set_low = platform_gpio_set_low,
    .delay_us = platform_delay_us,
    .get_us_tick = platform_get_us_tick,
    .disable_interrupts = platform_disable_interrupts,
    .enable_interrupts = platform_enable_interrupts
};

// Initialize strip
neopixel_status_t status = neopixel_init(&strip, GPIO_PIN, LED_COUNT, 
                                        NEO_GRB + NEO_KHZ800, interface);

// Set pixel colors
neopixel_set_pixel_color_rgb(&strip, 0, 255, 0, 0);  // First pixel red
neopixel_set_pixel_color_rgb(&strip, 1, 0, 255, 0);  // Second pixel green
neopixel_set_pixel_color_rgb(&strip, 2, 0, 0, 255);  // Third pixel blue

// Update the strip
neopixel_show(&strip);
```

## Color Order Configuration

Different NeoPixel strips may have different color orders. Use the appropriate constant:

- `NEO_RGB` - Red, Green, Blue
- `NEO_GRB` - Green, Red, Blue (most common)
- `NEO_BGR` - Blue, Green, Red
- `NEO_RGBW` - Red, Green, Blue, White (for RGBW strips)
- Add `NEO_KHZ400` for older 400kHz strips (default is 800kHz)

Example: `NEO_GRB + NEO_KHZ800`

## API Reference

### Initialization
- `neopixel_init()` - Initialize the driver
- `neopixel_status_str()` - Get human-readable status string

### Pixel Control
- `neopixel_set_pixel_color_rgb()` - Set pixel using R,G,B values
- `neopixel_set_pixel_color_rgbw()` - Set pixel using R,G,B,W values
- `neopixel_set_pixel_color_packed()` - Set pixel using packed 32-bit color
- `neopixel_get_pixel_color()` - Get current pixel color
- `neopixel_show()` - Update the strip with new data

### Strip Control
- `neopixel_clear()` - Turn off all pixels
- `neopixel_fill()` - Fill range with color
- `neopixel_set_brightness()` - Set global brightness (0-255)
- `neopixel_get_brightness()` - Get current brightness setting
- `neopixel_can_show()` - Check if strip is ready for new data

### Effects
- `neopixel_rainbow()` - Create rainbow pattern across strip

### Utility Functions
- `neopixel_color_rgb()` - Create packed RGB color
- `neopixel_color_rgbw()` - Create packed RGBW color  
- `neopixel_color_hsv()` - Convert HSV to RGB color
- `neopixel_gamma8()` - Apply gamma correction to 8-bit value
- `neopixel_gamma32()` - Apply gamma correction to packed color
- `neopixel_str_to_type()` - Convert color order string to type constant

## Timing Considerations

NeoPixel LEDs require precise timing:
- **800kHz (WS2812B)**: 1.25μs per bit (850ns high/400ns low for '1', 400ns high/850ns low for '0')
- **400kHz (WS2811)**: 2.5μs per bit (1.2μs high/1.3μs low for '1', 0.5μs high/2.0μs low for '0')
- **Latch time**: 300μs minimum between transmissions

The driver handles timing automatically, but for best results:
- Minimize interrupt latency during transmission
- Use the platform-optimized implementations when available
- Consider using DMA on capable platforms for large strips

## Performance Tips

1. **Group Updates**: Update multiple pixels before calling `neopixel_show()`
2. **Use Packed Colors**: `neopixel_color_rgb()` is faster than separate R,G,B calls
3. **Brightness**: Set brightness once during initialization rather than per-pixel
4. **Memory**: Free the driver properly when done to avoid memory leaks
5. **Large Strips**: Consider using DMA-based implementations for >100 LEDs

## Color Examples

```c
// Basic colors
uint32_t red    = neopixel_color_rgb(255, 0, 0);
uint32_t green  = neopixel_color_rgb(0, 255, 0);
uint32_t blue   = neopixel_color_rgb(0, 0, 255);
uint32_t white  = neopixel_color_rgb(255, 255, 255);
uint32_t off    = neopixel_color_rgb(0, 0, 0);

// HSV colors (hue, saturation, value)
uint32_t rainbow_red = neopixel_color_hsv(0, 255, 255);      // Red
uint32_t rainbow_green = neopixel_color_hsv(21845, 255, 255); // Green  
uint32_t rainbow_blue = neopixel_color_hsv(43690, 255, 255);  // Blue

// RGBW (for SK6812 strips)
uint32_t warm_white = neopixel_color_rgbw(0, 0, 0, 255);
uint32_t cool_white = neopixel_color_rgbw(255, 255, 255, 0);
```

## Common Issues & Solutions

### LEDs Not Lighting Up
- Check power supply (5V, adequate current)
- Verify GPIO pin configuration
- Ensure correct color order (try different NEO_* constants)
- Check wiring (Data In pin)

### Wrong Colors
- Try different color order constants (NEO_GRB vs NEO_RGB, etc.)
- Check if strip is RGBW when expecting RGB
- Verify voltage levels (3.3V MCU may need level shifter for 5V LEDs)

### Flickering/Glitching  
- Minimize interrupt latency during transmission
- Add decoupling capacitors near LEDs
- Keep data wires short and use appropriate pullup/pulldown
- Check power supply stability

### Performance Issues
- Use platform-optimized timing where available
- Consider DMA implementations for large strips
- Group pixel updates before calling show()

## Platform-Specific Notes

### STM32
- Requires DWT (Data Watchpoint and Trace) for precise timing
- Timing optimized for STM32F4 @ 168MHz - adjust for other speeds
- Consider using SPI/DMA for better performance with large strips
- Pin mapping function needs customization for your board

### Arduino
- Works with most Arduino-compatible boards
- ESP32/ESP8266 may need special timing adjustments
- AVR boards use inline assembly for precise timing
- Some boards may need level shifters for 5V LEDs

### Zephyr
- Requires proper GPIO configuration in device tree
- Uses Zephyr's timing APIs for portability
- May need timing adjustments for different SoCs
- Consider real-time thread priority for critical timing

## License

This driver is based on the Adafruit NeoPixel library and maintains compatibility with open-source licensing. Check individual file headers for specific license information.

## Contributing

When adding support for new platforms:
1. Implement the platform interface functions in `neopixel_platform_yourplatform.c`
2. Optimize timing for your target hardware
3. Add example code demonstrating usage
4. Test with different LED strip types and lengths
5. Update this README with platform-specific notes

## Version History

- v1.0.0 - Initial release with Arduino, STM32, and Zephyr support
- Platform-agnostic architecture based on Adafruit NeoPixel library
- Full RGB/RGBW support with comprehensive API