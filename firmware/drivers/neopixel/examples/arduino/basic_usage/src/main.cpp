#include <Arduino.h>
#include <neopixel.h>
#include <neopixel_platform.h>

#define LED_PIN     6      // GPIO pin connected to NeoPixel data input
#define LED_COUNT   60     // Number of NeoPixels in strip
#define BRIGHTNESS  128    // Set brightness (0-255)

// NeoPixel driver instance
neopixel_t strip;

void colorWipe(uint32_t color, int wait);
void rainbowCycle(uint8_t wait);
void theaterChase(uint32_t color, int wait);
uint32_t wheel(uint8_t wheelPos);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("NeoPixel Driver Test");

    // Set up GPIO pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Configure platform interface
    constexpr neopixel_interface_t interface = {
        .gpio_set_high = platform_gpio_set_high,
        .gpio_set_low = platform_gpio_set_low,
        .delay_us = platform_delay_us,
        .get_us_tick = platform_get_us_tick,
        .disable_interrupts = platform_disable_interrupts,
        .enable_interrupts = platform_enable_interrupts
    };

    // Initialize NeoPixel strip
    const neopixel_status_t status = neopixel_init(&strip, LED_PIN, LED_COUNT,
                                            NEO_GRB + NEO_KHZ800, interface);
    if (status != NEOPIXEL_OK) {
        Serial.print("Failed to initialize NeoPixel: ");
        Serial.println(neopixel_status_str(status));
        while (true) delay(100);
    }

    // Set brightness
    neopixel_set_brightness(&strip, BRIGHTNESS);

    Serial.println("NeoPixel initialized successfully!");
    Serial.print("Strip has ");
    Serial.print(LED_COUNT);
    Serial.println(" LEDs");

    // Clear all pixels
    neopixel_clear(&strip);
    neopixel_show(&strip);
}

void loop() {
    // Demo 1: Color wipe
    Serial.println("Color wipe demo...");
    colorWipe(neopixel_color_rgb(255, 0, 0), 50);    // Red
    colorWipe(neopixel_color_rgb(0, 255, 0), 50);    // Green
    colorWipe(neopixel_color_rgb(0, 0, 255), 50);    // Blue
    colorWipe(neopixel_color_rgb(0, 0, 0), 50);      // Off

    // Demo 2: Rainbow
    Serial.println("Rainbow demo...");
    neopixel_rainbow(&strip, 0, 1, 255, 255);
    neopixel_show(&strip);
    delay(1000);

    // Demo 3: Rainbow cycle
    Serial.println("Rainbow cycle demo...");
    for (int i = 0; i < 256; i++) {
        rainbowCycle(i);
        delay(20);
    }

    // Demo 4: Theater chase
    Serial.println("Theater chase demo...");
    theaterChase(neopixel_color_rgb(127, 127, 127), 50); // White
    theaterChase(neopixel_color_rgb(127, 0, 0), 50);     // Red
    theaterChase(neopixel_color_rgb(0, 0, 127), 50);     // Blue

    delay(2000);
}

// Fill strip pixels one after another with a color
void colorWipe(const uint32_t color, const int wait) {
    for (int i = 0; i < LED_COUNT; i++) {
        neopixel_set_pixel_color_packed(&strip, i, color);
        neopixel_show(&strip);
        delay(wait);
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
void rainbowCycle(const uint8_t wait) {
    for (uint16_t j = 0; j < 256; j++) {
        for (uint16_t i = 0; i < LED_COUNT; i++) {
            const uint32_t color = wheel(((i * 256 / LED_COUNT) + j) & 255);
            neopixel_set_pixel_color_packed(&strip, i, color);
        }
        neopixel_show(&strip);
        delay(wait);
    }
}

// Theater-style crawling lights
void theaterChase(const uint32_t color, const int wait) {
    for (int a = 0; a < 10; a++) {  // Repeat 10 times
        for (int b = 0; b < 3; b++) { // 'b' counts from 0 to 2
            neopixel_clear(&strip);   // Turn all pixels off
            for (int c = b; c < LED_COUNT; c += 3) {
                neopixel_set_pixel_color_packed(&strip, c, color); // Turn every third pixel on
            }
            neopixel_show(&strip);
            delay(wait);
        }
    }
}