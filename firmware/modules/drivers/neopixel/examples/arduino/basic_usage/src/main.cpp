/*!
 * @file main.cpp
 * @brief NeoPixel test for Adafruit STM32F405 Feather (168MHz)
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-04
 *
 * STM32F405 runs at 168MHz, not 16MHz like Arduino Uno!
 * Timing needs to be completely different.
 */

#include <Arduino.h>

#define LED_PIN 8  // Pin 8 on STM32F405 Feather

// STM32F405 @ 168MHz timing
// Each NOP is ~6ns at 168MHz (vs 62.5ns on Arduino Uno)
// We need MANY more NOPs for the same timing

void sendBit_STM32(bool bit) {
    if (bit) {
        // Send '1': ~850ns high, ~400ns low
        digitalWrite(LED_PIN, HIGH);
        // 850ns @ 168MHz ≈ 142 NOPs (850ns / 6ns)
        for(volatile int i = 0; i < 35; i++) {
            __asm__("nop\nnop\nnop\nnop\n");
        }
        digitalWrite(LED_PIN, LOW);
        // 400ns @ 168MHz ≈ 67 NOPs (400ns / 6ns)
        for(volatile int i = 0; i < 17; i++) {
            __asm__("nop\nnop\nnop\nnop\n");
        }
    } else {
        // Send '0': ~400ns high, ~850ns low
        digitalWrite(LED_PIN, HIGH);
        // 400ns @ 168MHz ≈ 67 NOPs
        for(volatile int i = 0; i < 17; i++) {
            __asm__("nop\nnop\nnop\nnop\n");
        }
        digitalWrite(LED_PIN, LOW);
        // 850ns @ 168MHz ≈ 142 NOPs
        for(volatile int i = 0; i < 35; i++) {
            __asm__("nop\nnop\nnop\nnop\n");
        }
    }
}

// Alternative: Use cycle-accurate timing with DWT
void sendBit_DWT(bool bit) {
    // Enable DWT if not already enabled
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t start = DWT->CYCCNT;

    if (bit) {
        // Send '1': ~850ns high, ~400ns low
        digitalWrite(LED_PIN, HIGH);
        while((DWT->CYCCNT - start) < (168 * 850 / 1000)); // 850ns in cycles

        start = DWT->CYCCNT;
        digitalWrite(LED_PIN, LOW);
        while((DWT->CYCCNT - start) < (168 * 400 / 1000)); // 400ns in cycles
    } else {
        // Send '0': ~400ns high, ~850ns low
        digitalWrite(LED_PIN, HIGH);
        while((DWT->CYCCNT - start) < (168 * 400 / 1000)); // 400ns in cycles

        start = DWT->CYCCNT;
        digitalWrite(LED_PIN, LOW);
        while((DWT->CYCCNT - start) < (168 * 850 / 1000)); // 850ns in cycles
    }
}

void sendByte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        sendBit_DWT(byte & (1 << i));  // Use DWT timing
    }
}

void sendPixel_GRB(uint8_t r, uint8_t g, uint8_t b) {
    sendByte(g);
    sendByte(r);
    sendByte(b);
}

void endFrame() {
    digitalWrite(LED_PIN, LOW);
    delayMicroseconds(500);  // Latch time
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("=== STM32F405 Feather NeoPixel Test ===");
    Serial.print("System Clock: ");
    Serial.print(SystemCoreClock);
    Serial.println(" Hz");
    Serial.print("Pin: ");
    Serial.println(LED_PIN);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Enable DWT for cycle-accurate timing
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    Serial.println("DWT cycle counter enabled");
    Serial.println("Starting test in 2 seconds...");

    endFrame();
    delay(2000);
}

void loop() {
    Serial.println("RED ON (GRB order)");
    noInterrupts();
    sendPixel_GRB(100, 0, 0);  // Red
    interrupts();
    endFrame();
    delay(2000);

    Serial.println("RED OFF");
    noInterrupts();
    sendPixel_GRB(0, 0, 0);  // Off
    interrupts();
    endFrame();
    delay(2000);

    Serial.println("GREEN ON (GRB order)");
    noInterrupts();
    sendPixel_GRB(0, 100, 0);  // Green
    interrupts();
    endFrame();
    delay(2000);

    Serial.println("GREEN OFF");
    noInterrupts();
    sendPixel_GRB(0, 0, 0);  // Off
    interrupts();
    endFrame();
    delay(2000);

    Serial.println("BLUE ON (GRB order)");
    noInterrupts();
    sendPixel_GRB(0, 0, 100);  // Blue
    interrupts();
    endFrame();
    delay(2000);

    Serial.println("BLUE OFF");
    noInterrupts();
    sendPixel_GRB(0, 0, 0);  // Off
    interrupts();
    endFrame();
    delay(2000);

    Serial.println("WHITE ON (all colors)");
    noInterrupts();
    sendPixel_GRB(50, 50, 50);  // White
    interrupts();
    endFrame();
    delay(2000);

    Serial.println("WHITE OFF");
    noInterrupts();
    sendPixel_GRB(0, 0, 0);  // Off
    interrupts();
    endFrame();
    delay(3000);
}