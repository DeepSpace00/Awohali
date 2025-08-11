/**
 * @file bq25798_platform_arduino.cpp
 * @brief Platform abstraction layer implementation for BQ25798 I2C driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-21
 *
 * This file should be placed in the src folder and contains platform-specific
 * implementations. Only one version should be compiled based on your platform.
 */

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include <bq25798_platform.h>
}

extern "C" {
static TwoWire *i2c_wire = &Wire;

int platform_i2c_write(const uint8_t dev_addr, const uint8_t *data, const uint16_t len) {
    i2c_wire->beginTransmission(dev_addr);
    for (uint16_t i = 0; i < len; i++) {
        i2c_wire->write(data[i]);
    }
    const int result = i2c_wire->endTransmission();
    return (result == 0) ? 0 : -1;
}

int platform_i2c_read(const uint8_t dev_addr, uint8_t *data, const uint16_t len) {
    delay(2); // Add a short delay before reading
    i2c_wire->requestFrom(static_cast<int>(dev_addr), (int)len);
    if (i2c_wire->available() != len) return -1;
    for (uint16_t i = 0; i < len; i++) {
        data[i] = i2c_wire->read();
    }
    return 0;
}

void platform_delay_ms(uint32_t ms) {
    delay(ms);
}
}