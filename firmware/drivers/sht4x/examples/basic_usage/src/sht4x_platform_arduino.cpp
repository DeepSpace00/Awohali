/**
 * @file sht4x_platform_arduino.cpp
 * @brief Platform abstraction layer for SHT4x I2C driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-15
 */

#include <Arduino.h>
#include <Wire.h>

#define SHT4X_I2C_TIMEOUT 100

extern "C" {
#include "sht4x_platform.h"
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

void platform_delay_ms(const uint32_t ms) {
    delay(ms);
}
}