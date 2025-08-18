/*!
 * @file zedf9p_platform_arduino.cpp
 * @brief Platform abstraction layer implementation for ZEDF9P GNSS driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-15
 *
 * This file should be placed in the src folder and contains platform-specific
 * implementations for Arduino. Only one version should be compiled based on your platform.
 * Supports both I2C and UART interfaces for the ZEDF9P GNSS module.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

extern "C" {
#include <zedf9p_platform.h>
}

extern "C" {

// Global interface pointers - set during initialization
static TwoWire *i2c_wire = &Wire;
static HardwareSerial *hw_serial = nullptr;
static SoftwareSerial *sw_serial = nullptr;

// I2C Implementation
int platform_i2c_write(const uint8_t dev_addr, const uint8_t *data, const uint16_t len) {
    if (!i2c_wire) return -1;

    i2c_wire->beginTransmission(dev_addr);
    for (uint16_t i = 0; i < len; i++) {
        i2c_wire->write(data[i]);
    }
    const int result = i2c_wire->endTransmission();
    return (result == 0) ? 0 : -1;
}

int platform_i2c_read(const uint8_t dev_addr, uint8_t *data, const uint16_t len) {
    if (!i2c_wire) return -1;

    delay(2); // Add a short delay before reading
    i2c_wire->requestFrom(static_cast<int>(dev_addr), (int)len);
    if (i2c_wire->available() != len) return -1;

    for (uint16_t i = 0; i < len; i++) {
        data[i] = i2c_wire->read();
    }
    return 0;
}

int platform_i2c_available(const uint8_t dev_addr) {
    if (!i2c_wire) return -1;

    // Request 2 bytes to check if device responds (UBX packet length bytes)
    i2c_wire->requestFrom(static_cast<int>(dev_addr), 2);
    const int available = i2c_wire->available();

    // Read and discard the bytes we requested
    while (i2c_wire->available()) {
        i2c_wire->read();
    }

    return available;
}

// UART Implementation
int platform_uart_write(const uint8_t *data, const uint16_t len) {
    if (hw_serial) {
        return hw_serial->write(data, len) == len ? 0 : -1;
    } else if (sw_serial) {
        return sw_serial->write(data, len) == len ? 0 : -1;
    }
    return -1;
}

int platform_uart_read(uint8_t *data, const uint16_t len) {
    if (hw_serial) {
        uint16_t bytes_read = 0;
        while (bytes_read < len && hw_serial->available()) {
            data[bytes_read++] = hw_serial->read();
        }
        return bytes_read;
    } else if (sw_serial) {
        uint16_t bytes_read = 0;
        while (bytes_read < len && sw_serial->available()) {
            data[bytes_read++] = sw_serial->read();
        }
        return bytes_read;
    }
    return -1;
}

int platform_uart_available(void) {
    if (hw_serial) {
        return hw_serial->available();
    } else if (sw_serial) {
        return sw_serial->available();
    }
    return -1;
}

void platform_uart_flush(void) {
    if (hw_serial) {
        hw_serial->flush();
    } else if (sw_serial) {
        sw_serial->flush();
    }
}

// Common delay function
void platform_delay_ms(const uint32_t ms) {
    delay(ms);
}

void platform_delay_us(const uint32_t us) {
    delayMicroseconds(us);
}
    
// Utility functions for Arduino-specific features
uint32_t platform_millis(void) {
    return millis();
}

uint32_t platform_micros(void) {
    return micros();
}

}