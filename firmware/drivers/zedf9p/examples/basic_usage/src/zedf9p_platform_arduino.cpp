/**
 * @file zedf9p_platform_arduino.cpp
 * @brief Platform abstraction layer implementation for ZEDF9P GNSS driver (Arduino)
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-19
 *
 * This file should be placed in the src folder and contains platform-specific
 * implementations for Arduino. Only one platform implementation should be
 * compiled based on your target platform.
 */

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include "zedf9p_platform.h"
}

extern "C" {

// I2C interface pointer - defaults to Wire but can be changed
static TwoWire *i2c_wire = &Wire;

// UART interface pointer - defaults to Serial3 but can be changed
static HardwareSerial *uart_serial = &Serial1;

// I2C implementation
int platform_i2c_write(const uint8_t dev_addr, const uint8_t *data, const uint16_t len) {
    if (data == nullptr || len == 0U) {
        return -1;
    }

    i2c_wire->beginTransmission(dev_addr);

    for (uint16_t i = 0; i < len; i++) {
        i2c_wire->write(data[i]);
    }

    const int result = i2c_wire->endTransmission();
    return (result == 0) ? 0 : -1;
}

int platform_i2c_read(const uint8_t dev_addr, uint8_t *data, const uint16_t len) {
    if (data == nullptr || len == 0U) {
        return -1;
    }

    // Small delay before reading to ensure data is ready
    delay(2);

    const uint8_t bytes_received = i2c_wire->requestFrom(static_cast<int>(dev_addr), static_cast<int>(len));

    if (bytes_received != len) {
        return -1;  // Didn't receive expected number of bytes
    }

    for (uint16_t i = 0; i < len; i++) {
        if (i2c_wire->available()) {
            data[i] = static_cast<uint8_t>(i2c_wire->read());
        } else {
            return -1;  // No more data available
        }
    }

    return 0;
}

// UART implementation
int platform_uart_write(const uint8_t *data, const uint16_t len) {
    if (data == nullptr || len == 0U) {
        return -1;
    }

    const size_t bytes_written = uart_serial->write(data, len);
    return (bytes_written == len) ? 0 : -1;
}

int platform_uart_read(uint8_t *data, const uint16_t len) {
    if (data == nullptr || len == 0U) {
        return -1;
    }

    // Check if enough data is available
    if (uart_serial->available() < static_cast<int>(len)) {
        return -1;  // Not enough data available
    }

    for (uint16_t i = 0; i < len; i++) {
        const int byte_read = uart_serial->read();
        if (byte_read == -1) {
            return -1;  // No data available
        }
        data[i] = static_cast<uint8_t>(byte_read);
    }

    return 0;
}

// Timing implementation
void platform_delay_ms(const uint32_t ms) {
    delay(ms);
}

uint32_t platform_get_millis(void) {
    return millis();
}

// Interface configuration functions

/**
 * @brief Set the I2C interface to use
 * @param wire_interface Pointer to TwoWire interface (e.g., &Wire, &Wire1)
 */
void platform_set_i2c_interface(TwoWire *wire_interface) {
    if (wire_interface != nullptr) {
        i2c_wire = wire_interface;
    }
}

/**
 * @brief Set the UART interface to use
 * @param serial_interface Pointer to HardwareSerial interface (e.g., &Serial1, &Serial2)
 */
void platform_set_uart_interface(HardwareSerial *serial_interface) {
    if (serial_interface != nullptr) {
        uart_serial = serial_interface;
    }
}

}