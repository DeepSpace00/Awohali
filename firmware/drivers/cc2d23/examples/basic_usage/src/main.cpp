/**
 * @file main.cpp
 * @brief Arduino test program for CC2Dxx humidity and temperature sensor driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-21
 *
 * This test program demonstrates comprehensive functionality of the CC2Dxx driver
 * including initialization, configuration, status monitoring, and measurements.
 */

#include <Arduino.h>
#include <Wire.h>
#include <cc2dxx.h>
#include <cc2dxx_platform.h>

cc2dxx_t cc;
cc2dxx_measurements_t measurements;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    delay(100);

    while (!Serial) delay(10);

    Serial.println("=== CC2Dxx Humd & Temp Sensor Test ===");
    Serial.println();

    // Define the IO interface
    cc2dxx_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .delay_ms = platform_delay_ms,
    };

    // Initialize the CC2Dxx
    Serial.print("Initializing CC2Dxx... ");
    cc2dxx_status_t status = cc2dxx_init(&cc, CC2DXX_I2C_DEFAULT_ADDRESS, io);

    if (status != CC2DXX_OK) {
        Serial.println("FAILED!");
        Serial.print("Error: ");
        Serial.println(ccd2dxx_stat_error(status));
        while (true) delay(10);
    }
    Serial.println("SUCCESS!");

    delay(100);
}

void loop() {
    cc2dxx_status_t measurements_status = cc2dxx_get_measurements(&cc, &measurements);

    if (measurements_status != CC2DXX_OK) {
        char buffer[80];
        sprintf(buffer, "Humidity: %.2f %%, Temperature: %.2f °C", measurements.humidity, measurements.temperature);
        Serial.println(buffer);
    }
    else {
        Serial.println("Sensor read failed.");
        Serial.print("Error: ");
        Serial.println(ccd2dxx_stat_error(measurements_status));
    }

    delay(1000);
}