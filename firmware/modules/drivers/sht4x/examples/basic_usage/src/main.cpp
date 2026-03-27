/**
 * @file main.cpp
 * @brief Arduino test program for SHT4x humidity and temperature sensor driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-15
 *
 * This test program demonstrates comprehensive functionality of the SHT4x driver
 * including initialization, configuration, and measurements.
 */

#include <Arduino.h>
#include <Wire.h>
#include <sht4x.h>
#include <sht4x_platform.h>

sht4x_t sht;
sht4x_measurements_t env;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    while(!Serial) delay(10);

    Serial.println("=== SHT4X Humidity & Temperature Sensor Test ===");
    Serial.println("");

    // Define the IO interface
    constexpr sht4x_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .delay_ms = platform_delay_ms
    };

    // Initialize the SHT4x
    Serial.println("Initializing SHT4x... ");
    sht4x_status_t status = sht4x_init(&sht, SHT4X_I2C_ADDR, io);

    if (status != SHT4X_OK) {
        Serial.println("Failed!");
        Serial.print("Error: ");
        Serial.println(sht4x_stat_error(status));
        while (true) delay(10);
    }
    Serial.println("SUCCESS!");
}

void loop() {
    sht4x_status_t status = sht4x_read_measurements(&sht, &env);

    if (status != SHT4X_OK) {
        Serial.println("Failed to read sensor data.");
        Serial.print("Error: ");
        Serial.println(sht4x_stat_error(status));
    } else {
        char buffer[32];
        sprintf(buffer, "Temp: %0.2f C, RH: %0.2f %%\n", env.temperature_c, env.humidity_rh);
        Serial.print(buffer);
    }

    delay(1000);
}