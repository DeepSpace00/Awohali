/**
* @file main.cpp
 * @brief Arduino test program for BQ76905 Battery Monitor and Protector driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-11
 *
 * This test program demonstrates comprehensive functionality of the BQ76905 driver
 * including initialization, configuration, status monitoring, and ADC measurements.
 */

#include <Arduino.h>
#include <Wire.h>
#include <bq76905.h>
#include <bq76905_platform.h>

bq76905_t bq;
bq76905_measurements_t measurements;
bq76905_alarm_status_t alarm_status;
bq76905_battery_status_t battery_status;
bq76905_active_cells_t active_cells;

// Test configuration values
constexpr int TEST_ACTIVE_CELLS = 2;        // 2S battery configuration

void setup() {
    Serial.begin(115200);
    Wire.begin();

    delay(100);

    while (!Serial) delay(10);

    Serial.println("=== BQ76905 Battery Monitor Test ===");
    Serial.println();

    // Define the IO interface
    constexpr bq76905_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .delay_ms = platform_delay_ms,
    };

    Serial.println("Initializing BQ76905... ");
    bq76905_status_t status = bq76905_init(&bq, BQ76905_I2C_DEFAULT_ADDRESS, io);

    if (status != BQ76905_OK) {
        Serial.println("FAILED!");
        Serial.print("Error: ");
        Serial.println(bq76905_stat_error(status));
        while(true) delay(10);
    }
    Serial.println("SUCCESS!");

    // Test basic configuration
    Serial.println("\n--- Testing Basic Configuration ---");

    // Set cells to balance

}

void loop() {

}