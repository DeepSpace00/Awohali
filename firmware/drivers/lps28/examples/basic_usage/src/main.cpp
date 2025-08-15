/*!
 * @file main.cpp
 * @brief Arduino example sketch for LPS28DFW pressure and temperature sensor
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-14
 * 
 * This example demonstrates how to use the LPS28DFW driver with Arduino.
 */

#include <Arduino.h>
#include <Wire.h>
#include <lps28.h>
#include <lps28_platform.h>

lps28_t lps;
lps28_status_reg_t sensor_status;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    while (!Serial) delay(10);

    delay(100);

    while(!Serial) delay(10);

    Serial.println("LPS28DFW Basic Usage");

    // Define the IO pointers
    lps28_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .delay_ms = platform_delay_ms
    };

    // Initialize the sensor
    lps28_status_t status = lps28_init(&lps, LPS28_I2C_ADDR_1, io);

    if (status != LPS28_OK) {
        Serial.println("Failed to init LPS28DFW");
        Serial.println(lps28_stat_error(status));
        while(1) delay(10);
    }
    else {
        // Verify WHO_AM_I
        uint8_t who_am_i;
        lps28_who_am_i(&lps, &who_am_i);
        Serial.print("LPS28DFW WHO_AM_I: 0x");
        Serial.println(who_am_i, HEX);
    }

    delay(10);

    // Configure sensor for continuous measurement at 10Hz with 16 sample averaging
    status = lps28_ctrl_reg1(&lps, LPS28_ODR_10_HZ, LPS28_AVG_16);
    if (status != LPS28_OK) {
        Serial.println("Failed to configure CTRL_REG1");
        Serial.println(lps28_stat_error(status));
        while(1) delay(10);
    }

    // Optional: Set reference pressure (e.g., sea level pressure)
    //lps28_reference_pressure(&lps, 1013.25f);

    // Optional: Set pressure offset for calibration
    //lps28_pressure_offset(&lps, 0.0f);

    // Optional: Perform soft reset
    //lps28_ctrl_reg2(&lps, false, true, false);
    //delay(100);

    Serial.println("Starting measurements...");
}

void loop() {
    // Check sensor status
    lps28_status_t status = lps28_status(&lps, &sensor_status);
    if (status == LPS28_OK) {
        // Check if new data is available
        if (sensor_status.pressure_data_available && sensor_status.temperature_data_available) {
            // Read pressure and temperature
            float pressure, temperature;
            
            lps28_status_t pressure_status = lps28_read_pressure(&lps, &pressure);
            lps28_status_t temp_status = lps28_read_temperature(&lps, &temperature);
            
            if (pressure_status == LPS28_OK && temp_status == LPS28_OK) {
                char buffer[80];
                sprintf(buffer, "Pressure: %.2f hPa, Temperature: %.2f °C", pressure, temperature);
                Serial.println(buffer);
            }
            else {
                Serial.println("Sensor read failed.");
                if (pressure_status != LPS28_OK) {
                    Serial.print("Pressure error: ");
                    Serial.println(lps28_stat_error(pressure_status));
                }
                if (temp_status != LPS28_OK) {
                    Serial.print("Temperature error: ");
                    Serial.println(lps28_stat_error(temp_status));
                }
            }
        }
    }
    else {
        Serial.println("Status read failed.");
        Serial.println(lps28_stat_error(status));
    }
    
    delay(1000); // Wait 500ms between readings (sensor runs at 10Hz)
}