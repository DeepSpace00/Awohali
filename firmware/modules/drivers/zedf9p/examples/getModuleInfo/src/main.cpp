/**
 * @file main.cpp
 * @brief Arduino example sketch for ZEDF9P GNSS module driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-22
 *
 * This example demonstrates how to:
 * - Initialize the ZEDF9P driver
 * - Poll for MON-VER message
 * - Display firmware version information
 */

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include "zedf9p.h"
#include "zedf9p_platform.h"
}

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Communication interface selection
bool use_i2c = true;  // Set to false to use UART instead

// I2C Configuration
TwoWire* selected_i2c;
//unsigned long i2c_freq = 100000;   // I2C frequency in Hz

// UART Configuration
HardwareSerial* selected_uart;
unsigned long uart_baudrate = 38400;       // UART baudrate
uint32_t uart_config = SERIAL_8N1;         // UART config (8N1, 8E1, etc.)

// ============================================================================

// Function declarations
bool initialize_gps();
bool get_version_info();
void print_version_info(const zedf9p_mon_ver_t *version);
void parse_extension_info(const zedf9p_mon_ver_t *version);

// ZEDF9P driver instance
zedf9p_t gps;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("ZEDF9P Version Information Example");
    Serial.println("==================================");


    // Configure hardware interfaces
    selected_i2c = &Wire;        // Change to &Wire1, &Wire2, etc. as needed
    selected_uart = &Serial3;    // Change to &Serial2, &Serial3, etc. as needed

    delay(100);

    // Initialize communication interfaces
    if (use_i2c) {
        platform_set_i2c_interface(selected_i2c);
        selected_i2c->begin();
        //selected_i2c->setClock(i2c_freq);
    } else {
        platform_set_uart_interface(selected_uart);
        selected_uart->begin(uart_baudrate, uart_config);
        Serial.println("Using UART communication");
    }

    // Initialize the ZEDF9P driver
    if (!initialize_gps()) {
        Serial.println("Failed to initialize ZEDF9P!");
        Serial.println("Check connections and power supply");
        while (1) {
            delay(1000);
            Serial.println("System halted - check connections");
        }
    }

    Serial.println("ZEDF9P driver initialized successfully!");
    Serial.println();

    // Get and display version information
    if (get_version_info()) {
        Serial.println("Version information retrieved successfully!");
    } else {
        Serial.println("Failed to get version information");
    }

    Serial.println();
    Serial.println("Version check complete. Reset to run again.");
}

void loop() {
    // Nothing to do in loop for this example
    delay(1000);
}

bool initialize_gps() {
    // Define the interface functions
    zedf9p_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .uart_write = platform_uart_write,
        .uart_read = platform_uart_read,
        .delay_ms = platform_delay_ms,
        .get_millis = platform_get_millis
    };

    // Initialize the driver
    zedf9p_interface_type_t interface = use_i2c ? ZEDF9P_INTERFACE_I2C : ZEDF9P_INTERFACE_UART;
    zedf9p_status_t status = zedf9p_init(&gps, interface, ZEDF9P_I2C_ADDR, io);

    if (status != ZEDF9P_OK) {
        Serial.print("Driver initialization failed: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    return true;
}

bool get_version_info() {
    Serial.println("Requesting version information...");

    // Add debugging before the poll
    Serial.println("About to send MON-VER poll command...");
    //Serial.flush();  // Ensure the message is sent

    // Poll for MON-VER message
    zedf9p_status_t status = zedf9p_poll_mon_ver(&gps);

    // This line should print - if it doesn't, the poll function is hanging
    Serial.print("Poll result: ");
    Serial.println(zedf9p_status_error(status));

    if (status != ZEDF9P_OK) {
        Serial.print("Failed to poll MON-VER: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    Serial.println("MON-VER poll sent successfully");

    // Wait for response with timeout
    unsigned long start_time = millis();
    const unsigned long timeout_ms = 5000;  // 5 second timeout

    Serial.print("Waiting for response");

    uint32_t process_count = 0;
    uint32_t last_dot_time = millis();

    while (!zedf9p_is_mon_ver_available(&gps)) {
        // Process incoming data
        zedf9p_status_t process_status = zedf9p_process_data(&gps);
        if (process_status != ZEDF9P_OK && process_status != ZEDF9P_ERR_NO_DATA) {
            Serial.println();
            Serial.print("Error processing data: ");
            Serial.println(zedf9p_status_error(process_status));
        }

        process_count++;

        // Check timeout
        if (millis() - start_time > timeout_ms) {
            Serial.println();
            Serial.println("Timeout waiting for MON-VER response");
            Serial.print("Data processing attempts: ");
            Serial.println(process_count);
            return false;
        }

        // Print progress dots every 500ms
        if (millis() - last_dot_time >= 500) {
            Serial.print(".");
            last_dot_time = millis();
        }

        delay(10);
    }

    Serial.println();
    Serial.print("Response received after ");
    Serial.print(millis() - start_time);
    Serial.print("ms (");
    Serial.print(process_count);
    Serial.println(" data processing attempts)");

    // Get the version data
    zedf9p_mon_ver_t version_info;
    status = zedf9p_get_mon_ver(&gps, &version_info);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to get MON-VER data: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    // Display the version information
    print_version_info(&version_info);

    return true;
}

void print_version_info(const zedf9p_mon_ver_t *version) {
    Serial.println("ZEDF9P Version Information:");
    Serial.println("===========================");

    // Software version
    Serial.print("Software Version: ");
    Serial.println(version->sw_version);

    // Hardware version
    Serial.print("Hardware Version: ");
    Serial.println(version->hw_version);

    // ROM version (if available)
    if (strlen(version->rom_version) > 0) {
        Serial.print("ROM Version:      ");
        Serial.println(version->rom_version);
    }

    // Extensions
    if (version->num_extensions > 0) {
        Serial.println();
        Serial.println("Extensions:");
        for (uint8_t i = 0; i < version->num_extensions; i++) {
            Serial.print("  ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.println(version->extensions[i]);
        }
    }

    Serial.println();

    // Parse and display key information from extensions
    parse_extension_info(version);
}

void parse_extension_info(const zedf9p_mon_ver_t *version) {
    Serial.println("Parsed Information:");
    Serial.println("===================");

    // Look for specific extension types
    for (uint8_t i = 0; i < version->num_extensions; i++) {
        const char *ext = version->extensions[i];

        // Protocol version
        if (strncmp(ext, "PROTVER=", 8) == 0) {
            Serial.print("Protocol Version: ");
            Serial.println(&ext[8]);
        }

        // Module type
        else if (strncmp(ext, "MOD=", 4) == 0) {
            Serial.print("Module Type:      ");
            Serial.println(&ext[4]);
        }

        // GNSS constellations
        else if (strstr(ext, "GPS") != NULL || strstr(ext, "GLO") != NULL ||
                 strstr(ext, "GAL") != NULL || strstr(ext, "BDS") != NULL) {
            Serial.print("GNSS Support:     ");
            Serial.println(ext);
        }

        // Additional capabilities
        else if (strstr(ext, "SBAS") != NULL || strstr(ext, "QZSS") != NULL ||
                 strstr(ext, "L2C") != NULL || strstr(ext, "L5") != NULL) {
            Serial.print("Capabilities:     ");
            Serial.println(ext);
        }

        // Firmware date/build info
        else if (strstr(ext, "EXT CORE") != NULL) {
            Serial.print("Core Version:     ");
            Serial.println(ext);
        }
    }
}