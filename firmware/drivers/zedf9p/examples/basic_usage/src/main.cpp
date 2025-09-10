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
TwoWire* selected_i2c;              // Will be set in setup()
unsigned long i2c_frequency = 400000;   // I2C frequency in Hz

// UART Configuration
HardwareSerial* selected_uart;      // Will be set in setup()
unsigned long uart_baudrate = 38400;       // UART baudrate
uint32_t uart_config = SERIAL_8N1;         // UART config (8N1, 8E1, etc.)

// GNSS Configuration
bool enable_high_precision = true;
bool enable_rawx = true;
bool configure_as_rover = false;

// ============================================================================

// Function declarations
bool initialize_gps();
bool configure_gps();
bool configure_gnss_signals();
bool configure_rtk_rover();
void print_data_header();
void print_position_data();
void print_pvt_data();
void print_hppos_data();
void print_status_info();
String get_fix_type_string(uint8_t fix_type);
void pvt_callback(const ubx_message_t *message, void *user_data);
void rawx_callback(const ubx_message_t *message, void *user_data);

// ZEDF9P driver instance
zedf9p_t gps;

// Data structures for storing measurements
zedf9p_nav_pvt_t pvt_data;
zedf9p_nav_hpposllh_t hppos_data;
zedf9p_rawx_t rawx_data;

// Timing variables
unsigned long last_position_print = 0;
unsigned long last_status_print = 0;
constexpr unsigned long POSITION_INTERVAL = 1000;  // Print position every 1 second
constexpr unsigned long STATUS_INTERVAL = 5000;    // Print status every 5 seconds

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("ZEDF9P GNSS Basic Usage Example");
    Serial.println("================================");

    // Configure hardware interfaces - MODIFY THESE FOR YOUR SETUP
    selected_i2c = &Wire;        // Change to &Wire1, &Wire2, etc. as needed
    selected_uart = &Serial3;    // Change to &Serial2, &Serial3, etc. as needed

    // Initialize communication interfaces
    if (use_i2c) {
        platform_set_i2c_interface(selected_i2c);
        selected_i2c->begin();
        selected_i2c->setClock(i2c_frequency);
        Serial.println("Using I2C communication");
    } else {
        platform_set_uart_interface(selected_uart);
        selected_uart->begin(uart_baudrate, uart_config);
        Serial.println("Using UART communication");
    }

    delay(100);

    // Initialize the ZEDF9P driver
    if (!initialize_gps()) {
        Serial.println("Failed to initialize ZEDF9P!");
        while (true) {
            delay(1000);
            Serial.println("System halted - check connections");
        }
    }

    // Configure the GNSS module
    if (!configure_gps()) {
        Serial.println("Failed to configure ZEDF9P!");
        while (true) delay(1000);
    }

    Serial.println("ZEDF9P initialized and configured successfully!");
    Serial.println("Waiting for GNSS fix...");
    Serial.println();

    // Print header for position data
    print_data_header();
}

void loop() {
    // Process incoming GNSS data
    zedf9p_process_data(&gps);

    const unsigned long current_time = millis();

    // Print position data periodically
    if (current_time - last_position_print >= POSITION_INTERVAL) {
        print_position_data();
        last_position_print = current_time;
    }

    // Print status information periodically
    if (current_time - last_status_print >= STATUS_INTERVAL) {
        print_status_info();
        last_status_print = current_time;
    }

    // Small delay to prevent overwhelming the processor
    delay(10);
}

bool initialize_gps() {
    // Define the interface functions
    constexpr zedf9p_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .uart_write = platform_uart_write,
        .uart_read = platform_uart_read,
        .delay_ms = platform_delay_ms,
        .get_millis = platform_get_millis
    };

    // Initialize the driver
    const zedf9p_interface_type_t interface = use_i2c ? ZEDF9P_INTERFACE_I2C : ZEDF9P_INTERFACE_UART;
    const zedf9p_status_t status = zedf9p_init(&gps, interface, ZEDF9P_I2C_ADDR, io);

    if (status != ZEDF9P_OK) {
        Serial.print("Driver initialization failed: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    return true;
}

bool configure_gps() {
    // Set measurement rate to 1Hz (1000ms)
    Serial.println("Setting measurement rate to 1Hz...");
    zedf9p_status_t status = zedf9p_set_measurement_rate(&gps, 1000, 1);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to set measurement rate: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    // Set dynamic model to automotive (good for ground vehicles)
    Serial.println("Setting dynamic model to automotive...");
    status = zedf9p_set_dynamic_model(&gps, DYN_MODEL_AUTOMOTIVE);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to set dynamic model: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    // Enable PVT messages
    Serial.println("Enabling PVT messages...");
    status = zedf9p_set_message_rate(&gps, UBX_CLASS_NAV, UBX_NAV_PVT, 1);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to enable PVT: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    // Configure GNSS constellations
    if (!configure_gnss_signals()) {
        return false;
    }

    // Configure high precision positioning if enabled
    if (enable_high_precision) {
        Serial.println("Enabling high precision positioning...");
        status = zedf9p_set_message_rate(&gps, UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, 1);
        if (status != ZEDF9P_OK) {
            Serial.print("Failed to enable HPPOSLLH: ");
            Serial.println(zedf9p_status_error(status));
            return false;
        }
    }

    // Configure RAWX data if enabled (for RTK)
    if (enable_rawx) {
        Serial.println("Enabling RAWX data for RTK...");
        status = zedf9p_set_message_rate(&gps, UBX_CLASS_RXM, UBX_RXM_RAWX, 1);
        if (status != ZEDF9P_OK) {
            Serial.print("Failed to enable RAWX: ");
            Serial.println(zedf9p_status_error(status));
            return false;
        }
    }

    // Configure RTK mode if enabled
    if (configure_as_rover && enable_rawx) {
        if (!configure_rtk_rover()) {
            return false;
        }
    }

    delay(1000);  // Allow configuration to settle
    return true;
}

bool configure_gnss_signals() {
    Serial.println("Configuring GNSS signals...");

    // Configure multi-constellation GNSS
    constexpr zedf9p_gnss_config_t gnss_config = {
        .gps_enabled = true,
        .gps_l1ca_enabled = true,
        .gps_l2c_enabled = true,          // L2C for better accuracy
        .sbas_enabled = true,             // WAAS/EGNOS corrections
        .sbas_l1ca_enabled = true,
        .galileo_enabled = true,          // European constellation
        .galileo_e1_enabled = true,
        .galileo_e5b_enabled = true,
        .beidou_enabled = false,          // Disable for cleaner solution
        .beidou_b1_enabled = false,
        .beidou_b2_enabled = false,
        .qzss_enabled = true,             // Good for Asia-Pacific
        .qzss_l1ca_enabled = true,
        .qzss_l1s_enabled = false,
        .qzss_l2c_enabled = true,
        .glonass_enabled = true,          // Russian constellation
        .glonass_l1_enabled = true,
        .glonass_l2_enabled = true
    };

    const zedf9p_status_t status = zedf9p_config_gnss_signals(&gps, &gnss_config);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to configure GNSS signals: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    return true;
}

bool configure_rtk_rover() {
    Serial.println("Configuring as RTK rover...");

    // Enable satellite messages for RTK
    zedf9p_status_t status = zedf9p_set_message_rate(&gps, UBX_CLASS_NAV, UBX_NAV_SAT, 1);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to enable SAT messages: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    // Enable relative positioning messages
    status = zedf9p_set_message_rate(&gps, UBX_CLASS_NAV, UBX_NAV_RELPOSNED, 1);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to enable RELPOSNED: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    Serial.println("RTK rover configuration complete");
    Serial.println("Note: Connect RTCM correction stream for RTK operation");

    return true;
}

void print_data_header() {
    Serial.println("Time(ms)   | Fix | Sats | Latitude      | Longitude     | Height(m) | Accuracy(m)");
    Serial.println("-----------|-----|------|---------------|---------------|-----------|------------");
}

void print_position_data() {
    // Check if PVT data is available
    if (zedf9p_is_pvt_available(&gps)) {
        zedf9p_status_t status = zedf9p_get_pvt(&gps, &pvt_data);
        if (status == ZEDF9P_OK) {
            print_pvt_data();
        }
    }

    // Print high precision data if available and enabled
    if (enable_high_precision && zedf9p_is_hpposllh_available(&gps)) {
        zedf9p_status_t status = zedf9p_get_hpposllh(&gps, &hppos_data);
        if (status == ZEDF9P_OK) {
            print_hppos_data();
        }
    }
}

void print_pvt_data() {
    // Convert coordinates to degrees
    const double latitude = pvt_data.lat / 1e7;
    const double longitude = pvt_data.lon / 1e7;
    const double height = pvt_data.height / 1000.0;  // mm to m
    const double h_acc = pvt_data.h_acc / 1000.0;    // mm to m

    // Get fix type string
    const String fix_type = get_fix_type_string(pvt_data.fix_type);

    // Print formatted data
    char buffer[200];
    sprintf(buffer, "%10lu | %3s | %4d | %13.7f | %13.7f | %9.3f | %10.3f",
            millis(),
            fix_type.c_str(),
            pvt_data.num_sv,
            latitude,
            longitude,
            height,
            h_acc);

    Serial.println(buffer);

    // Print additional useful information
    if (pvt_data.fix_type >= 3) {  // 3D fix or better
        const double speed_kmh = (pvt_data.g_speed / 1000.0) * 3.6;  // mm/s to km/h
        const double heading = pvt_data.head_mot / 1e5;  // 1e-5 deg to deg

        Serial.print("   Speed: ");
        Serial.print(speed_kmh, 1);
        Serial.print(" km/h, Heading: ");
        Serial.print(heading, 1);
        Serial.println("°");
    }
}

void print_hppos_data() {
    // High precision coordinates
    const double lat_hp = (pvt_data.lat / 1e7) + (hppos_data.lat_hp / 1e9);
    const double lon_hp = (pvt_data.lon / 1e7) + (hppos_data.lon_hp / 1e9);
    const double height_hp = (pvt_data.height / 1000.0) + (hppos_data.height_hp / 10000.0);

    Serial.print("   HP: ");
    Serial.print(lat_hp, 9);
    Serial.print(", ");
    Serial.print(lon_hp, 9);
    Serial.print(", ");
    Serial.print(height_hp, 4);
    Serial.println(" m");
}

void print_status_info() {
    Serial.println("\n--- GNSS Status ---");

    if (zedf9p_is_pvt_available(&gps)) {
        zedf9p_get_pvt(&gps, &pvt_data);

        Serial.print("Fix Type: ");
        Serial.println(get_fix_type_string(pvt_data.fix_type));

        Serial.print("Satellites: ");
        Serial.println(pvt_data.num_sv);

        Serial.print("Time: ");
        Serial.print(pvt_data.hour);
        Serial.print(":");
        Serial.print(pvt_data.min);
        Serial.print(":");
        Serial.print(pvt_data.sec);
        Serial.print(" UTC ");
        Serial.print(pvt_data.day);
        Serial.print("/");
        Serial.print(pvt_data.month);
        Serial.print("/");
        Serial.println(pvt_data.year);

        if (pvt_data.fix_type >= 3) {
            Serial.print("PDOP: ");
            Serial.println(pvt_data.p_dop / 100.0);
        }
    } else {
        Serial.println("No PVT data available");
    }

    // Print RAWX status if enabled
    if (enable_rawx && zedf9p_is_rawx_available(&gps)) {
        zedf9p_get_rawx(&gps, &rawx_data);
        Serial.print("RAWX measurements: ");
        Serial.println(rawx_data.num_meas);
    }

    Serial.println("------------------\n");
}

String get_fix_type_string(uint8_t fix_type) {
    switch (fix_type) {
        case 0: return "NoFix";
        case 1: return "DR";      // Dead reckoning
        case 2: return "2D";
        case 3: return "3D";
        case 4: return "GNSS+DR";
        case 5: return "Time";
        default: return "Unk";
    }
}

// Optional: Callback functions for real-time data processing
void pvt_callback(const ubx_message_t *message, void *user_data) {
    // This function is called automatically when PVT data is received
    // Useful for time-critical applications
    Serial.println("PVT data received via callback!");
}

void rawx_callback(const ubx_message_t *message, void *user_data) {
    // This function is called automatically when RAWX data is received
    // Useful for RTK applications requiring immediate processing
    Serial.println("RAWX data received via callback!");
}

// Uncomment the following lines in setup() to enable callbacks:
// zedf9p_register_pvt_callback(&gps, pvt_callback, NULL);
// zedf9p_register_rawx_callback(&gps, rawx_callback, NULL);