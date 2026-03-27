/*!
 * @file zedf9p_rawx_example.ino
 * @brief Arduino example to get RAWX data from ZEDF9P GNSS module
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-19
 *
 * This example demonstrates how to:
 * - Configure ZEDF9P for RAWX data output
 * - Read raw measurement data for RTK processing
 * - Display satellite measurement information
 * - Save data for post-processing (optional)
 *
 * Similar to SparkFun Example23_getRXMRAWX.ino
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
bool use_i2c = false;  // Set to true to use I2C instead of UART

// I2C Configuration
TwoWire* selected_i2c;
unsigned long i2c_frequency = 400000;   // I2C frequency in Hz

// UART Configuration
HardwareSerial* selected_uart;
unsigned long uart_baudrate = 38400;       // UART baudrate
uint32_t uart_config = SERIAL_8N1;         // UART config (8N1, 8E1, etc.)

// RAWX Configuration
bool enable_automatic_rawx = true;         // Enable automatic RAWX messages
bool log_to_serial = true;                // Log RAWX data to Serial
bool show_satellite_details = true;       // Show individual satellite measurements

// ============================================================================

// Function declarations
bool initialize_gps();
bool configure_rawx();
void print_statistics();
void rawx_callback(const ubx_message_t *message, void *user_data);
void print_rawx_header(const zedf9p_rawx_t *rawx);
void print_satellite_measurement(const zedf9p_rawx_meas_t *meas, uint8_t sat_num);
void print_rawx_summary(const zedf9p_rawx_t *rawx);
String get_gnss_name(uint8_t gnss_id);
String get_signal_name(uint8_t gnss_id, uint8_t sig_id);

// ZEDF9P driver instance
zedf9p_t gps;

// Statistics
unsigned long rawx_count = 0;
unsigned long last_rawx_time = 0;
unsigned long start_time = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("ZEDF9P RAWX Data Example");
    Serial.println("========================");
    Serial.println("Similar to SparkFun Example23_getRXMRAWX.ino");
    Serial.println();

    // Configure hardware interfaces
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
        Serial.println("Check connections and power supply");
        while (1) {
            delay(1000);
            Serial.println("System halted - check connections");
        }
    }

    Serial.println("ZEDF9P driver initialized successfully!");

    // Configure RAWX data collection
    if (!configure_rawx()) {
        Serial.println("Failed to configure RAWX!");
        while (1) delay(1000);
    }

    Serial.println("RAWX configuration complete!");
    Serial.println("Collecting raw measurement data...");
    Serial.println("Press Ctrl+C to stop data collection");
    Serial.println();

    start_time = millis();
}

void loop() {
    // Process incoming GNSS data
    zedf9p_process_data(&gps);

    // Check for new RAWX data (polling method)
    if (zedf9p_is_rawx_available(&gps)) {
        zedf9p_rawx_t rawx_data;
        zedf9p_status_t status = zedf9p_get_rawx(&gps, &rawx_data);

        if (status == ZEDF9P_OK) {
            rawx_count++;
            last_rawx_time = millis();

            if (log_to_serial) {
                print_rawx_header(&rawx_data);

                if (show_satellite_details) {
                    // Print details for each satellite measurement
                    for (uint8_t i = 0; i < rawx_data.num_meas; i++) {
                        print_satellite_measurement(&rawx_data.meas[i], i + 1);
                    }
                }

                print_rawx_summary(&rawx_data);
                Serial.println("----------------------------------------");
            }
        }
    }

    // Print statistics every 30 seconds
    static unsigned long last_stats_time = 0;
    if (millis() - last_stats_time >= 30000) {
        print_statistics();
        last_stats_time = millis();
    }

    delay(10);  // Small delay to prevent overwhelming the processor
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

bool configure_rawx() {
    zedf9p_status_t status;

    // Set measurement rate to 1Hz for RAWX data
    Serial.println("Setting measurement rate to 1Hz...");
    status = zedf9p_set_measurement_rate(&gps, 1000, 1);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to set measurement rate: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    // Enable RAWX messages
    Serial.println("Enabling RAWX messages...");
    status = zedf9p_set_message_rate(&gps, UBX_CLASS_RXM, UBX_RXM_RAWX, 1);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to enable RAWX: ");
        Serial.println(zedf9p_status_error(status));
        return false;
    }

    // Enable satellite status messages for additional info
    Serial.println("Enabling satellite status messages...");
    status = zedf9p_set_message_rate(&gps, UBX_CLASS_NAV, UBX_NAV_SAT, 1);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to enable SAT: ");
        Serial.println(zedf9p_status_error(status));
        // Don't return false - this is not critical
    }

    // Configure GNSS signals for better RAWX data
    zedf9p_gnss_config_t gnss_config = {
        .gps_enabled = true,
        .gps_l1ca_enabled = true,
        .gps_l2c_enabled = true,          // L2C for dual-frequency
        .sbas_enabled = false,            // Disable SBAS for cleaner RAWX
        .sbas_l1ca_enabled = false,
        .galileo_enabled = true,          // European constellation
        .galileo_e1_enabled = true,
        .galileo_e5b_enabled = true,
        .beidou_enabled = false,          // Keep it simple for now
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

    Serial.println("Configuring GNSS signals...");
    status = zedf9p_config_gnss_signals(&gps, &gnss_config);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to configure GNSS signals: ");
        Serial.println(zedf9p_status_error(status));
        // Don't return false - use defaults
    }

    // Register callback for automatic RAWX processing
    if (enable_automatic_rawx) {
        Serial.println("Registering RAWX callback...");
        status = zedf9p_register_rawx_callback(&gps, rawx_callback, NULL);
        if (status != ZEDF9P_OK) {
            Serial.print("Failed to register callback: ");
            Serial.println(zedf9p_status_error(status));
            // Continue without callback
        }
    }

    delay(2000);  // Allow configuration to settle
    Serial.println("Waiting for RAWX data...");

    return true;
}

void rawx_callback(const ubx_message_t *message, void *user_data) {
    // This callback is called automatically when RAWX data arrives
    // You can process the raw message here for real-time applications
    Serial.println("RAWX callback triggered!");

    // Note: The actual RAWX data parsing is done in the driver
    // and will be available via zedf9p_get_rawx()
}

void print_rawx_header(const zedf9p_rawx_t *rawx) {
    Serial.print("RAWX Data - TOW: ");
    Serial.print(rawx->rc_tow, 3);
    Serial.print("s, Week: ");
    Serial.print(rawx->week);
    Serial.print(", Measurements: ");
    Serial.print(rawx->num_meas);
    Serial.print(", Status: 0x");
    Serial.println(rawx->rec_stat, HEX);
}

void print_satellite_measurement(const zedf9p_rawx_meas_t *meas, uint8_t sat_num) {
    Serial.print("  [");
    Serial.print(sat_num);
    Serial.print("] ");
    Serial.print(get_gnss_name(meas->gnss_id));
    Serial.print(" ");
    Serial.print(meas->sv_id);
    Serial.print(" (");
    Serial.print(get_signal_name(meas->gnss_id, meas->sig_id));
    Serial.print(")");

    Serial.print(" - PR: ");
    Serial.print(meas->rc_mes, 3);
    Serial.print("m, CP: ");
    Serial.print(meas->pr_mes, 3);
    Serial.print(" cycles, CNR: ");
    Serial.print(meas->cno);
    Serial.print(" dB-Hz");

    // Add lock indicator
    if (meas->lock_time > 1000) {
        Serial.print(" [LOCKED]");
    } else if (meas->lock_time > 100) {
        Serial.print(" [TRACK]");
    } else {
        Serial.print(" [SEARCH]");
    }

    Serial.println();
}

void print_rawx_summary(const zedf9p_rawx_t *rawx) {
    // Count measurements by constellation
    uint8_t gps_count = 0, glo_count = 0, gal_count = 0, bds_count = 0, qzss_count = 0;

    for (uint8_t i = 0; i < rawx->num_meas; i++) {
        switch (rawx->meas[i].gnss_id) {
            case 0: gps_count++; break;     // GPS
            case 1: break;                  // SBAS (not counted)
            case 2: gal_count++; break;     // Galileo
            case 3: bds_count++; break;     // BeiDou
            case 5: qzss_count++; break;    // QZSS
            case 6: glo_count++; break;     // GLONASS
        }
    }

    Serial.print("  Constellation summary: GPS(");
    Serial.print(gps_count);
    Serial.print("), GLO(");
    Serial.print(glo_count);
    Serial.print("), GAL(");
    Serial.print(gal_count);
    Serial.print("), BDS(");
    Serial.print(bds_count);
    Serial.print("), QZSS(");
    Serial.print(qzss_count);
    Serial.println(")");
}

String get_gnss_name(uint8_t gnss_id) {
    switch (gnss_id) {
        case 0: return "GPS";
        case 1: return "SBAS";
        case 2: return "GAL";
        case 3: return "BDS";
        case 4: return "IMES";
        case 5: return "QZSS";
        case 6: return "GLO";
        default: return "UNK";
    }
}

String get_signal_name(uint8_t gnss_id, uint8_t sig_id) {
    switch (gnss_id) {
        case 0: // GPS
            switch (sig_id) {
                case 0: return "L1CA";
                case 3: return "L2CL";
                case 4: return "L2CM";
                default: return "L?";
            }
        case 2: // Galileo
            switch (sig_id) {
                case 0: return "E1C";
                case 1: return "E1B";
                case 5: return "E5aI";
                case 6: return "E5aQ";
                case 7: return "E5bI";
                case 8: return "E5bQ";
                default: return "E?";
            }
        case 3: // BeiDou
            switch (sig_id) {
                case 0: return "B1I";
                case 1: return "B1Q";
                case 2: return "B2I";
                case 3: return "B2Q";
                default: return "B?";
            }
        case 5: // QZSS
            switch (sig_id) {
                case 0: return "L1CA";
                case 1: return "L1S";
                case 4: return "L2CM";
                case 5: return "L2CL";
                default: return "Q?";
            }
        case 6: // GLONASS
            switch (sig_id) {
                case 0: return "L1OF";
                case 2: return "L2OF";
                default: return "R?";
            }
        default:
            return "?";
    }
}

void print_statistics() {
    unsigned long runtime = (millis() - start_time) / 1000;
    double rawx_rate = rawx_count > 0 ? (double)rawx_count / (runtime / 60.0) : 0.0;

    Serial.println();
    Serial.println("=== RAWX STATISTICS ===");
    Serial.print("Runtime: ");
    Serial.print(runtime);
    Serial.println(" seconds");
    Serial.print("Total RAWX messages: ");
    Serial.println(rawx_count);
    Serial.print("RAWX rate: ");
    Serial.print(rawx_rate, 2);
    Serial.println(" msg/min");

    if (last_rawx_time > 0) {
        Serial.print("Last RAWX: ");
        Serial.print((millis() - last_rawx_time) / 1000);
        Serial.println(" seconds ago");
    }

    Serial.println("=======================");
    Serial.println();
}