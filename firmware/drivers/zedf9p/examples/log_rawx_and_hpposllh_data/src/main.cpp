/*!
 * @file main.cpp
 * @brief ZEDF9P GNSS Data Logger - Saves RAWX and HPPOSLLH to SD card in UBX format
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-09-14
 *
 * This example logs GNSS data to SD card in native UBX format for post-processing.
 * Based on SparkFun DataLoggingExample3 but adapted for the ZEDF9P driver.
 */

#include <Arduino.h>
#include <Wire.h>
#include <STM32SD.h>
#include "Sd2Card.h"
#include <zedf9p.h>
#include <zedf9p_platform.h>

bool init_stm32_sdio();
bool create_new_log_file();
void write_file_header();
size_t write_ubx_message_to_file(const ubx_message_t *message);
void flush_write_buffer();
void calculate_ubx_checksum(const ubx_message_t *message, uint8_t *ck_a, uint8_t *ck_b);
void configure_gnss_for_logging();
void handle_serial_commands();
void print_brief_stats();
void print_detailed_statistics();
void restart_log_file();
void stop_logging();
void force_flush();
void error_blink_forever();
void print_help();

//char filename[32];

// STM32F405 specific SDIO pin definitions (hardcoded)
#define SDIO_D0   PC8
#define SDIO_D1   PC9
#define SDIO_D2   PC10
#define SDIO_D3   PC11
#define SDIO_CLK  PC12
#define SDIO_CMD  PD2

// SparkFun STM32 Thing Plus pin definitions
#define STATUS_LED_PIN LED_BUILTIN  // PC13 on STM32F405

#define GNSS_SERIAL &Serial3
#define GNSS_WIRE &Wire

// Data logging configuration
#define LOG_FILENAME_PREFIX "GNSS"
#define LOG_FILENAME_EXTENSION ".ubx"
#define MAX_LOG_FILE_SIZE_MB 200    // Larger files for SDIO
#define FLUSH_INTERVAL_MS 5000      // Flush every 5 seconds
#define WRITE_BUFFER_SIZE 64       // Buffer size for batched writes

// SDIO configuration for STM32F405
#define USE_SDIO 1
#define ENABLE_DEDICATED_SPI 0

// GNSS module and data structures
zedf9p_t gnss_module;
zedf9p_nav_hpposllh_t hppos_data; // For high precision position
zedf9p_rawx_t rawx_data;          // For raw measurements
zedf9p_mon_ver_t version_info;    // For version information

// Communication interface selection
bool use_i2c = true;  // Set to false to use UART instead

// I2C Configuration
TwoWire* selected_i2c;
//unsigned long i2c_freq = 100000;   // I2C frequency in Hz

// UART Configuration
HardwareSerial* selected_uart;
unsigned long uart_baudrate = 38400;       // UART baudrate
uint32_t uart_config = SERIAL_8N1;         // UART config (8N1, 8E1, etc.)

File dataFile;
// Write buffer for performance optimization
uint8_t write_buffer[WRITE_BUFFER_SIZE];
size_t buffer_index = 0;

// Logging statistics
struct {
    unsigned long session_start_ms;
    unsigned long last_flush_ms;
    unsigned long bytes_logged;
    unsigned long rawx_count;
    unsigned long hppos_count;
    unsigned long file_number;
    bool logging_active;
    bool sd_card_present;
    unsigned long write_errors;
    unsigned long max_write_time_us;
    unsigned long total_messages;
} logging_stats = {0};

// Message callback for logging
void gnss_message_logger(const ubx_message_t *message, void *user_data) {
    if (!logging_stats.logging_active || !dataFile) {
        return;
    }

    // Check if we need to create a new file due to size limit
    if (dataFile.size() > (MAX_LOG_FILE_SIZE_MB * 1024UL * 1024UL)) {
        flush_write_buffer();
        dataFile.close();
        if (!create_new_log_file()) {
            Serial.println("ERROR: Failed to create new log file");
            logging_stats.logging_active = false;
            return;
        }
    }

    // Measure write performance
    const unsigned long write_start = micros();

    // Write complete UBX message to buffer
    const size_t bytes_written = write_ubx_message_to_file(message);

    if (const unsigned long write_time = micros() - write_start; write_time > logging_stats.max_write_time_us) {
        logging_stats.max_write_time_us = write_time;
    }

    if (bytes_written > 0) {
        logging_stats.bytes_logged += bytes_written;
        logging_stats.total_messages++;

        // Update message counters
        if (message->msg_class == UBX_CLASS_RXM && message->msg_id == UBX_RXM_RAWX) {
            logging_stats.rawx_count++;
        } else if (message->msg_class == UBX_CLASS_NAV && message->msg_id == UBX_NAV_HPPOSLLH) {
            logging_stats.hppos_count++;
        }

        // Periodic flush to ensure data is saved
        if (millis() - logging_stats.last_flush_ms > FLUSH_INTERVAL_MS) {
            flush_write_buffer();
            dataFile.flush();
            logging_stats.last_flush_ms = millis();

            // Performance statistics
            if (const unsigned long uptime_s = (millis() - logging_stats.session_start_ms) / 1000; uptime_s > 0) {
                const unsigned long data_rate = logging_stats.bytes_logged / uptime_s;
                Serial.print("SDIO Rate: ");
                Serial.print(data_rate);
                Serial.print(" B/s, Msgs: ");
                Serial.print(logging_stats.total_messages);
                Serial.print(", Max Write: ");
                Serial.print(logging_stats.max_write_time_us);
                Serial.println(" us");
            }
        }
    } else {
        logging_stats.write_errors++;
        Serial.println("ERROR: Failed to write message");
        if (logging_stats.write_errors > 10) {
            logging_stats.logging_active = false;
            Serial.println("Too many write errors - stopping logging");
        }
    }
}

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);

    Serial.begin(115200);

    while (!Serial) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        delay(100);
    }

    digitalWrite(STATUS_LED_PIN, HIGH);

    Serial.println("ZEDF9P GNSS Data Logger");
    Serial.println("=======================");

    // Initialize STM32 SDIO SD card
    if (!init_stm32_sdio()) {
        Serial.println("FATAL: STM32 SDIO initialization failed!");
        error_blink_forever();
    }

    selected_i2c = GNSS_WIRE;
    selected_uart = GNSS_SERIAL;    // Change to &Serial2, &Serial3, etc. as needed

    zedf9p_status_t status;
    if (use_i2c) {
        platform_set_i2c_interface(selected_i2c);
        selected_i2c->begin();
        //selected_i2c->setClock(i2c_freq);

        constexpr zedf9p_interface_t io = {
            .i2c_write = platform_i2c_write,  // Not used for UART
            .i2c_read = platform_i2c_read,   // Not used for UART
            .uart_write = NULL,
            .uart_read = NULL,
            .delay_ms = platform_delay_ms,
            .get_millis = platform_get_millis,
            .debug_print = NULL  // Add this
            };

        status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_I2C, 0, io);
    } else {
        platform_set_uart_interface(GNSS_SERIAL);
        selected_uart->begin(uart_baudrate, uart_config);

        constexpr zedf9p_interface_t io = {
            .i2c_write = NULL,  // Not used for UART
            .i2c_read = NULL,   // Not used for UART
            .uart_write = platform_uart_write,
            .uart_read = platform_uart_read,
            .delay_ms = platform_delay_ms,
            .get_millis = platform_get_millis,
            .debug_print = NULL  // Add this
            };

        status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_UART, 0, io);
    }

    if (status != ZEDF9P_OK) {
        Serial.print("FATAL: GNSS initialization failed: ");
        Serial.println(zedf9p_status_error(status));
        while (true) {
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
            delay(1000);
        }
    }

    Serial.println("GNSS module initialized successfully");

    // Register callback for all messages
    zedf9p_register_generic_callback(&gnss_module, gnss_message_logger, NULL);

    // Configure GNSS for logging
    configure_gnss_for_logging();

    // Create initial log file
    if (!create_new_log_file()) {
        Serial.println("FATAL: Failed to create log file!");
        error_blink_forever();
    }

    logging_stats.session_start_ms = millis();
    logging_stats.last_flush_ms = millis();
    logging_stats.logging_active = true;

    Serial.println("Data logging started!");
    Serial.println("Commands: 's' = statistics, 'r' = restart file, 'q' = stop logging");
    Serial.println("=========================================================");
}

void loop() {
    static unsigned long last_status_ms = 0;
    static bool led_state = false;

    // Process GNSS data continuously
    if (const zedf9p_status_t status = zedf9p_process_data(&gnss_module); status != ZEDF9P_OK && status != ZEDF9P_ERR_NO_DATA) {
        Serial.print("GNSS error: ");
        Serial.println(zedf9p_status_error(status));
    }

    // Handle serial commands
    handle_serial_commands();

    // Status LED and periodic info (every 10 seconds for high-rate logging)
    if (millis() - last_status_ms > 10000) {
        led_state = !led_state;
        digitalWrite(STATUS_LED_PIN, led_state);

        if (logging_stats.logging_active) {
            print_brief_stats();
        }

        last_status_ms = millis();
    }

    // Minimal delay for maximum throughput
    delayMicroseconds(100);
}

bool init_stm32_sdio() {
    Serial.print("Initializing STM32 SDIO...");

    // Initialize STM32SD library with SDIO
    if (!SD.begin()) {
        Serial.println(" FAILED");
        Serial.println("Troubleshooting:");
        Serial.println("- Check SD card is inserted properly");
        Serial.println("- Verify SD card is FAT32 formatted");
        Serial.println("- Try a different SD card (Class 10 recommended)");
        Serial.println("- Check SDIO pins are not used elsewhere");
        Serial.println("- SDIO pins: D0=PC8, D1=PC9, D2=PC10, D3=PC11, CLK=PC12, CMD=PD2");
        return false;
    }

    Serial.println(" SUCCESS");

    logging_stats.sd_card_present = true;
    return true;
}

bool create_new_log_file() {
    if (!logging_stats.sd_card_present) {
        return false;
    }

    // Flush any pending data
    flush_write_buffer();

    // Close existing file
    if (dataFile) {
        dataFile.close();
    }

    // Find next available filename
    char filename[32];
    do {
        logging_stats.file_number++;
        sprintf(filename, "/%s%03lu%s", LOG_FILENAME_PREFIX,
                logging_stats.file_number, LOG_FILENAME_EXTENSION);
    } while (SDClass::exists(filename));

    // Create new file
    dataFile = SDClass::open(filename, FILE_WRITE);
    if (!dataFile) {
        Serial.print("ERROR: Failed to create file: ");
        Serial.println(filename);
        return false;
    }

    Serial.print("Created log file: ");
    Serial.println(filename);

    // Write file header
    write_file_header();

    return true;
}

void write_file_header() {
    if (!dataFile) return;

    // Write file header with STM32 specific info
    char header[512];
    sprintf(header,
        "# ZEDF9P GNSS Data Log - STM32F405 SDIO\n"
        "# Start Time: %lu ms\n"
        "# File Number: %lu\n"
        "# Format: UBX binary\n"
        "# Platform: SparkFun STM32 Thing Plus\n"
        "# Library: STM32duino STM32SD\n"
        "# Interface: SDIO (High Speed)\n"
        "# Contains: RAWX, HPPOSLLH messages\n"
        "# SDIO Pins: D0=PC8, D1=PC9, D2=PC10, D3=PC11, CLK=PC12, CMD=PD2\n"
        "# ==========================================\n",
        millis(), logging_stats.file_number);

    dataFile.print(header);
    dataFile.flush();
}

size_t write_ubx_message_to_file(const ubx_message_t *message) {
    if (!dataFile || !message || !message->valid) {
        return 0;
    }

    // Calculate total message size
    const size_t total_size = 6 + message->length + 2;

    // Write directly without buffering
    dataFile.write(0xB5);  // Sync char 1
    dataFile.write(0x62);  // Sync char 2
    dataFile.write(message->msg_class);
    dataFile.write(message->msg_id);
    dataFile.write(static_cast<uint8_t>(message->length & 0xFF));
    dataFile.write(static_cast<uint8_t>((message->length >> 8) & 0xFF));

    if (message->length > 0) {
        dataFile.write(message->payload, message->length);
    }

    uint8_t ck_a, ck_b;
    calculate_ubx_checksum(message, &ck_a, &ck_b);
    dataFile.write(ck_a);
    dataFile.write(ck_b);

    dataFile.flush(); // Force immediate write

    return total_size;
}

void flush_write_buffer() {
    if (buffer_index > 0 && dataFile) {
        if (const size_t bytes_written = dataFile.write(write_buffer, buffer_index); bytes_written != buffer_index) {
            logging_stats.write_errors++;
            Serial.print("ERROR: Buffer flush failed. Expected ");
            Serial.print(buffer_index);
            Serial.print(" wrote ");
            Serial.println(bytes_written);
        }
        buffer_index = 0;
    }
}

void calculate_ubx_checksum(const ubx_message_t *message, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;

    // Add class and ID
    *ck_a += message->msg_class;
    *ck_b += *ck_a;
    *ck_a += message->msg_id;
    *ck_b += *ck_a;

    // Add length bytes
    *ck_a += static_cast<uint8_t>(message->length & 0xFF);
    *ck_b += *ck_a;
    *ck_a += static_cast<uint8_t>((message->length >> 8) & 0xFF);
    *ck_b += *ck_a;

    // Add payload
    for (uint16_t i = 0; i < message->length; i++) {
        *ck_a += message->payload[i];
        *ck_b += *ck_a;
    }
}

void configure_gnss_for_logging() {
    Serial.println("Configuring GNSS for data logging...");

    // Set measurement rate (adjust as needed for your application)
    zedf9p_set_measurement_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, 1000, 1);  // 1 Hz

    // Configure constellation (start with GPS only for manageable data rates)
    constexpr zedf9p_gnss_config_t gnss_config = {
        .beidou_enabled = false,
        .beidou_b1_enabled = false,
        .beidou_b2_enabled = false,
        .galileo_enabled = false,
        .galileo_e1_enabled = false,
        .galileo_e5b_enabled = false,
        .glonass_enabled = false,
        .glonass_l1_enabled = false,
        .glonass_l2_enabled = false,
        .gps_enabled = true,
        .gps_l1ca_enabled = true,
        .gps_l2c_enabled = false,
        .qzss_enabled = false,
        .qzss_l1ca_enabled = false,
        .qzss_l1s_enabled = false,
        .qzss_l2c_enabled = false,
        .sbas_enabled = false,
        .sbas_l1ca_enabled = false
    };

    zedf9p_config_gnss_signals(&gnss_module, UBLOX_CFG_LAYER_RAM, &gnss_config);

    // Enable RAWX messages (every measurement)
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_RXM, UBX_RXM_RAWX, 1);

    // Enable HPPOSLLH messages (every measurement)
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, 1);

    // Disable 7F check for RAWX compatibility
    zedf9p_disable_7f_check(&gnss_module, true);

    delay(1000);  // Allow configuration to take effect
    Serial.println("GNSS configuration complete");
}

void handle_serial_commands() {
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
            case 's':
            case 'S':
                print_detailed_statistics();
                break;

            case 'r':
            case 'R':
                restart_log_file();
                break;

            case 'q':
            case 'Q':
                stop_logging();
                break;

            case 'f':
            case 'F':
                force_flush();
                break;

            case 'h':
            case 'H':
                print_help();
                break;

            default:
                break;
        }
    }
}

void print_brief_stats() {
    const unsigned long uptime_s = (millis() - logging_stats.session_start_ms) / 1000;
    unsigned long data_rate = 0;
    if (uptime_s > 0) {
        data_rate = logging_stats.bytes_logged / uptime_s;
    }

    Serial.print("Up:");
    Serial.print(uptime_s);
    Serial.print("s Rate:");
    Serial.print(data_rate);
    Serial.print("B/s RAWX:");
    Serial.print(logging_stats.rawx_count);
    Serial.print(" HP:");
    Serial.print(logging_stats.hppos_count);
    Serial.print(" Err:");
    Serial.println(logging_stats.write_errors);
}

void print_detailed_statistics() {
    const unsigned long uptime_s = (millis() - logging_stats.session_start_ms) / 1000;
    char filename[32];

    Serial.println("\n=== STM32 SDIO LOGGING STATISTICS ===");
    Serial.print("Session uptime: ");
    Serial.print(uptime_s);
    Serial.println(" seconds");

    Serial.print("Logging active: ");
    Serial.println(logging_stats.logging_active ? "YES" : "NO");

    Serial.print("Current file: GNSS");
    sprintf(filename, "%03lu", logging_stats.file_number);
    Serial.print(filename);
    Serial.println(".ubx");

    Serial.print("Total bytes logged: ");
    Serial.println(logging_stats.bytes_logged);

    Serial.print("Total messages: ");
    Serial.println(logging_stats.total_messages);

    Serial.print("RAWX messages: ");
    Serial.println(logging_stats.rawx_count);

    Serial.print("HPPOSLLH messages: ");
    Serial.println(logging_stats.hppos_count);

    Serial.print("Write errors: ");
    Serial.println(logging_stats.write_errors);

    Serial.print("Buffer index: ");
    Serial.print(buffer_index);
    Serial.print("/");
    Serial.println(WRITE_BUFFER_SIZE);

    if (dataFile) {
        Serial.print("Current file size: ");
        Serial.print(dataFile.size() / 1024);
        Serial.println(" KB");
    }

    if (uptime_s > 0) {
        Serial.print("Average data rate: ");
        Serial.print(logging_stats.bytes_logged / uptime_s);
        Serial.println(" bytes/sec");

        Serial.print("Messages per second: ");
        Serial.print(logging_stats.total_messages / uptime_s);
        Serial.println(" msg/s");
    }

    Serial.print("Max write time: ");
    Serial.print(logging_stats.max_write_time_us);
    Serial.println(" microseconds");

    Serial.println("=====================================\n");
}

void restart_log_file() {
    Serial.println("Restarting log file...");
    logging_stats.logging_active = false;

    flush_write_buffer();
    if (dataFile) {
        dataFile.close();
    }

    if (create_new_log_file()) {
        logging_stats.logging_active = true;
        Serial.println("New log file created - logging resumed");
    } else {
        Serial.println("ERROR: Failed to create new log file");
    }
}

void stop_logging() {
    Serial.println("Stopping STM32 SDIO logging...");
    logging_stats.logging_active = false;

    flush_write_buffer();
    if (dataFile) {
        dataFile.flush();
        dataFile.close();
        Serial.println("Log file closed and synced");
    }

    print_detailed_statistics();
    Serial.println("Logging stopped. Reset to resume.");
}

void force_flush() {
    flush_write_buffer();
    if (dataFile) {
        dataFile.flush();
        Serial.println("File flushed to SD card");
    } else {
        Serial.println("No file open");
    }
}

void error_blink_forever() {
    while (true) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }
}

void print_help() {
    Serial.println("\n=== STM32SD LOGGER COMMANDS ===");
    Serial.println("s - Show detailed statistics");
    Serial.println("r - Restart log file");
    Serial.println("q - Stop logging");
    Serial.println("f - Force flush to SD card");
    Serial.println("h - Show this help");
    Serial.println("===============================\n");
}