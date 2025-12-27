#include <inttypes.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "zedf9p/zedf9p.h"
#include "zedf9p/zedf9p_platform.h"
#include "f_util.h"
#include "ff.h"
#include "hw_config.h"
#include "pico/stdio.h"

struct {
    uint32_t session_start_ms;
    uint32_t last_flush_ms;
    uint32_t last_stats_ms;
    uint32_t bytes_logged;
    uint32_t messages_logged;
    uint32_t file_number;
    uint8_t logging_active;
    uint8_t sd_card_present;
    uint32_t write_errors;
    uint32_t recovery_attempts;
    uint8_t usb_ready;
    uint8_t file_open;
} logging_stats = {0};

struct {
    uint32_t i_tow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint8_t time_available;
} gnss_time = {0};

#define GNSS_EN 3
#define GNSS_RST 7
#define GNSS_PPS 6
#define GNSS_EXT_INT 5
#define GNSS_TX_RDY 4

#define USE_UART_LOGGING 0  // Set to 1 for UART DMA mode, 0 for I2C callback mode

#define LOG_FILENAME_PREFIX "GNSS"
#define LOG_FILENAME_EXTENSION ".ubx"
#define UART_TIMEOUT 1000
#define MAX_LOG_FILE_SIZE_MB 100
#define FLUSH_INTERVAL_MS 5000
#define USB_DEBUG_BUFFER_SIZE 512

#if USE_UART_LOGGING
#define UART_DMA_BUFFER_SIZE 2048
#endif

FIL dataFile;
zedf9p_t gnss_module;
static FATFS fs;  // *** CRITICAL FIX: Make this a static variable, not a pointer ***

// Buffers
char filename[32];
char debug_buffer[USB_DEBUG_BUFFER_SIZE];

// Initialize functions
uint8_t init_sd_card(void);
uint8_t create_new_log_file(void);
uint8_t recover_sd_card(void);
uint8_t reopen_log_file(void);
void configure_gnss_for_logging(void);
void process_gnss_logging(void);
void print_statistics(void);
void usb_debug_print(const char *message);

#if !USE_UART_LOGGING
void generic_ubx_callback(const ubx_message_t *message, const void *user_data);
void write_ubx_message_to_file(const ubx_message_t *message);
void update_gnss_time_from_timeutc(const ubx_message_t *message);
#endif

uint8_t init_sd_card(void) {
    usb_debug_print("Initializing SD card via SPI...\r\n");

    // Debug card detection before initializing
    sd_debug_card_detect();

    if (!sd_init_driver()) {
        usb_debug_print("Failed to initialize SD driver\r\n");
        return 0;
    }

    // Check again after driver init
    sd_debug_card_detect();

    // Multiple hardware initialization attempts
    for (int attempt = 0; attempt < 3; attempt++) {
        sleep_ms(100 * (attempt + 1));

        snprintf(debug_buffer, sizeof(debug_buffer),
            "SD mount attempt %d...\r\n", attempt + 1);
        usb_debug_print(debug_buffer);

        FRESULT result = f_mount(&fs, "0:", 1);  // *** FIXED: Pass address of fs ***

        if (result == FR_OK) {
            usb_debug_print("SD card mounted successfully\r\n");
            logging_stats.sd_card_present = 1;

            // Get SD card info
            DWORD free_clusters;
            FATFS *pfs = &fs;

            result = f_getfree("0:", &free_clusters, &pfs);
            if (result == FR_OK) {
                DWORD total_clusters = (fs.n_fatent - 2);
                DWORD total_sectors = total_clusters * fs.csize;
                DWORD free_sectors = free_clusters * fs.csize;
                uint32_t total_mb = total_sectors / 2048;
                uint32_t free_mb = free_sectors / 2048;

                snprintf(debug_buffer, sizeof(debug_buffer),
                    "SD Card: %"PRIu32" MB total, %"PRIu32" MB free\r\n", total_mb, free_mb);
                usb_debug_print(debug_buffer);

                snprintf(debug_buffer, sizeof(debug_buffer),
                    "Filesystem: FAT%d, Sector size: %d, Cluster size: %d KB\r\n",
                    fs.fs_type, FF_MAX_SS, (fs.csize * FF_MAX_SS) / 1024);
                usb_debug_print(debug_buffer);
            }

            usb_debug_print("SD card initialization completed successfully\r\n");
            return 1;
        }

        snprintf(debug_buffer, sizeof(debug_buffer),
            "Mount failed: %d\r\n", result);
        usb_debug_print(debug_buffer);

        if (attempt == 2) {
            usb_debug_print("SD card initialization failed after 3 attempts\r\n");
            usb_debug_print("Attempting to format SD card as FAT32...\r\n");

            BYTE work[FF_MAX_SS];
            const MKFS_PARM opt = {
                .fmt = FM_FAT32,
                .n_fat = 1,
                .align = 0,
                .n_root = 0,
                .au_size = 0
            };

            result = f_mkfs("0:", &opt, work, sizeof(work));

            if (result == FR_OK) {
                usb_debug_print("SD card formatted successfully\r\n");
                sleep_ms(100);

                result = f_mount(&fs, "0:", 1);  // *** FIXED: Pass address of fs ***
                if (result != FR_OK) {
                    snprintf(debug_buffer, sizeof(debug_buffer),
                        "Mount failed after format: %d\r\n", result);
                    usb_debug_print(debug_buffer);
                    return 0;
                }
                logging_stats.sd_card_present = 1;
                usb_debug_print("Filesystem mounted successfully after format\r\n");
                return 1;
            } else {
                snprintf(debug_buffer, sizeof(debug_buffer),
                    "Format failed: %d\r\n", result);
                usb_debug_print(debug_buffer);
                return 0;
            }
        }
    }

    return 0;
}

uint8_t create_new_log_file(void) {
    if (!logging_stats.sd_card_present) {
        return 0;
    }

    if (logging_stats.file_open) {
        f_close(&dataFile);
        logging_stats.file_open = 0;
    }

    FRESULT result;
    do {
        logging_stats.file_number++;
        snprintf(filename, sizeof(filename),
            "%s%03"PRIu32"%s", LOG_FILENAME_PREFIX, logging_stats.file_number, LOG_FILENAME_EXTENSION);
        result = f_stat(filename, NULL);
    } while (result == FR_OK);

    result = f_open(&dataFile, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (result != FR_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer),
            "ERROR: Failed to create file %s: %d\r\n", filename, result);
        usb_debug_print(debug_buffer);
        logging_stats.file_open = 0;
        return 0;
    }

    logging_stats.file_open = 1;
    logging_stats.write_errors = 0;

    snprintf(debug_buffer, sizeof(debug_buffer),
        "Created log file: %s\r\n", filename);
    usb_debug_print(debug_buffer);

    return 1;
}

uint8_t recover_sd_card(void) {
    usb_debug_print("Attempting SD card recovery...\r\n");
    logging_stats.recovery_attempts++;

    f_mount(NULL, "0:", 0);
    sleep_ms(500);

    const FRESULT result = f_mount(&fs, "0:", 1);  // *** FIXED: Pass address of fs ***
    if (result != FR_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer),
            "SD card remount failed: %d\r\n", result);
        usb_debug_print(debug_buffer);
        logging_stats.sd_card_present = 0;
        return 0;
    }

    logging_stats.sd_card_present = 1;
    usb_debug_print("SD card recovery successful\r\n");
    return 1;
}

uint8_t reopen_log_file(void) {
    if (!logging_stats.sd_card_present) {
        return 0;
    }

    const FRESULT result = f_open(&dataFile, filename, FA_OPEN_APPEND | FA_WRITE);
    if (result == FR_OK) {
        logging_stats.file_open = 1;
        logging_stats.write_errors = 0;
        usb_debug_print("Successfully reopened log file\r\n");
        return 1;
    } else {
        snprintf(debug_buffer, sizeof(debug_buffer),
            "Failed to reopen file %s: %d\r\n", filename, result);
        usb_debug_print(debug_buffer);
        return create_new_log_file();
    }
}

void configure_gnss_for_logging(void) {
    usb_debug_print("Configuring GNSS for UBX message logging...\r\n");

    // Set GNSS constellations and signals
    const zedf9p_gnss_config_t gnss_config = {
        .beidou_enabled = false,
        .beidou_b1_enabled = false,
        .beidou_b2_enabled = false,
        .galileo_enabled = true,
        .galileo_e1_enabled = true,
        .galileo_e5b_enabled = true,
        .glonass_enabled = false,
        .glonass_l1_enabled = false,
        .glonass_l2_enabled = false,
        .gps_enabled = true,
        .gps_l1ca_enabled = true,
        .gps_l2c_enabled = true,
        .qzss_enabled = false,
        .qzss_l1ca_enabled = false,
        .qzss_l1s_enabled = false,
        .qzss_l2c_enabled = false,
        .sbas_enabled = false,
        .sbas_l1ca_enabled = false
    };

    // Configure GNSS signals
    zedf9p_config_gnss_signals(&gnss_module, UBLOX_CFG_LAYER_RAM, &gnss_config);

    // Configure messages
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_CLOCK, UBLOX_CFG_LAYER_RAM, 1);
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_TIMEUTC, UBLOX_CFG_LAYER_RAM, 1);
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_RXM, UBX_RXM_RAWX, UBLOX_CFG_LAYER_RAM, 1);
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_RXM, UBX_RXM_SFRBX, UBLOX_CFG_LAYER_RAM, 1);

    usb_debug_print("GNSS configuration complete\r\n");
}

void process_gnss_logging(void) {
    if (!logging_stats.logging_active) {
        return;
    }

#if USE_UART_LOGGING
    // UART DMA mode - data is written automatically in DMA callback
    // Nothing to do here except handle errors
#else
    // I2C callback mode - process through library
    const zedf9p_status_t status = zedf9p_process_data(&gnss_module);
    if (status != ZEDF9P_OK && status != ZEDF9P_ERR_NO_DATA) {
        snprintf(debug_buffer, sizeof(debug_buffer),
            "GNSS processing error: %s\r\n", zedf9p_status_error(status));
        usb_debug_print(debug_buffer);
    }
#endif

    // Handle SD card errors and recovery
    if (logging_stats.write_errors > 3) {
        usb_debug_print("Multiple write errors detected, attempting recovery...\r\n");

        if (!reopen_log_file()) {
            if (recover_sd_card()) {
                if (!reopen_log_file()) {
                    create_new_log_file();
                }
            } else {
                logging_stats.logging_active = 0;
                usb_debug_print("SD card recovery failed, logging paused\r\n");
                return;
            }
        }
    }

    // Try to restart logging if paused
    if (!logging_stats.logging_active && logging_stats.recovery_attempts < 10) {
        if (recover_sd_card() && create_new_log_file()) {
            logging_stats.logging_active = 1;
            usb_debug_print("Logging resumed after recovery\r\n");
#if USE_UART_LOGGING
            // Restart UART DMA
            active_buffer = 0;
            zedf9p_platform_uart_read(uart_buffer_0, UART_DMA_BUFFER_SIZE, UART_TIMEOUT);
#endif
        }
    }

    // Check file size limit and rotate if needed
    if (logging_stats.sd_card_present && logging_stats.file_open &&
        f_size(&dataFile) > (MAX_LOG_FILE_SIZE_MB * 1024UL * 1024UL)) {

        usb_debug_print("File size limit reached, creating new file...\r\n");

        if (logging_stats.file_open) {
            f_close(&dataFile);
            logging_stats.file_open = 0;
        }

        if (!create_new_log_file()) {
            usb_debug_print("ERROR: Failed to create new log file\r\n");
            logging_stats.logging_active = 0;
        }
    }
}

#if !USE_UART_LOGGING
void write_ubx_message_to_file(const ubx_message_t *message) {
    if (!logging_stats.file_open || !logging_stats.sd_card_present) {
        return;
    }

    // Build UBX header
    const uint8_t ubx_header[6] = {
        0xB5, 0x62,
        message->msg_class,
        message->msg_id,
        (uint8_t)(message->length & 0xFF),
        (uint8_t)((message->length >> 8) & 0xFF)
    };

    // Calculate checksum
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) {
        ck_a += ubx_header[i];
        ck_b += ck_a;
    }
    for (uint16_t i = 0; i < message->length; i++) {
        ck_a += message->payload[i];
        ck_b += ck_a;
    }

    const uint8_t checksum[2] = {ck_a, ck_b};

    // Write header
    UINT bytes_written;
    FRESULT result = f_write(&dataFile, ubx_header, 6, &bytes_written);
    if (result != FR_OK || bytes_written != 6) {
        logging_stats.write_errors++;
        return;
    }

    // Write payload
    if (message->length > 0) {
        result = f_write(&dataFile, message->payload, message->length, &bytes_written);
        if (result != FR_OK || bytes_written != message->length) {
            logging_stats.write_errors++;
            return;
        }
    }

    // Write checksum
    result = f_write(&dataFile, checksum, 2, &bytes_written);
    if (result != FR_OK || bytes_written != 2) {
        logging_stats.write_errors++;
        return;
    }

    // Update statistics
    logging_stats.bytes_logged += 6 + message->length + 2;
    logging_stats.messages_logged++;

    if (logging_stats.write_errors > 0) {
        logging_stats.write_errors = 0;
    }
}

void generic_ubx_callback(const ubx_message_t *message, const void *user_data) {
    (void)user_data;

    write_ubx_message_to_file(message);

    if (message->msg_class == UBX_CLASS_NAV && message->msg_id == UBX_NAV_TIMEUTC) {
        update_gnss_time_from_timeutc(message);
    }
}

void update_gnss_time_from_timeutc(const ubx_message_t *message) {
    if (!message || message->length < sizeof(zedf9p_nav_timeutc_t)) {
        return;
    }

    zedf9p_nav_timeutc_t timeutc;
    memcpy(&timeutc, message->payload, sizeof(zedf9p_nav_timeutc_t));

    gnss_time.i_tow = timeutc.i_tow;
    gnss_time.year = timeutc.year;
    gnss_time.month = timeutc.month;
    gnss_time.day = timeutc.day;
    gnss_time.hour = timeutc.hour;
    gnss_time.min = timeutc.min;
    gnss_time.sec = timeutc.sec;
    gnss_time.valid = timeutc.valid;
    gnss_time.time_available = 1;
}
#endif

void usb_debug_print(const char *message) {
    if (logging_stats.usb_ready && message != NULL) {
        const uint16_t len = strlen(message);
        if (len > 0) {
            printf("%s", message);
            sleep_ms(1);
        }
    }
}

void print_statistics(void) {
    const uint32_t uptime_s = (zedf9p_platform_get_tick() - logging_stats.session_start_ms) / 1000;
    uint32_t data_rate = 0;

    if (uptime_s > 0) {
        data_rate = logging_stats.bytes_logged / uptime_s;
    }

    snprintf(debug_buffer, sizeof(debug_buffer),
        "Stats: Up=%"PRIu32"s Rate=%"PRIu32"B/s Msgs=%"PRIu32" Bytes=%"PRIu32" Err=%"PRIu32"\r\n",
        uptime_s, data_rate, logging_stats.messages_logged, logging_stats.bytes_logged, logging_stats.write_errors);
    usb_debug_print(debug_buffer);

#if !USE_UART_LOGGING
    if (gnss_time.time_available) {
        snprintf(debug_buffer, sizeof(debug_buffer),
            "GNSS Time: %04d-%02d-%02d %02d:%02d:%02d UTC (iTOW: %"PRIu32" ms)\r\n",
            gnss_time.year, gnss_time.month, gnss_time.day,
            gnss_time.hour, gnss_time.min, gnss_time.sec, gnss_time.i_tow);
        usb_debug_print(debug_buffer);
    }
#endif
}

int main() {
    stdio_init_all();

    gpio_init(GNSS_EN);
    gpio_init(GNSS_PPS);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(GNSS_EN, GPIO_OUT);
    gpio_set_dir(GNSS_PPS, GPIO_OUT);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_put(GNSS_EN, 1);
    gpio_put(GNSS_PPS, 0);

    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    logging_stats.usb_ready = 1;

#if USE_UART_LOGGING
    usb_debug_print("ZEDF9P UBX Logger - UART DMA Mode\r\n");
#else
    usb_debug_print("ZEDF9P UBX Logger - I2C Callback Mode\r\n");
#endif
    usb_debug_print("=====================================\r\n");

    // Initialize SD card
    if (!init_sd_card()) {
        usb_debug_print("FATAL: SD card initialization failed!\r\n");
        while(1) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(500);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(500);
        }
    }

#if !USE_UART_LOGGING
    zedf9p_platform_i2c_init(ZEDF9P_I2C_ADDR);
    const zedf9p_interface_t io = {
        .i2c_write = zedf9p_platform_i2c_write,
        .i2c_read = zedf9p_platform_i2c_read,
        .uart_write = NULL,
        .uart_read = NULL,
        .delay_ms = zedf9p_platform_delay,
        .get_millis = zedf9p_platform_get_tick,
        .debug_print = usb_debug_print
    };

    const zedf9p_status_t status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_I2C, 0x42, io);
    if (status != ZEDF9P_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer),
            "FATAL: GNSS init failed: %s\r\n", zedf9p_status_error(status));
        usb_debug_print(debug_buffer);
        while (1) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
        }
    }

    usb_debug_print("GNSS module initialized successfully (I2C mode)\r\n");
#else
    const zedf9p_interface_t io = {
        .i2c_write = NULL,
        .i2c_read = NULL,
        .uart_write = zedf9p_platform_uart_write,
        .uart_read = zedf9p_platform_uart_read,
        .delay_ms = zedf9p_platform_delay,
        .get_millis = zedf9p_platform_get_tick,
        .debug_print = usb_debug_print
    };

    const zedf9p_status_t status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_UART, 0x42, io);
    if (status != ZEDF9P_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer),
            "FATAL: GNSS init failed: %s\r\n", zedf9p_status_error(status));
        usb_debug_print(debug_buffer);
        while (1) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
        }
    }
#endif

    configure_gnss_for_logging();

    if (!create_new_log_file()) {
        usb_debug_print("FATAL: Failed to create log file!\r\n");
        while (1) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
        }
    }

#if !USE_UART_LOGGING
    zedf9p_register_generic_callback(&gnss_module, generic_ubx_callback, NULL);
    usb_debug_print("Registered generic callback for UBX messages\r\n");
#endif

    logging_stats.session_start_ms = zedf9p_platform_get_tick();
    logging_stats.last_flush_ms = zedf9p_platform_get_tick();
    logging_stats.last_stats_ms = zedf9p_platform_get_tick();
    logging_stats.logging_active = 1;
    logging_stats.file_open = 1;

    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    usb_debug_print("Logging started!\r\n");

    while (1) {
        process_gnss_logging();

        if (zedf9p_platform_get_tick() - logging_stats.last_stats_ms > 10000) {
            print_statistics();
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            logging_stats.last_stats_ms = zedf9p_platform_get_tick();
        }

        if (zedf9p_platform_get_tick() - logging_stats.last_flush_ms > FLUSH_INTERVAL_MS) {
            if (logging_stats.sd_card_present && logging_stats.logging_active && logging_stats.file_open) {
                const FRESULT flush_result = f_sync(&dataFile);
                if (flush_result != FR_OK) {
                    snprintf(debug_buffer, sizeof(debug_buffer),
                        "File sync error: %d\r\n", flush_result);
                    usb_debug_print(debug_buffer);
                    logging_stats.write_errors++;
                } else {
                    logging_stats.last_flush_ms = zedf9p_platform_get_tick();
                }
            }
        }

        sleep_ms(1);
    }
}