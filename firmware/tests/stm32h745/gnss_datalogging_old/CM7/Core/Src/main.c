/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usb2422.h"
#include "usb2422_platform.h"
#include "zedf9p.h"
#include "zedf9p_platform.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

#define LOG_FILENAME_PREFIX "GNSS"
#define LOG_FILENAME_EXTENSION ".ubx"
#define MAX_LOG_FILE_SIZE_MB 100
#define FLUSH_INTERVAL_MS 5000
#define PROCESS_BUFFER_SIZE 1024
#define USB_DEBUG_BUFFER_SIZE 512

#define ZEDF9P_I2C &hi2c2
#define ZEDF9P_UART &huart4

#define USB2422_I2C &hi2c1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
bool zedf9p_use_i2c = false; // Set to false to use UART instead

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
FIL dataFile;
zedf9p_t gnss_module;

// Buffers
uint8_t process_buffer[PROCESS_BUFFER_SIZE];
char filename[32];
char debug_buffer[USB_DEBUG_BUFFER_SIZE];

// USB instances
usb2422_t usb_hub;

logging_stats_t logging_stats = {0};
gnss_time_t gnss_time = {0};

usb2422_hub_settings_t hub_settings = {0};
usb2422_power_settings_t power_settings = {0};
usb2422_downstream_port_settings_t port_settings = {false};
usb2422_cfg_regs_t config_registers = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_UART4_Init(void);

/* USER CODE BEGIN PFP */
uint8_t init_sd_card(void);
uint8_t create_new_log_file(void);
uint8_t recover_sd_card(void);
uint8_t reopen_log_file(void);

void configure_gnss_for_logging(void);
void process_gnss_logging(void);
void print_statistics(void);
// void usb_debug_print(const char *message);
void update_gnss_time_from_timeutc(const zedf9p_nav_timeutc_t *timeutc);

// Message callbacks
void clock_callback(const ubx_message_t *message, void *user_data);
void hpposecef_callback(const ubx_message_t *message, void *user_data);
void hpposllh_callback(const ubx_message_t *message, void *user_data);
void timeutc_callback(const ubx_message_t *message, void *user_data);
void rawx_callback(const ubx_message_t *message, void *user_data);
void sfrbx_callback(const ubx_message_t *message, void *user_data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t init_sd_card(void) {
    usb_debug_print("Initializing SD card...\r\n");

    // Multiple hardware initialization attempts
    for (int attempt = 0; attempt < 3; attempt++) {
        HAL_Delay(100 * (attempt + 1)); // Increasing delay between attempts

        snprintf(debug_buffer, sizeof(debug_buffer),
                "SD hardware init attempt %d...\r\n", attempt + 1);
        usb_debug_print(debug_buffer);

        // Try to initialize SD hardware
        const HAL_StatusTypeDef hal_result = HAL_SD_Init(&hsd2);
        if (hal_result != HAL_OK) {
            snprintf(debug_buffer, sizeof(debug_buffer),
                    "HAL_SD_Init failed with code: %d\r\n", hal_result);
            usb_debug_print(debug_buffer);
            continue;
        }

        // Check if card is ready
        const HAL_SD_CardStateTypeDef cardState = HAL_SD_GetCardState(&hsd2);
        snprintf(debug_buffer, sizeof(debug_buffer),
                "SD card state: %lu\r\n", cardState);
        usb_debug_print(debug_buffer);

        if (cardState == HAL_SD_CARD_TRANSFER) {
            usb_debug_print("SD card hardware ready\r\n");
            break;
        }

        if (attempt == 2) {
            usb_debug_print("SD card hardware initialization failed after 3 attempts\r\n");
            return 0;
        }
    }

    // Wait before filesystem operations
    HAL_Delay(500);

    // Try to mount the filesystem
    usb_debug_print("Attempting to mount filesystem...\r\n");
    FRESULT result = f_mount(&SDFatFS, SDPath, 1);

    if (result != FR_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "Initial SD mount failed: %d\r\n", result);
        usb_debug_print(debug_buffer);

        // Try formatting the card if mount fails
        usb_debug_print("Attempting to format SD card as FAT32...\r\n");
        BYTE work[_MAX_SS]; // Work area for f_mkfs
        result = f_mkfs(SDPath, FM_FAT32, 0, work, sizeof(work));

        if (result == FR_OK) {
            usb_debug_print("SD card formatted successfully\r\n");
            HAL_Delay(100);

            // Try mounting again after format
            result = f_mount(&SDFatFS, SDPath, 1);
            if (result != FR_OK) {
                snprintf(debug_buffer, sizeof(debug_buffer),
                        "Mount failed after format: %d\r\n", result);
                usb_debug_print(debug_buffer);
                return 0;
            }
        } else {
            snprintf(debug_buffer, sizeof(debug_buffer),
                    "Format failed: %d\r\n", result);
            usb_debug_print(debug_buffer);
            return 0;
        }
    }

    usb_debug_print("Filesystem mounted successfully\r\n");

    // Get detailed SD card and filesystem information
    DWORD free_clusters;
    FATFS* fs;
    result = f_getfree(SDPath, &free_clusters, &fs);
    if (result == FR_OK) {
        const DWORD total_clusters = (fs->n_fatent - 2) * fs->csize;
        const uint32_t total_mb = total_clusters / 2048;
        const uint32_t free_mb = free_clusters * fs->csize / 2048;

        snprintf(debug_buffer, sizeof(debug_buffer),
                "SD Card: %lu MB total, %lu MB free\r\n", total_mb, free_mb);
        usb_debug_print(debug_buffer);

        snprintf(debug_buffer, sizeof(debug_buffer),
                "Filesystem: FAT%d, Sector size: %d, Cluster size: %d\r\n",
                fs->fs_type, _MAX_SS, fs->csize * _MAX_SS);
        usb_debug_print(debug_buffer);
    } else {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "Failed to get filesystem info: %d\r\n", result);
        usb_debug_print(debug_buffer);
        return 0;
    }

    logging_stats.sd_card_present = 1;
    usb_debug_print("SD card initialization completed successfully\r\n");
    return 1;
}

uint8_t create_new_log_file(void) {
    if (!logging_stats.sd_card_present) {
        return 0;
    }

    // Close existing file
    if (logging_stats.file_open) {
        f_close(&dataFile);
        logging_stats.file_open = 0;
    }

    // Find next available filename
    FRESULT result;
    do {
        logging_stats.file_number++;
        snprintf(filename, sizeof(filename), "%s%03lu%s",
                LOG_FILENAME_PREFIX, logging_stats.file_number, LOG_FILENAME_EXTENSION);

        // Check if file exists
        result = f_stat(filename, NULL);
    } while (result == FR_OK); // Continue if file exists

    // Create new file
    result = f_open(&dataFile, filename, FA_CREATE_NEW | FA_WRITE);
    if (result != FR_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "ERROR: Failed to create file %s: %d\r\n", filename, result);
        usb_debug_print(debug_buffer);
        logging_stats.file_open = 0;
        return 0;
    }

    logging_stats.file_open = 1;
    logging_stats.write_errors = 0; // Reset error count on new file

    snprintf(debug_buffer, sizeof(debug_buffer),
            "Created log file: %s\r\n", filename);
    usb_debug_print(debug_buffer);

    return 1;
}

uint8_t recover_sd_card(void) {
    usb_debug_print("Attempting SD card recovery...\r\n");

    logging_stats.recovery_attempts++;

    // Try to unmount and remount the SD card
    f_mount(NULL, SDPath, 0); // Unmount
    HAL_Delay(500); // Wait a bit

    const FRESULT result = f_mount(&SDFatFS, SDPath, 1); // Remount
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

    // Try to reopen the existing file in append mode
    const FRESULT result = f_open(&dataFile, filename, FA_OPEN_APPEND | FA_WRITE);
    if (result == FR_OK) {
        logging_stats.file_open = 1;
        logging_stats.write_errors = 0; // Reset error count
        usb_debug_print("Successfully reopened log file\r\n");
        return 1;
    } else {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "Failed to reopen file %s: %d\r\n", filename, result);
        usb_debug_print(debug_buffer);

        // If we can't reopen, try creating a new file
        return create_new_log_file();
    }
}

void configure_gnss_for_logging(void) {
    usb_debug_print("Configuring GNSS for RAWX, HPPOSLLH, & PVT logging...\r\n");

    // Set measurement rate to 1Hz
    zedf9p_set_measurement_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, 5000, 1);

    // Configure GPS + Galileo for optimal RAWX data
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

    zedf9p_config_gnss_signals(&gnss_module, UBLOX_CFG_LAYER_RAM, &gnss_config);

    // Disable 7F check for RAWX compatibility
    //zedf9p_disable_7f_check(&gnss_module, true);

    // Enable UBX messages we want to log
    zedf9p_set_message_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, UBX_CLASS_NAV, UBX_NAV_CLOCK, 5);        // NAV-CLOCK
    zedf9p_set_message_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, UBX_CLASS_NAV, UBX_NAV_HPPOSECEF, 5);    // NAV-HPPOSECEF
    zedf9p_set_message_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, 5);     // NAV-HPPOSLLH
    zedf9p_set_message_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, UBX_CLASS_NAV, UBX_NAV_TIMEUTC, 5);      // NAV-TIMEUTC
    zedf9p_set_message_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, UBX_CLASS_RXM, UBX_RXM_RAWX, 5);         // RXM-RAWX
    zedf9p_set_message_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, UBX_CLASS_RXM, UBX_RXM_SFRBX, 5);        // RXM-SFRBX

    HAL_Delay(2000); // Wait for configuration to take effect
    usb_debug_print("GNSS configured for RAWX + HPPOSLLH + PVT logging\r\n");
}

void process_gnss_logging(void) {
    if (!logging_stats.logging_active) {
        return;
    }

    // Process incoming GNSS data
    const zedf9p_status_t status = zedf9p_process_data(&gnss_module);
    if (status != ZEDF9P_OK && status != ZEDF9P_ERR_NO_DATA) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "GNSS processing error: %s\r\n", zedf9p_status_error(status));
        usb_debug_print(debug_buffer);
    }

    // Handle SD card errors and recovery
    if (logging_stats.write_errors > 3) {  // Lower threshold for quicker recovery
        usb_debug_print("Multiple write errors detected, attempting recovery...\r\n");

        // First try to reopen the file
        if (!reopen_log_file()) {
            // If that fails, try SD card recovery
            if (recover_sd_card()) {
                if (!reopen_log_file()) {
                    // Last resort - create new file
                    create_new_log_file();
                }
            } else {
                // SD card recovery failed, pause logging temporarily
                logging_stats.logging_active = 0;
                usb_debug_print("SD card recovery failed, logging paused\r\n");
                return;
            }
        }
    }

    // Try to restart logging if it was paused due to SD card issues
    if (!logging_stats.logging_active && logging_stats.recovery_attempts < 10) {
        if (recover_sd_card() && create_new_log_file()) {
            logging_stats.logging_active = 1;
            usb_debug_print("Logging resumed after recovery\r\n");
        }
    }

    // Check file size limit and rotate if needed
    if (logging_stats.sd_card_present && logging_stats.file_open &&
        f_size(&dataFile) > (MAX_LOG_FILE_SIZE_MB * 1024UL * 1024UL)) {

        snprintf(debug_buffer, sizeof(debug_buffer),
                "File size limit reached, creating new file...\r\n");
        usb_debug_print(debug_buffer);

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

// Generic function to write any UBX message to file
void write_ubx_message_to_file(const ubx_message_t *message) {
    if (!logging_stats.sd_card_present || !logging_stats.logging_active || !logging_stats.file_open) {
        return;
    }

    // Build complete UBX message with header and checksum
    uint8_t ubx_header[6];
    ubx_header[0] = 0xB5; // UBX_SYNCH_1
    ubx_header[1] = 0x62; // UBX_SYNCH_2
    ubx_header[2] = message->msg_class;
    ubx_header[3] = message->msg_id;
    ubx_header[4] = (uint8_t)(message->length & 0xFF);
    ubx_header[5] = (uint8_t)((message->length >> 8) & 0xFF);

    // Calculate checksum for the complete message
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) { // Class, ID, Length
        ck_a += ubx_header[i];
        ck_b += ck_a;
    }
    for (uint16_t i = 0; i < message->length; i++) { // Payload
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
    logging_stats.bytes_logged += 6 + message->length + 2; // Header + payload + checksum
    logging_stats.messages_logged++;

    // Reset write error count on successful write
    if (logging_stats.write_errors > 0) {
        logging_stats.write_errors = 0;
    }
}

// Message callback implementations
void clock_callback(const ubx_message_t *message, void *user_data) {
    (void)user_data; // Unused parameter

    write_ubx_message_to_file(message);
    logging_stats.clock_count++;

    // Optional: Print some debug info about the clock message
    if (logging_stats.usb_ready && message->length >= sizeof(zedf9p_nav_clock_t)) {
        zedf9p_nav_clock_t clock;
        memcpy(&clock, message->payload, sizeof(zedf9p_nav_clock_t));

        snprintf(debug_buffer, sizeof(debug_buffer),
                "CLOCK: iTOW=%lu ClkB=%ld ClkD=%ld TAcc=%lu\r\n",
                clock.i_tow, clock.clk_b, clock.clk_d, clock.t_acc);
        usb_debug_print(debug_buffer);
    }
}

void hpposecef_callback(const ubx_message_t *message, void *user_data) {
    (void)user_data; // Unused parameter

    write_ubx_message_to_file(message);
    logging_stats.hpposecef_count++;

    // Optional: Print some debug info
    if (logging_stats.usb_ready && message->length >= sizeof(zedf9p_nav_hpposecef_t)) {
        zedf9p_nav_hpposecef_t hppos;
        memcpy(&hppos, message->payload, sizeof(zedf9p_nav_hpposecef_t));

        snprintf(debug_buffer, sizeof(debug_buffer),
                "HPPOSECEF: iTOW=%lu X=%ld Y=%ld Z=%ld PAcc=%lu\r\n",
                hppos.i_tow, hppos.ecef_x, hppos.ecef_y, hppos.ecef_z, hppos.p_acc);
        usb_debug_print(debug_buffer);
    }
}

void hpposllh_callback(const ubx_message_t *message, void *user_data) {
    (void)user_data; // Unused parameter

    write_ubx_message_to_file(message);
    logging_stats.hpposllh_count++;
}

void timeutc_callback(const ubx_message_t *message, void *user_data) {
    (void)user_data; // Unused parameter

    write_ubx_message_to_file(message);
    logging_stats.timeutc_count++;

    // Update GNSS time from this message
    if (message->length >= sizeof(zedf9p_nav_timeutc_t)) {
        zedf9p_nav_timeutc_t timeutc;
        memcpy(&timeutc, message->payload, sizeof(zedf9p_nav_timeutc_t));
        update_gnss_time_from_timeutc(&timeutc);
    }
}

void rawx_callback(const ubx_message_t *message, void *user_data) {
    (void)user_data; // Unused parameter

    write_ubx_message_to_file(message);
    logging_stats.rawx_count++;

    // Optional: Print debug info about RAWX message
    if (logging_stats.usb_ready && message->length >= 16) {
        zedf9p_rawx_t rawx;
        memcpy(&rawx, message->payload, sizeof(zedf9p_rawx_t));

        // Print header information
        snprintf(debug_buffer, sizeof(debug_buffer),
                "RAWX: rcTow=%.3f week=%u numMeas=%u recStat=0x%02X\r\n",
                rawx.rc_tow, rawx.week, rawx.num_meas, rawx.rec_stat);
        usb_debug_print(debug_buffer);

        // Print first few satellite measurements as examples (limit to 3 to avoid flooding)
        const uint8_t num_to_print = (rawx.num_meas < 3) ? rawx.num_meas : 3;
        for (uint8_t i = 0; i < num_to_print; i++) {
            const zedf9p_rawx_meas_t *meas = &rawx.meas[i];

            // Determine GNSS system name
            const char* gnss_name = "UNK";
            switch (meas->gnss_id) {
            case 0: gnss_name = "GPS"; break;
            case 1: gnss_name = "SBA"; break;
            case 2: gnss_name = "GAL"; break;
            case 3: gnss_name = "BDS"; break;
            case 5: gnss_name = "QZS"; break;
            case 6: gnss_name = "GLO"; break;
            }

            snprintf(debug_buffer, sizeof(debug_buffer),
                    "  [%u] %s SV=%u sig=%u PR=%.3fm CP=%.3fcyc Dopp=%.2fHz CNO=%udBHz LT=%ums\r\n",
                    i, gnss_name, meas->sv_id, meas->sig_id,
                    meas->pr_mes, meas->cp_mes, meas->do_mes,
                    meas->cno, meas->lock_time);
            usb_debug_print(debug_buffer);
        }

        // If there are more measurements, indicate how many were skipped
        if (rawx.num_meas > num_to_print) {
            snprintf(debug_buffer, sizeof(debug_buffer),
                    "  ... (%u more measurements)\r\n", rawx.num_meas - num_to_print);
            usb_debug_print(debug_buffer);
        }
    }
}

void sfrbx_callback(const ubx_message_t *message, void *user_data) {
    (void)user_data; // Unused parameter

    write_ubx_message_to_file(message);
    logging_stats.sfrbx_count++;

    // // Optional: Print some debug info about SFRBX
    // if (logging_stats.usb_ready && message->length >= sizeof(zedf9p_sfrbx_t)) {
    //     zedf9p_sfrbx_t sfrbx;
    //     memcpy(&sfrbx, message->payload, sizeof(zedf9p_sfrbx_t));
    //
    //     snprintf(debug_buffer, sizeof(debug_buffer),
    //             "SFRBX: GNSS=%d SV=%d NumWords=%d Chn=%d\r\n",
    //             sfrbx.gnss_id, sfrbx.sv_id, sfrbx.num_words, sfrbx.chn);
    //     usb_debug_print(debug_buffer);
    // }
}

void usb_debug_print(const char* message) {
    if (logging_stats.usb_ready && message != NULL) {
        const uint16_t len = strlen(message);
        if (len > 0) {
            // Use CDC_Transmit_FS function to send data via USB CDC
            CDC_Transmit_FS((uint8_t*)message, len);

            // Small delay to prevent overwhelming USB
            HAL_Delay(1);
        }
    }
}

void print_statistics(void) {
    const uint32_t uptime_s = (HAL_GetTick() - logging_stats.session_start_ms) / 1000;
    uint32_t data_rate = 0;

    if (uptime_s > 0) {
        data_rate = logging_stats.bytes_logged / uptime_s;
    }

    snprintf(debug_buffer, sizeof(debug_buffer),
            "Stats: Up=%lus Rate=%luB/s Msgs=%lu CLK=%lu HPECEF=%lu HPLLH=%lu UTC=%lu RAWX=%lu SFRBX=%lu Bytes=%lu Err=%lu\r\n",
            uptime_s, data_rate, logging_stats.messages_logged,
            logging_stats.clock_count, logging_stats.hpposecef_count,
            logging_stats.hpposllh_count, logging_stats.timeutc_count,
            logging_stats.rawx_count, logging_stats.sfrbx_count,
            logging_stats.bytes_logged, logging_stats.write_errors);
    usb_debug_print(debug_buffer);

    // Print GNSS time if available
    if (gnss_time.time_available) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "GNSS Time: %04d-%02d-%02d %02d:%02d:%02d UTC (iTOW: %lu ms)\r\n",
                gnss_time.year, gnss_time.month, gnss_time.day,
                gnss_time.hour, gnss_time.min, gnss_time.sec, gnss_time.i_tow);
        usb_debug_print(debug_buffer);
    }
}

void update_gnss_time_from_timeutc(const zedf9p_nav_timeutc_t *timeutc) {
    if (!timeutc) return;

    gnss_time.i_tow = timeutc->i_tow;
    gnss_time.year = timeutc->year;
    gnss_time.month = timeutc->month;
    gnss_time.day = timeutc->day;
    gnss_time.hour = timeutc->hour;
    gnss_time.min = timeutc->min;
    gnss_time.sec = timeutc->sec;
    gnss_time.valid = timeutc->valid;
    gnss_time.time_available = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
    /* USER CODE BEGIN Boot_Mode_Sequence_0 */
    #if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
      int32_t timeout;
    #endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
    /* USER CODE END Boot_Mode_Sequence_0 */

      /* MPU Configuration--------------------------------------------------------*/
      MPU_Config();

    /* USER CODE BEGIN Boot_Mode_Sequence_1 */
    #if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
      /* Wait until CPU2 boots and enters in stop mode or timeout*/
      timeout = 0xFFFF;
      while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
      if ( timeout < 0 )
      {
      Error_Handler();
      }
    #endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
    /* USER CODE END Boot_Mode_Sequence_1 */
      /* MCU Configuration--------------------------------------------------------*/

      /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
      HAL_Init();

      /* USER CODE BEGIN Init */

      /* USER CODE END Init */

      /* Configure the system clock */
      SystemClock_Config();
    /* USER CODE BEGIN Boot_Mode_Sequence_2 */
    #if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
    /* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
    HSEM notification */
    /*HW semaphore Clock enable*/
    __HAL_RCC_HSEM_CLK_ENABLE();
    /*Take HSEM */
    HAL_HSEM_FastTake(HSEM_ID_0);
    /*Release HSEM in order to notify the CPU2(CM4)*/
    HAL_HSEM_Release(HSEM_ID_0,0);
    /* wait until CPU2 wakes up from stop mode */
    timeout = 0xFFFF;
    while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
    if ( timeout < 0 )
    {
    Error_Handler();
    }
    #endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
    /* USER CODE END Boot_Mode_Sequence_2 */

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_SDMMC2_SD_Init();
    MX_UART4_Init();
    MX_FATFS_Init();
    MX_USB_DEVICE_Init();

    /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(L4_RESET_GPIO_Port, L4_RESET_Pin, GPIO_PIN_RESET); // Set pin low to make sure the reset line stays high
    HAL_GPIO_WritePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin, GPIO_PIN_SET); // LED on
    HAL_Delay(5000);
    //HAL_GPIO_WritePin(GNSS_ENABLE_GPIO_Port, GNSS_ENABLE_Pin, GPIO_PIN_SET); // Turn on the GNSS load switch

    // Initialize USB hub
    usb2422_platform_set_i2c_handle(USB2422_I2C);

    const usb2422_interface_t io_usb = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read_reg,
        .delay_ms = platform_delay_ms,
    };

    usb2422_status_t status_usb = usb2422_init(&usb_hub, USB2422_SMBUS_ADDRESS, io_usb);
    if (status_usb != USB2422_OK) {
        while(1) {
            HAL_GPIO_TogglePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin);
            HAL_Delay(100);
        }
    }

    status_usb = configure_usb2422_for_enumeration(&usb_hub);
    if (status_usb != USB2422_OK) {
        while(1) {
            HAL_GPIO_TogglePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin);
            HAL_Delay(100);
        }
    }

    status_usb = usb2422_usb_attach_and_protect(&usb_hub, &hub_settings);
    if (status_usb != USB2422_OK) {
        while(1) {
            HAL_GPIO_TogglePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin);
            HAL_Delay(100);
        }
    }

    logging_stats.usb_ready = 1;

    HAL_Delay(5000);

    usb_debug_print("ZEDF9P GNSS Data Logger\r\n");
    usb_debug_print("=======================\r\n");


    // Initialize SD card with FatFS
    if (!init_sd_card()) {
        usb_debug_print("FATAL: SD card initialization failed!\r\n");
        while(1) {
            HAL_GPIO_TogglePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin);
            HAL_Delay(500);
        }
    }

    // Initialize GNSS module
    zedf9p_status_t status;
    if (zedf9p_use_i2c) {
        zedf9p_platform_set_i2c_handle(ZEDF9P_I2C);

        const zedf9p_interface_t io = {
            .i2c_write = platform_i2c_write,
            .i2c_read = platform_i2c_read,
            .uart_write = NULL,
            .uart_read = NULL,
            .delay_ms = platform_delay_ms,
            .get_millis = platform_get_millis,
            .debug_print = NULL
        };

        status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_I2C, 0x42, io);
        if (status != ZEDF9P_OK) {
            snprintf(debug_buffer, sizeof(debug_buffer),
                    "FATAL: GNSS init failed: %s\r\n", zedf9p_status_error(status));
            usb_debug_print(debug_buffer);
            while(1) {
                HAL_GPIO_TogglePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin);
                HAL_Delay(100);
            }
        }
    } else {
        zedf9p_platform_set_uart_handle(ZEDF9P_UART);

        const zedf9p_interface_t io = {
            .i2c_write = NULL,
            .i2c_read = NULL,
            .uart_write = platform_uart_write,
            .uart_read = platform_uart_read,
            .delay_ms = platform_delay_ms,
            .get_millis = platform_get_millis,
            .debug_print = NULL
        };

        status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_UART, 0, io);
    }

    if (status != ZEDF9P_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "FATAL: GNSS init failed: %s\r\n", zedf9p_status_error(status));
        usb_debug_print(debug_buffer);
        while(1) {
            HAL_GPIO_TogglePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin);
            HAL_Delay(100);
        }
    }

    usb_debug_print("GNSS module initialized successfully\r\n");

    // Configure GNSS for logging
    configure_gnss_for_logging();

    // Create initial log file
    if (!create_new_log_file()) {
        usb_debug_print("FATAL: Failed to create log file!\r\n");
        while(1) {
            HAL_GPIO_TogglePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin);
            HAL_Delay(100);
        }
    }

    // Register message callbacks
    zedf9p_register_clock_callback(&gnss_module, clock_callback, NULL);
    zedf9p_register_hpposecef_callback(&gnss_module, hpposecef_callback, NULL);
    zedf9p_register_hpposllh_callback(&gnss_module, hpposllh_callback, NULL);
    zedf9p_register_timeutc_callback(&gnss_module, timeutc_callback, NULL);
    zedf9p_register_rawx_callback(&gnss_module, rawx_callback, NULL);
    zedf9p_register_sfrbx_callback(&gnss_module, sfrbx_callback, NULL);

    // Initialize logging
    logging_stats.session_start_ms = HAL_GetTick();
    logging_stats.last_flush_ms = HAL_GetTick();
    logging_stats.last_stats_ms = HAL_GetTick();
    logging_stats.logging_active = 1;
    logging_stats.file_open = 1;

    usb_debug_print("Logging started!\r\n");
    usb_debug_print("================\r\n");

    HAL_GPIO_WritePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin, GPIO_PIN_RESET);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */
        // Process GNSS data
        process_gnss_logging();

        // Print statistics every 10 seconds
        if (HAL_GetTick() - logging_stats.last_stats_ms > 10000) {
            print_statistics();
            HAL_GPIO_TogglePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin); // Blink LED
            logging_stats.last_stats_ms = HAL_GetTick();
        }

        // Periodic flush
        if (HAL_GetTick() - logging_stats.last_flush_ms > FLUSH_INTERVAL_MS) {
            if (logging_stats.sd_card_present && logging_stats.logging_active && logging_stats.file_open) {
                const FRESULT flush_result = f_sync(&dataFile);
                if (flush_result != FR_OK) {
                    snprintf(debug_buffer, sizeof(debug_buffer),
                            "File sync error: %d\r\n", flush_result);
                    usb_debug_print(debug_buffer);
                    logging_stats.write_errors++;
                } else {
                    logging_stats.last_flush_ms = HAL_GetTick();
                }
            }
        }

        // Small delay to prevent overwhelming the system
        HAL_Delay(1);
    /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 8;
  if (HAL_SD_Init(&hsd2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GNSS_ENABLE_Pin|GNSS_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(H7_LEDBUILTIN_GPIO_Port, H7_LEDBUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GNSS_ENABLE_Pin */
  GPIO_InitStruct.Pin = GNSS_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GNSS_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GNSS_TXRDY_Pin GNSS_INT_Pin GNSS_PPS_Pin */
  GPIO_InitStruct.Pin = GNSS_TXRDY_Pin|GNSS_INT_Pin|GNSS_PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : GNSS_RESET_Pin */
  GPIO_InitStruct.Pin = GNSS_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GNSS_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L4_RESET_Pin */
  GPIO_InitStruct.Pin = L4_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(L4_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L4_BOOT_Pin */
  GPIO_InitStruct.Pin = L4_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(L4_BOOT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : H7_LEDBUILTIN_Pin */
  GPIO_InitStruct.Pin = H7_LEDBUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(H7_LEDBUILTIN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
