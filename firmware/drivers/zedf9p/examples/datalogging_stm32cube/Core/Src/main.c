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
#include "zedf9p.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct {
    uint32_t session_start_ms;
    uint32_t last_flush_ms;
    uint32_t last_stats_ms;
    uint32_t bytes_logged;
    uint32_t messages_logged;
    uint32_t clock_count;
    uint32_t hpposecef_count;
    uint32_t hpposllh_count;
    uint32_t timeutc_count;
    uint32_t rawx_count;
    uint32_t sfrbx_count;
    uint32_t file_number;
    uint8_t logging_active;
    uint8_t sd_card_present;
    uint32_t write_errors;
    uint32_t recovery_attempts;
    uint8_t usb_ready;
    uint8_t file_open;
} logging_stats = {0};

// GNSS time tracking
struct {
    uint32_t i_tow;        // GPS time of week (ms)
    uint16_t year;         // Year (UTC)
    uint8_t month;         // Month (UTC)
    uint8_t day;           // Day (UTC)
    uint8_t hour;          // Hour (UTC)
    uint8_t min;           // Minute (UTC)
    uint8_t sec;           // Second (UTC)
    uint8_t valid;         // Time validity flags
    uint8_t time_available;
} gnss_time = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_FILENAME_PREFIX "GNSS"
#define LOG_FILENAME_EXTENSION ".ubx"
#define MAX_LOG_FILE_SIZE_MB 100
#define FLUSH_INTERVAL_MS 5000
#define PROCESS_BUFFER_SIZE 1024
#define USB_DEBUG_BUFFER_SIZE 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

/* USER CODE BEGIN PV */
FIL dataFile;
zedf9p_t gnss_module;

// Buffers
uint8_t process_buffer[PROCESS_BUFFER_SIZE];
char filename[32];
char debug_buffer[USB_DEBUG_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);

/* USER CODE BEGIN PFP */
uint8_t init_sd_card(void);
uint8_t create_new_log_file(void);
uint8_t recover_sd_card(void);
uint8_t reopen_log_file(void);
void configure_gnss_for_logging(void);
void process_gnss_logging(void);
void print_statistics(void);
void usb_debug_print(const char* message);
void update_gnss_time_from_timeutc(const zedf9p_nav_timeutc_t *timeutc);

// Generic UBX message writing function
void write_ubx_message_to_file(const ubx_message_t *message);

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
// Platform interface implementations for STM32Cube
int platform_i2c_write_cube(const uint8_t dev_addr, const uint8_t *data, const uint16_t len) {
    const HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1,
        (uint8_t*)data, len, 1000);
    return (status == HAL_OK) ? 0 : -1;
}

int platform_i2c_read_cube(const uint8_t dev_addr, uint8_t *data, const uint16_t len) {
    HAL_Delay(1); // Small delay before read
    const HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, dev_addr << 1,
        data, len, 1000);
    if (status == HAL_OK) {
        return len;
    }
    return 0;
}

void platform_delay_ms_cube(const uint32_t ms) {
    HAL_Delay(ms);
}

uint32_t platform_get_millis_cube(void) {
    return HAL_GetTick();
}

void platform_debug_print_cube(const char *message) {
    // Only print debug messages if USB is ready and logging is active
    if (logging_stats.usb_ready) {
        usb_debug_print(message);
    }
}

uint8_t init_sd_card(void) {
    usb_debug_print("Initializing SD card via SDIO...\r\n");

    // Multiple hardware initialization attempts
    for (int attempt = 0; attempt < 3; attempt++) {
        HAL_Delay(100 * (attempt + 1)); // Increasing delay between attempts

        snprintf(debug_buffer, sizeof(debug_buffer),
                "SD hardware init attempt %d...\r\n", attempt + 1);
        usb_debug_print(debug_buffer);

        // Try to initialize SD hardware
        const HAL_StatusTypeDef hal_result = HAL_SD_Init(&hsd);
        if (hal_result != HAL_OK) {
            snprintf(debug_buffer, sizeof(debug_buffer),
                    "HAL_SD_Init failed with code: %d\r\n", hal_result);
            usb_debug_print(debug_buffer);
            continue;
        }

        // Check if card is ready
        const HAL_SD_CardStateTypeDef cardState = HAL_SD_GetCardState(&hsd);
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
    usb_debug_print("Configuring GNSS for UBX message logging...\r\n");

    // Set measurement rate to 1Hz
    //zedf9p_set_measurement_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, 1000, 1);

    // Configure GPS + Galileo for optimal data
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

    //zedf9p_config_gnss_signals(&gnss_module, UBLOX_CFG_LAYER_RAM, &gnss_config);

    // Disable 7F check for RAWX compatibility
    //zedf9p_disable_7f_check(&gnss_module, true);

    // Enable the specific UBX messages we want to log (1Hz rate)
    // Now using CFG-VALSET internally via the updated zedf9p_set_message_rate function
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_CLOCK, 5);     // NAV-CLOCK
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_HPPOSECEF, 5); // NAV-HPPOSECEF
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, 5);  // NAV-HPPOSLLH
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_TIMEUTC, 5);   // NAV-TIMEUTC
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_RXM, UBX_RXM_RAWX, 5);      // RXM-RAWX
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_RXM, UBX_RXM_SFRBX, 5);     // RXM-SFRBX

    HAL_Delay(2000); // Wait for configuration to take effect
    usb_debug_print("GNSS configured for UBX message logging (using CFG-VALSET)\r\n");
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
}

void sfrbx_callback(const ubx_message_t *message, void *user_data) {
    (void)user_data; // Unused parameter

    write_ubx_message_to_file(message);
    logging_stats.sfrbx_count++;

    // Optional: Print some debug info about SFRBX
    if (logging_stats.usb_ready && message->length >= sizeof(zedf9p_sfrbx_t)) {
        zedf9p_sfrbx_t sfrbx;
        memcpy(&sfrbx, message->payload, sizeof(zedf9p_sfrbx_t));

        snprintf(debug_buffer, sizeof(debug_buffer),
                "SFRBX: GNSS=%d SV=%d NumWords=%d Chn=%d\r\n",
                sfrbx.gnss_id, sfrbx.sv_id, sfrbx.num_words, sfrbx.chn);
        usb_debug_print(debug_buffer);
    }
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

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_SDIO_SD_Init();
    MX_FATFS_Init();
    MX_USB_DEVICE_Init();

    /* USER CODE BEGIN 2 */
    // Turn on status LED
    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET); // LED on
    HAL_Delay(5000);
    logging_stats.usb_ready = 1;

    usb_debug_print("ZEDF9P UBX Message Logger\r\n");
    usb_debug_print("=========================\r\n");

    // Initialize SD card with FatFS
    if (!init_sd_card()) {
        usb_debug_print("FATAL: SD card initialization failed!\r\n");
        while(1) {
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
            HAL_Delay(500);
        }
    }

    // Initialize GNSS module
    const zedf9p_interface_t io = {
        .i2c_write = platform_i2c_write_cube,
        .i2c_read = platform_i2c_read_cube,
        .uart_write = NULL,
        .uart_read = NULL,
        .delay_ms = platform_delay_ms_cube,
        .get_millis = platform_get_millis_cube,
        .debug_print = NULL
    };

    const zedf9p_status_t status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_I2C, 0x42, io);
    if (status != ZEDF9P_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "FATAL: GNSS init failed: %s\r\n", zedf9p_status_error(status));
        usb_debug_print(debug_buffer);
        while(1) {
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
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
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
            HAL_Delay(100);
        }
    }

    // Register message callbacks for the specific UBX messages
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

    usb_debug_print("UBX Message Logging started!\r\n");
    usb_debug_print("============================\r\n");

    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);

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
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin); // Blink LED
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 8;
  /* USER CODE BEGIN SDIO_Init 2 */
    if (HAL_SD_Init(&hsd) != HAL_OK) {
        Error_Handler();
    }

    // Switch to 4-bit after successful init
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
        Error_Handler();
    }
  /* USER CODE END SDIO_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USR_LED_Pin */
  GPIO_InitStruct.Pin = USR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USR_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
