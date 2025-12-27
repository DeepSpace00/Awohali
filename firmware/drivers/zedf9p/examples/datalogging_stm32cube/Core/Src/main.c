/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - GNSS Logger with I2C and UART modes
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
#include <inttypes.h>

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
    uint32_t file_number;
    uint8_t logging_active;
    uint8_t sd_card_present;
    uint32_t write_errors;
    uint32_t recovery_attempts;
    uint8_t usb_ready;
    uint8_t file_open;
} logging_stats = {0};

// Optional: GNSS time tracking (only used if you want to display time)
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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ============================================================================
// LOGGING MODE SELECTION - CHANGE THIS TO SWITCH BETWEEN I2C AND UART
// ============================================================================
#define USE_UART_LOGGING 0  // Set to 1 for UART DMA mode, 0 for I2C callback mode

#define LOG_FILENAME_PREFIX "GNSS"
#define LOG_FILENAME_EXTENSION ".ubx"
#define MAX_LOG_FILE_SIZE_MB 100
#define FLUSH_INTERVAL_MS 5000
#define USB_DEBUG_BUFFER_SIZE 512

#if USE_UART_LOGGING
#define UART_DMA_BUFFER_SIZE 2048
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

#if USE_UART_LOGGING
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
#endif

/* USER CODE BEGIN PV */
FIL dataFile;
zedf9p_t gnss_module;

// Buffers
char filename[32];
char debug_buffer[USB_DEBUG_BUFFER_SIZE];

#if USE_UART_LOGGING
// Double buffer for continuous UART DMA reception
uint8_t uart_buffer_0[UART_DMA_BUFFER_SIZE];
uint8_t uart_buffer_1[UART_DMA_BUFFER_SIZE];
volatile uint8_t active_buffer = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
#if USE_UART_LOGGING
static void MX_USART2_UART_Init(void);
#endif

/* USER CODE BEGIN PFP */
uint8_t init_sd_card(void);
uint8_t create_new_log_file(void);
uint8_t recover_sd_card(void);
uint8_t reopen_log_file(void);
void configure_gnss_for_logging(void);
void process_gnss_logging(void);
void print_statistics(void);
void usb_debug_print(const char* message);

#if !USE_UART_LOGGING
// I2C mode: Generic callback for all UBX messages
void generic_ubx_callback(const ubx_message_t *message, const void *user_data);
void write_ubx_message_to_file(const ubx_message_t *message);
void update_gnss_time_from_timeutc(const ubx_message_t *message);
#endif
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
    HAL_Delay(1);
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
    if (logging_stats.usb_ready) {
        usb_debug_print(message);
    }
}

uint8_t init_sd_card(void) {
    usb_debug_print("Initializing SD card via SDIO...\r\n");

    // Multiple hardware initialization attempts
    for (int attempt = 0; attempt < 3; attempt++) {
        HAL_Delay(100 * (attempt + 1));

        snprintf(debug_buffer, sizeof(debug_buffer),
                "SD hardware init attempt %d...\r\n", attempt + 1);
        usb_debug_print(debug_buffer);

        const HAL_StatusTypeDef hal_result = HAL_SD_Init(&hsd);
        if (hal_result != HAL_OK) {
            snprintf(debug_buffer, sizeof(debug_buffer),
                    "HAL_SD_Init failed with code: %d\r\n", hal_result);
            usb_debug_print(debug_buffer);
            continue;
        }

        const HAL_SD_CardStateTypeDef cardState = HAL_SD_GetCardState(&hsd);
        snprintf(debug_buffer, sizeof(debug_buffer),
                "SD card state: %"PRIx32"\r\n", cardState);
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

    HAL_Delay(500);

    usb_debug_print("Attempting to mount filesystem...\r\n");
    FRESULT result = f_mount(&SDFatFS, SDPath, 1);

    if (result != FR_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "Initial SD mount failed: %d\r\n", result);
        usb_debug_print(debug_buffer);

        usb_debug_print("Attempting to format SD card as FAT32...\r\n");
        BYTE work[_MAX_SS];
        result = f_mkfs(SDPath, FM_FAT32, 0, work, sizeof(work));

        if (result == FR_OK) {
            usb_debug_print("SD card formatted successfully\r\n");
            HAL_Delay(100);

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

    // Get SD card info
    DWORD free_clusters;
    FATFS* fs;
    result = f_getfree(SDPath, &free_clusters, &fs);
    if (result == FR_OK) {
        const DWORD total_clusters = (fs->n_fatent - 2) * fs->csize;
        const uint32_t total_mb = total_clusters / 2048;
        const uint32_t free_mb = free_clusters * fs->csize / 2048;

        snprintf(debug_buffer, sizeof(debug_buffer),
                "SD Card: %"PRIx32" MB total, %"PRIx32" MB free\r\n", total_mb, free_mb);
        usb_debug_print(debug_buffer);

        snprintf(debug_buffer, sizeof(debug_buffer),
                "Filesystem: FAT%d, Sector size: %d, Cluster size: %d\r\n",
                fs->fs_type, _MAX_SS, fs->csize * _MAX_SS);
        usb_debug_print(debug_buffer);
    }

    logging_stats.sd_card_present = 1;
    usb_debug_print("SD card initialization completed successfully\r\n");
    return 1;
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
        snprintf(filename, sizeof(filename), "%s%03"PRIx32"%s",
                LOG_FILENAME_PREFIX, logging_stats.file_number, LOG_FILENAME_EXTENSION);
        result = f_stat(filename, NULL);
    } while (result == FR_OK);

    result = f_open(&dataFile, filename, FA_CREATE_NEW | FA_WRITE);
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

    f_mount(NULL, SDPath, 0);
    HAL_Delay(500);

    const FRESULT result = f_mount(&SDFatFS, SDPath, 1);
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

    // Set measurement rate if desired
    // zedf9p_set_measurement_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, 1000, 1);

    // Configure GNSS constellations and signals
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

    // Configure GNSS signals if desired
    zedf9p_config_gnss_signals(&gnss_module, UBLOX_CFG_LAYER_RAM, &gnss_config);

    // Configure messages
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_CLOCK, UBLOX_CFG_LAYER_RAM, 1);
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_TIMEUTC, UBLOX_CFG_LAYER_RAM, 1);
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_RXM, UBX_RXM_RAWX, UBLOX_CFG_LAYER_RAM, 1);
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_RXM, UBX_RXM_SFRBX, UBLOX_CFG_LAYER_RAM, 1);

    HAL_Delay(2000);
    usb_debug_print("GNSS configured for UBX message logging\r\n");
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
            HAL_UART_Receive_DMA(&huart2, uart_buffer_0, UART_DMA_BUFFER_SIZE);
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
// I2C mode: Write UBX message to SD card
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

// Generic callback for ALL UBX messages
void generic_ubx_callback(const ubx_message_t *message, const void *user_data) {
    (void)user_data;

    // Write to SD card
    write_ubx_message_to_file(message);

    // Optional: Update GNSS time if this is a TIMEUTC message
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

#if USE_UART_LOGGING
// UART DMA Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2 && logging_stats.logging_active && logging_stats.file_open) {
        UINT bytes_written;

        // Write the completed buffer to SD card
        uint8_t *completed_buffer = (active_buffer == 0) ? uart_buffer_0 : uart_buffer_1;
        FRESULT result = f_write(&dataFile, completed_buffer, UART_DMA_BUFFER_SIZE,
                                &bytes_written);

        if (result == FR_OK && bytes_written == UART_DMA_BUFFER_SIZE) {
            logging_stats.bytes_logged += bytes_written;
            logging_stats.messages_logged++; // Approximate - this is buffer count

            // Reset write errors on success
            if (logging_stats.write_errors > 0) {
                logging_stats.write_errors = 0;
            }
        } else {
            logging_stats.write_errors++;
        }

        // Switch to other buffer and restart DMA
        active_buffer = !active_buffer;
        uint8_t *next_buffer = (active_buffer == 0) ? uart_buffer_0 : uart_buffer_1;
        HAL_UART_Receive_DMA(&huart2, next_buffer, UART_DMA_BUFFER_SIZE);
    }
}
#endif

void usb_debug_print(const char *message) {
    if (logging_stats.usb_ready && message != NULL) {
        const uint16_t len = strlen(message);
        if (len > 0) {
            CDC_Transmit_FS((uint8_t*)message, len);
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
            "Stats: Up=%"PRIx32"s Rate=%"PRIx32"B/s Msgs=%"PRIx32" Bytes=%"PRIx32" Err=%"PRIx32"\r\n",
            uptime_s, data_rate, logging_stats.messages_logged,
            logging_stats.bytes_logged, logging_stats.write_errors);
    usb_debug_print(debug_buffer);

#if !USE_UART_LOGGING
    // Print GNSS time if available (I2C mode only)
    if (gnss_time.time_available) {
        snprintf(debug_buffer, sizeof(debug_buffer),
                "GNSS Time: %04d-%02d-%02d %02d:%02d:%02d UTC (iTOW: %"PRIx32" ms)\r\n",
                gnss_time.year, gnss_time.month, gnss_time.day,
                gnss_time.hour, gnss_time.min, gnss_time.sec, gnss_time.i_tow);
        usb_debug_print(debug_buffer);
    }
#endif
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
#if USE_UART_LOGGING
  MX_USART2_UART_Init();
#endif

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
  HAL_Delay(5000);
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
          HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
          HAL_Delay(500);
      }
  }

#if !USE_UART_LOGGING
  // I2C mode: Initialize GNSS module with I2C interface
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

  usb_debug_print("GNSS module initialized successfully (I2C mode)\r\n");
#else
  // UART mode: Initialize GNSS module with UART interface for configuration only
  // (We still need I2C to configure the module to output on UART)
  const zedf9p_interface_t io = {
      .i2c_write = platform_i2c_write_cube,
      .i2c_read = platform_i2c_read_cube,
      .uart_write = NULL,
      .uart_read = NULL,
      .delay_ms = platform_delay_ms_cube,
      .get_millis = platform_get_millis_cube,
      .debug_print = NULL
  };

  // Initialize as UART interface so set_message_rate configures UART1 output
  const zedf9p_status_t status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_UART, 0x42, io);
  if (status != ZEDF9P_OK) {
      snprintf(debug_buffer, sizeof(debug_buffer),
              "FATAL: GNSS init failed: %s\r\n", zedf9p_status_error(status));
      usb_debug_print(debug_buffer);
      while(1) {
          HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
          HAL_Delay(100);
      }
  }

  usb_debug_print("GNSS module initialized successfully (UART mode)\r\n");
#endif

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

#if !USE_UART_LOGGING
  // I2C mode: Register generic callback for ALL messages
  zedf9p_register_generic_callback(&gnss_module, generic_ubx_callback, NULL);
  usb_debug_print("Registered generic callback for all UBX messages\r\n");
#else
  // UART mode: Start DMA reception
  HAL_UART_Receive_DMA(&huart2, uart_buffer_0, UART_DMA_BUFFER_SIZE);
  usb_debug_print("Started UART DMA reception\r\n");
#endif

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
          HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
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

#if USE_UART_LOGGING
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200; // Match ZED-F9P UART baud rate
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
#endif

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
#if USE_UART_LOGGING
  __HAL_RCC_DMA1_CLK_ENABLE();
#endif

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

#if USE_UART_LOGGING
  /* DMA1_Stream5_IRQn interrupt configuration - USART2_RX */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
#endif

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