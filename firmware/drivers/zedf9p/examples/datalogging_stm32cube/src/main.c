/*!
 * @file main.c
 * @brief ZEDF9P GNSS Data Logger using STM32Cube HAL with SDIO
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-09-14
 *
 * This example logs GNSS data to SD card via SDIO using STM32Cube HAL.
 * Designed for SparkFun STM32 Thing Plus (STM32F405RGT6).
 *
 * Hardware Configuration:
 * - SDIO: PC8-PC12, PD2 (hardwired on board)
 * - I2C1: PB6 (SCL), PB7 (SDA) for GNSS
 * - UART2: PA2 (TX), PA3 (RX) for debug
 */

#include "main.h"
#include "fatfs.h"
#include "sdio.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

// Include your ZEDF9P driver
#include "zedf9p.h"
#include "zedf9p_platform.h"

// Configuration
#define LOG_FILENAME_PREFIX "GNSS"
#define LOG_FILENAME_EXTENSION ".ubx"
#define MAX_LOG_FILE_SIZE_MB 100
#define FLUSH_INTERVAL_MS 5000
#define RAW_BUFFER_SIZE 512

// Global variables
FATFS SDFatFS;
FIL dataFile;
char SDPath[4];
zedf9p_t gnss_module;

// Buffers
uint8_t raw_buffer[RAW_BUFFER_SIZE];
char filename[32];
char debug_buffer[256];

// Statistics
struct {
    uint32_t session_start_ms;
    uint32_t last_flush_ms;
    uint32_t bytes_logged;
    uint32_t file_number;
    uint8_t logging_active;
    uint8_t sd_card_present;
    uint32_t write_errors;
} logging_stats = {0};

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

uint8_t init_sd_card(void);
uint8_t create_new_log_file(void);
void configure_gnss_for_logging(void);
void process_gnss_logging(void);
void print_statistics(void);
void debug_print(const char* message);

// Platform interface implementations for STM32Cube
int platform_i2c_write_cube(uint8_t dev_addr, const uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1, (uint8_t*)data, len, 1000);
    return (status == HAL_OK) ? 0 : -1;
}

int platform_i2c_read_cube(uint8_t dev_addr, uint8_t *data, uint16_t len) {
    HAL_Delay(2); // Small delay before read
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, dev_addr << 1, data, len, 1000);
    if (status == HAL_OK) {
        return len;
    }
    return 0; // Return 0 bytes read on failure
}

void platform_delay_ms_cube(uint32_t ms) {
    HAL_Delay(ms);
}

uint32_t platform_get_millis_cube(void) {
    return HAL_GetTick();
}

void platform_debug_print_cube(const char *message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
}

int main(void) {
    // Initialize HAL
    HAL_Init();
    SystemClock_Config();

    // Initialize peripherals
    MX_GPIO_Init();
    MX_SDIO_SD_Init();
    MX_FATFS_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    // Turn on status LED
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED on

    debug_print("ZEDF9P GNSS Data Logger (STM32Cube HAL)\r\n");
    debug_print("======================================\r\n");

    // Initialize SD card with FatFS
    if (!init_sd_card()) {
        debug_print("FATAL: SD card initialization failed!\r\n");
        while(1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(500);
        }
    }

    // Initialize GNSS module
    zedf9p_interface_t io = {
        .i2c_write = platform_i2c_write_cube,
        .i2c_read = platform_i2c_read_cube,
        .uart_write = NULL,
        .uart_read = NULL,
        .delay_ms = platform_delay_ms_cube,
        .get_millis = platform_get_millis_cube,
        .debug_print = NULL // Disable for performance
    };

    zedf9p_status_t status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_I2C, 0x42, io);
    if (status != ZEDF9P_OK) {
        sprintf(debug_buffer, "FATAL: GNSS init failed: %s\r\n", zedf9p_status_error(status));
        debug_print(debug_buffer);
        while(1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(1000);
        }
    }

    debug_print("GNSS module initialized successfully\r\n");

    // Configure GNSS for logging
    configure_gnss_for_logging();

    // Create initial log file
    if (!create_new_log_file()) {
        debug_print("FATAL: Failed to create log file!\r\n");
        while(1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(1000);
        }
    }

    // Initialize logging
    logging_stats.session_start_ms = HAL_GetTick();
    logging_stats.last_flush_ms = HAL_GetTick();
    logging_stats.logging_active = 1;

    debug_print("STM32Cube SDIO logging started!\r\n");
    debug_print("================================\r\n");

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED off - ready

    // Main loop
    uint32_t last_stats = 0;
    while (1) {
        // Process GNSS data logging
        process_gnss_logging();

        // Print statistics every 10 seconds
        if (HAL_GetTick() - last_stats > 10000) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Blink LED
            print_statistics();
            last_stats = HAL_GetTick();
        }

        // Small delay
        HAL_Delay(1);
    }
}

uint8_t init_sd_card(void) {
    debug_print("Initializing SD card via SDIO...\r\n");

    // Mount the SD card
    FRESULT result = f_mount(&SDFatFS, SDPath, 1);
    if (result != FR_OK) {
        sprintf(debug_buffer, "SD mount failed: %d\r\n", result);
        debug_print(debug_buffer);
        return 0;
    }

    // Get SD card info
    DWORD free_clusters;
    FATFS* fs;
    result = f_getfree(SDPath, &free_clusters, &fs);
    if (result == FR_OK) {
        uint32_t total_sectors = (fs->n_fatent - 2) * fs->csize;
        uint32_t free_sectors = free_clusters * fs->csize;

        sprintf(debug_buffer, "SD Card: %lu MB total, %lu MB free\r\n",
                total_sectors / 2048, free_sectors / 2048);
        debug_print(debug_buffer);
    }

    logging_stats.sd_card_present = 1;
    debug_print("SD card initialized successfully\r\n");
    return 1;
}

uint8_t create_new_log_file(void) {
    if (!logging_stats.sd_card_present) {
        return 0;
    }

    // Close existing file
    f_close(&dataFile);

    // Find next available filename
    FRESULT result;
    do {
        logging_stats.file_number++;
        sprintf(filename, "%s%03lu%s", LOG_FILENAME_PREFIX,
                logging_stats.file_number, LOG_FILENAME_EXTENSION);

        // Check if file exists
        result = f_stat(filename, NULL);
    } while (result == FR_OK); // Continue if file exists

    // Create new file
    result = f_open(&dataFile, filename, FA_CREATE_NEW | FA_WRITE);
    if (result != FR_OK) {
        sprintf(debug_buffer, "ERROR: Failed to create file %s: %d\r\n", filename, result);
        debug_print(debug_buffer);
        return 0;
    }

    sprintf(debug_buffer, "Created log file: %s\r\n", filename);
    debug_print(debug_buffer);

    return 1;
}

void configure_gnss_for_logging(void) {
    debug_print("Configuring GNSS for raw data logging...\r\n");

    // Set measurement rate
    zedf9p_set_measurement_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, 1000, 1);

    // Configure GPS + Galileo
    zedf9p_gnss_config_t gnss_config = {
        .gps_enabled = 1,
        .gps_l1ca_enabled = 1,
        .gps_l2c_enabled = 1,
        .galileo_enabled = 1,
        .galileo_e1_enabled = 1,
        .galileo_e5b_enabled = 1,
        .glonass_enabled = 0,
        .glonass_l1_enabled = 0,
        .glonass_l2_enabled = 0,
        .beidou_enabled = 0,
        .beidou_b1_enabled = 0,
        .beidou_b2_enabled = 0,
        .qzss_enabled = 0,
        .qzss_l1ca_enabled = 0,
        .qzss_l1s_enabled = 0,
        .qzss_l2c_enabled = 0,
        .sbas_enabled = 0,
        .sbas_l1ca_enabled = 0
    };

    zedf9p_config_gnss_signals(&gnss_module, UBLOX_CFG_LAYER_RAM, &gnss_config);

    // Enable UBX messages
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_RXM, UBX_RXM_RAWX, 1);
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, 1);
    zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_PVT, 1);

    HAL_Delay(2000);
    debug_print("GNSS configured for raw logging\r\n");
}

void process_gnss_logging(void) {
    if (!logging_stats.logging_active) {
        return;
    }

    // Read raw bytes from I2C
    int bytes_read = platform_i2c_read_cube(0x42, raw_buffer, sizeof(raw_buffer));

    if (bytes_read > 0) {
        // Write raw bytes to SD card using FatFS
        UINT bytes_written;
        FRESULT result = f_write(&dataFile, raw_buffer, bytes_read, &bytes_written);

        if (result == FR_OK && bytes_written == bytes_read) {
            logging_stats.bytes_logged += bytes_written;
        } else {
            logging_stats.write_errors++;
            sprintf(debug_buffer, "Write error: %d, wrote %u/%d\r\n", result, bytes_written, bytes_read);
            debug_print(debug_buffer);

            if (logging_stats.write_errors > 20) {
                logging_stats.logging_active = 0;
                debug_print("Too many write errors - stopping logging\r\n");
            }
        }

        // Periodic flush
        if (HAL_GetTick() - logging_stats.last_flush_ms > FLUSH_INTERVAL_MS) {
            f_sync(&dataFile);
            logging_stats.last_flush_ms = HAL_GetTick();
        }

        // Check file size limit
        if (f_size(&dataFile) > (MAX_LOG_FILE_SIZE_MB * 1024UL * 1024UL)) {
            f_close(&dataFile);
            if (!create_new_log_file()) {
                debug_print("ERROR: Failed to create new log file\r\n");
                logging_stats.logging_active = 0;
            }
        }
    }
}

void print_statistics(void) {
    uint32_t uptime_s = (HAL_GetTick() - logging_stats.session_start_ms) / 1000;
    uint32_t data_rate = 0;

    if (uptime_s > 0) {
        data_rate = logging_stats.bytes_logged / uptime_s;
    }

    sprintf(debug_buffer, "Up:%lus Rate:%luB/s Bytes:%lu Errors:%lu\r\n",
            uptime_s, data_rate, logging_stats.bytes_logged, logging_stats.write_errors);
    debug_print(debug_buffer);
}

void debug_print(const char* message) {
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
}

// STM32Cube auto-generated initialization functions
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_SDIO_SD_Init(void) {
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 0;
    HAL_SD_Init(&hsd);
    HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Status LED (PC13)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}