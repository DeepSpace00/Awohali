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
#include "zedf9p_platform.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_FILENAME_PREFIX "GNSS"
#define LOG_FILENAME_EXTENSION ".ubx"
#define MAX_LOG_FILE_SIZE_MB 100
#define FLUSH_INTERVAL_MS 5000
#define RAW_BUFFER_SIZE 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;

/* USER CODE BEGIN PV */
uint8_t raw_buffer[RAW_BUFFER_SIZE];
char filename[32];
char debug_buffer[256];
zedf9p_t gnss_module;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */
uint8_t init_sd_card(void);
uint8_t create_new_log_file(void);
void configure_gnss_for_logging(void);
void process_gnss_logging(void);
void print_statistics(void);
void debug_print(const char* message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Platform interface implementations for STM32Cube
int platform_i2c_write_cube(uint8_t dev_addr, const uint8_t *data, uint16_t len) {
  const HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1, (uint8_t*)data, len, 1000);
  return (status == HAL_OK) ? 0 : -1;
}

int platform_i2c_read_cube(uint8_t dev_addr, uint8_t *data, uint16_t len) {
  HAL_Delay(2); // Small delay before read
  const HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, dev_addr << 1, data, len, 1000);
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
  (&huart2, (uint8_t*)message, strlen(message), 1000);
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
        const uint32_t total_sectors = (fs->n_fatent - 2) * fs->csize;
        const uint32_t free_sectors = free_clusters * fs->csize;

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
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
