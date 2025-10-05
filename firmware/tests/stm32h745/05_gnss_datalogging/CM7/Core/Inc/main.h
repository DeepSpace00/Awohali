/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "usb2422.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
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
} logging_stats_t;

typedef struct {
    uint32_t i_tow;        // GPS time of week (ms)
    uint16_t year;         // Year (UTC)
    uint8_t month;         // Month (UTC)
    uint8_t day;           // Day (UTC)
    uint8_t hour;          // Hour (UTC)
    uint8_t min;           // Minute (UTC)
    uint8_t sec;           // Second (UTC)
    uint8_t valid;         // Time validity flags
    uint8_t time_available;
} gnss_time_t;

extern logging_stats_t logging_stats;
extern gnss_time_t gnss_time;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void usb_debug_print(const char *message);

usb2422_status_t configure_usb2422_for_enumeration(usb2422_t *dev);
void print_hub_configuration(usb2422_t *dev);

extern usb2422_hub_settings_t hub_settings;
extern usb2422_power_settings_t power_settings;
extern usb2422_downstream_port_settings_t port_settings;
extern usb2422_cfg_regs_t config_registers;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GNSS_ENABLE_Pin GPIO_PIN_10
#define GNSS_ENABLE_GPIO_Port GPIOE
#define GNSS_TXRDY_Pin GPIO_PIN_11
#define GNSS_TXRDY_GPIO_Port GPIOE
#define GNSS_INT_Pin GPIO_PIN_12
#define GNSS_INT_GPIO_Port GPIOE
#define GNSS_PPS_Pin GPIO_PIN_13
#define GNSS_PPS_GPIO_Port GPIOE
#define GNSS_RESET_Pin GPIO_PIN_14
#define GNSS_RESET_GPIO_Port GPIOE
#define L4_RESET_Pin GPIO_PIN_3
#define L4_RESET_GPIO_Port GPIOD
#define L4_BOOT_Pin GPIO_PIN_4
#define L4_BOOT_GPIO_Port GPIOD
#define H7_LEDBUILTIN_Pin GPIO_PIN_8
#define H7_LEDBUILTIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ZEDF9P_I2C_TIMEOUT_MS     1000
#define ZEDF9P_UART_TIMEOUT_MS    1000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
