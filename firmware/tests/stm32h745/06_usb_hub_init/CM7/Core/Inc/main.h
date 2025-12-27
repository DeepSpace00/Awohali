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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb2422.h"

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

extern logging_stats_t logging_stats;
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
#define HUB_SUSPEND_Pin GPIO_PIN_0
#define HUB_SUSPEND_GPIO_Port GPIOB
#define RESET_HUB_Pin GPIO_PIN_1
#define RESET_HUB_GPIO_Port GPIOB
#define HUB_H7_INT_Pin GPIO_PIN_2
#define HUB_H7_INT_GPIO_Port GPIOB
#define L4_RESET_Pin GPIO_PIN_3
#define L4_RESET_GPIO_Port GPIOD
#define L4_BOOT_Pin GPIO_PIN_4
#define L4_BOOT_GPIO_Port GPIOD
#define H7_LEDBUILTIN_Pin GPIO_PIN_8
#define H7_LEDBUILTIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
