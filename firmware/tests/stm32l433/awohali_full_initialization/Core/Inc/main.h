/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NEOPIXEL_DIN_Pin GPIO_PIN_0
#define NEOPIXEL_DIN_GPIO_Port GPIOH
#define LED_BUILTIN_Pin GPIO_PIN_1
#define LED_BUILTIN_GPIO_Port GPIOH
#define CAN_STB_Pin GPIO_PIN_0
#define CAN_STB_GPIO_Port GPIOA
#define BAT_CHG_INT_Pin GPIO_PIN_2
#define BAT_CHG_INT_GPIO_Port GPIOA
#define EEPROM_WC_Pin GPIO_PIN_3
#define EEPROM_WC_GPIO_Port GPIOA
#define BOOT_H7_Pin GPIO_PIN_5
#define BOOT_H7_GPIO_Port GPIOA
#define PG_1V8_Pin GPIO_PIN_1
#define PG_1V8_GPIO_Port GPIOB
#define EN_1V8_Pin GPIO_PIN_2
#define EN_1V8_GPIO_Port GPIOB
#define PG_3V3_Pin GPIO_PIN_10
#define PG_3V3_GPIO_Port GPIOB
#define BAL_ALERT_Pin GPIO_PIN_11
#define BAL_ALERT_GPIO_Port GPIOB
#define SPI1_GRANT_Pin GPIO_PIN_12
#define SPI1_GRANT_GPIO_Port GPIOB
#define SPI1_REQ_Pin GPIO_PIN_13
#define SPI1_REQ_GPIO_Port GPIOB
#define QON_Pin GPIO_PIN_14
#define QON_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_15
#define CE_GPIO_Port GPIOB
#define SS_L4_Pin GPIO_PIN_8
#define SS_L4_GPIO_Port GPIOA
#define PG_2V8_Pin GPIO_PIN_15
#define PG_2V8_GPIO_Port GPIOA
#define EN_2V8_Pin GPIO_PIN_3
#define EN_2V8_GPIO_Port GPIOB
#define RESET_H7_Pin GPIO_PIN_4
#define RESET_H7_GPIO_Port GPIOB
#define HUB_INT_Pin GPIO_PIN_6
#define HUB_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
