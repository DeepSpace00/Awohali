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
#include "stm32h7xx_hal.h"

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
#define RESET_LTE_Pin GPIO_PIN_3
#define RESET_LTE_GPIO_Port GPIOE
#define LTE_SLEEP_Pin GPIO_PIN_6
#define LTE_SLEEP_GPIO_Port GPIOE
#define LTE_PWR_EN_Pin GPIO_PIN_13
#define LTE_PWR_EN_GPIO_Port GPIOC
#define SPI_GRANT_Pin GPIO_PIN_2
#define SPI_GRANT_GPIO_Port GPIOC
#define SPI_REQ_Pin GPIO_PIN_3
#define SPI_REQ_GPIO_Port GPIOC
#define RESET_ETH_Pin GPIO_PIN_3
#define RESET_ETH_GPIO_Port GPIOA
#define HUB_SUSPEND_Pin GPIO_PIN_0
#define HUB_SUSPEND_GPIO_Port GPIOB
#define RESET_HUB_Pin GPIO_PIN_1
#define RESET_HUB_GPIO_Port GPIOB
#define GNSS_PWR_EN_Pin GPIO_PIN_10
#define GNSS_PWR_EN_GPIO_Port GPIOE
#define RESET_GNSS_Pin GPIO_PIN_14
#define RESET_GNSS_GPIO_Port GPIOE
#define H7_CAN_STB_Pin GPIO_PIN_2
#define H7_CAN_STB_GPIO_Port GPIOD
#define RESET_L4_H7_Pin GPIO_PIN_3
#define RESET_L4_H7_GPIO_Port GPIOD
#define BOOT0_L4_Pin GPIO_PIN_4
#define BOOT0_L4_GPIO_Port GPIOD
#define SPI_SS_L4_Pin GPIO_PIN_5
#define SPI_SS_L4_GPIO_Port GPIOD
#define NEOPIXEL_Pin GPIO_PIN_9
#define NEOPIXEL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
