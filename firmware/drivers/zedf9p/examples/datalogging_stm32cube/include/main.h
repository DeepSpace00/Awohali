#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal_conf.h"

// Handle declarations
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern SD_HandleTypeDef hsd;

// Function prototypes
void Error_Handler(void);
void SystemClock_Config(void);

#endif