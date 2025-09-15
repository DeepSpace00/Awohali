/*!
* @file stm32f4xx_it.h
 * @brief This file contains the headers of the interrupt handlers.
 * @author STM32CubeMX Generated / Modified for ZEDF9P Logger
 * @date 2025-09-14
 */

#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "main.h"

    /* Exported types ------------------------------------------------------------*/
    /* Exported constants --------------------------------------------------------*/
    /* Exported macro ------------------------------------------------------------*/
    /* Exported functions prototypes ---------------------------------------------*/

    // Cortex-M4 Processor Interruption and Exception Handlers
    void NMI_Handler(void);
    void HardFault_Handler(void);
    void MemManage_Handler(void);
    void BusFault_Handler(void);
    void UsageFault_Handler(void);
    void SVC_Handler(void);
    void DebugMon_Handler(void);
    void PendSV_Handler(void);
    void SysTick_Handler(void);

    // STM32F4xx Peripheral Interrupt Handlers
    void I2C1_EV_IRQHandler(void);
    void I2C1_ER_IRQHandler(void);
    void USART2_IRQHandler(void);
    void SDIO_IRQHandler(void);
    void DMA2_Stream3_IRQHandler(void);  // For SDIO DMA if used
    void DMA2_Stream6_IRQHandler(void);  // For SDIO DMA if used

    // Optional: Additional interrupt handlers you might need
    void EXTI0_IRQHandler(void);         // External interrupt (if using GNSS interrupt pin)
    void TIM2_IRQHandler(void);          // Timer interrupt (if using periodic timer)
    void DMA1_Stream0_IRQHandler(void);  // I2C DMA if enabled
    void DMA1_Stream5_IRQHandler(void);  // I2C DMA if enabled

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */