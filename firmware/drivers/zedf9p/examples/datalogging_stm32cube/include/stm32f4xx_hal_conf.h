#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

// Module Selection
#define HAL_MODULE_ENABLED

// Peripheral modules
#define HAL_ADC_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED
#define HAL_CRC_MODULE_ENABLED
#define HAL_CRYP_MODULE_ENABLED
#define HAL_DAC_MODULE_ENABLED
#define HAL_DCMI_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_DMA2D_MODULE_ENABLED
#define HAL_ETH_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_NAND_MODULE_ENABLED
#define HAL_NOR_MODULE_ENABLED
#define HAL_PCCARD_MODULE_ENABLED
#define HAL_SRAM_MODULE_ENABLED
#define HAL_SDRAM_MODULE_ENABLED
#define HAL_HASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_I2S_MODULE_ENABLED
#define HAL_IWDG_MODULE_ENABLED
#define HAL_LTDC_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RNG_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_SAI_MODULE_ENABLED
#define HAL_SD_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_USART_MODULE_ENABLED
#define HAL_IRDA_MODULE_ENABLED
#define HAL_SMARTCARD_MODULE_ENABLED
#define HAL_WWDG_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_HCD_MODULE_ENABLED
#define HAL_FMPI2C_MODULE_ENABLED
#define HAL_SPDIFRX_MODULE_ENABLED
#define HAL_DFSDM_MODULE_ENABLED
#define HAL_LPTIM_MODULE_ENABLED
#define HAL_MMC_MODULE_ENABLED

// Oscillator Values
#define HSE_VALUE    8000000U
#define HSE_STARTUP_TIMEOUT    100U
#define HSI_VALUE    16000000U
#define LSI_VALUE    32000U
#define LSE_VALUE    32768U
#define LSE_STARTUP_TIMEOUT    5000U
#define EXTERNAL_CLOCK_VALUE    12288000U

// System Configuration
#define VDD_VALUE    3300U
#define TICK_INT_PRIORITY    0U
#define USE_RTOS    0U
#define PREFETCH_ENABLE    1U
#define INSTRUCTION_CACHE_ENABLE    1U
#define DATA_CACHE_ENABLE    1U

// HAL Includes (this is the critical part)
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_crc.h"
#include "stm32f4xx_hal_cryp.h"
#include "stm32f4xx_hal_dac.h"
#include "stm32f4xx_hal_dcmi.h"
#include "stm32f4xx_hal_dma2d.h"
#include "stm32f4xx_hal_eth.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_sram.h"
#include "stm32f4xx_hal_nor.h"
#include "stm32f4xx_hal_nand.h"
#include "stm32f4xx_hal_pccard.h"
#include "stm32f4xx_hal_sdram.h"
#include "stm32f4xx_hal_hash.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_i2s.h"
#include "stm32f4xx_hal_iwdg.h"
#include "stm32f4xx_hal_ltdc.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_rng.h"
#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx_hal_sai.h"
#include "stm32f4xx_hal_sd.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_irda.h"
#include "stm32f4xx_hal_smartcard.h"
#include "stm32f4xx_hal_wwdg.h"
#include "stm32f4xx_hal_pcd.h"
#include "stm32f4xx_hal_hcd.h"
#include "stm32f4xx_hal_fmpi2c.h"
#include "stm32f4xx_hal_spdifrx.h"
#include "stm32f4xx_hal_dfsdm.h"
#include "stm32f4xx_hal_lptim.h"
#include "stm32f4xx_hal_mmc.h"

// Error checking
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f4xx_hal_rcc_ex.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */