/*!
 * @file zedf9p_platform_stm32cube.c
 * @brief Platform abstraction layer implementation for ZEDF9P GNSS driver (STM32Cube HAL)
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-09-14
 *
 * This file contains platform-specific implementations for STM32Cube HAL.
 * Designed for SparkFun STM32 Thing Plus (STM32F405RGT6).
 */

#include "stm32f4xx_hal.h"  // Must be first
#include "main.h"
#include "zedf9p_platform.h"
#include "i2c.h"
#include "usart.h"
#include <string.h>

// External HAL handles (defined in main.c or generated files)
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

// Platform configuration
#define I2C_TIMEOUT_MS 1000
#define UART_TIMEOUT_MS 1000
#define I2C_RETRY_COUNT 3

// I2C interface implementation
int platform_i2c_write(const uint8_t dev_addr, const uint8_t *data, const uint16_t len) {
    if (data == NULL || len == 0) {
        return -1;
    }

    uint8_t retry_count = 0;
    HAL_StatusTypeDef status;

    while (retry_count < I2C_RETRY_COUNT) {
        // STM32 HAL expects device address to be left-shifted by 1
        status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr << 1, (uint8_t*)data, len, I2C_TIMEOUT_MS);

        if (status == HAL_OK) {
            return 0; // Success
        }

        // Handle specific error cases
        if (status == HAL_BUSY) {
            // I2C bus is busy, wait and retry
            HAL_Delay(10);
        } else if (status == HAL_ERROR) {
            // Reset I2C peripheral if error occurs
            __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF);
            HAL_I2C_DeInit(&hi2c1);
            HAL_Delay(1);
            HAL_I2C_Init(&hi2c1);
        }

        retry_count++;
        HAL_Delay(1);
    }

    return -1; // Failed after retries
}

int platform_i2c_read(const uint8_t dev_addr, uint8_t *data, const uint16_t len) {
    if (data == NULL || len == 0) {
        return -1;
    }

    // Small delay before reading to ensure data is ready
    HAL_Delay(2);

    uint8_t retry_count = 0;
    HAL_StatusTypeDef status;

    while (retry_count < I2C_RETRY_COUNT) {
        // STM32 HAL expects device address to be left-shifted by 1
        status = HAL_I2C_Master_Receive(&hi2c1, dev_addr << 1, data, len, I2C_TIMEOUT_MS);

        if (status == HAL_OK) {
            return len; // Return number of bytes read
        }

        // Handle specific error cases
        if (status == HAL_BUSY) {
            // I2C bus is busy, wait and retry
            HAL_Delay(10);
        } else if (status == HAL_ERROR) {
            // Reset I2C peripheral if error occurs
            __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF);
            HAL_I2C_DeInit(&hi2c1);
            HAL_Delay(1);
            HAL_I2C_Init(&hi2c1);
        } else if (status == HAL_TIMEOUT) {
            // Timeout occurred, device may not be responding
            break;
        }

        retry_count++;
        HAL_Delay(1);
    }

    return 0; // Return 0 bytes read on failure
}

// UART interface implementation (if needed for GNSS communication)
int platform_uart_write(const uint8_t *data, const uint16_t len) {
    if (data == NULL || len == 0) {
        return -1;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)data, len, UART_TIMEOUT_MS);
    return (status == HAL_OK) ? 0 : -1;
}

int platform_uart_read(uint8_t *data, const uint16_t len) {
    if (data == NULL || len == 0) {
        return -1;
    }

    // Check if data is available
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == RESET) {
        return 0; // No data available
    }

    uint16_t bytes_read = 0;
    uint32_t timeout = HAL_GetTick() + UART_TIMEOUT_MS;

    while (bytes_read < len && HAL_GetTick() < timeout) {
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
            data[bytes_read] = (uint8_t)(huart2.Instance->DR & 0xFF);
            bytes_read++;
        }
    }

    return bytes_read;
}

// Alternative UART read using HAL_UART_Receive with timeout
int platform_uart_read_blocking(uint8_t *data, const uint16_t len, uint32_t timeout_ms) {
    if (data == NULL || len == 0) {
        return -1;
    }

    HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, data, len, timeout_ms);
    if (status == HAL_OK) {
        return len;
    } else if (status == HAL_TIMEOUT) {
        return 0; // Timeout, no data received
    } else {
        return -1; // Error occurred
    }
}

// Timing functions
void platform_delay_ms(const uint32_t ms) {
    HAL_Delay(ms);
}

uint32_t platform_get_millis(void) {
    return HAL_GetTick();
}

// Debug print function (outputs to UART2)
void platform_debug_print(const char *message) {
    if (message == NULL) {
        return;
    }

    size_t len = strlen(message);
    HAL_UART_Transmit(&huart2, (uint8_t*)message, len, UART_TIMEOUT_MS);
}

// Enhanced debug print with formatting
void platform_debug_printf(const char *format, ...) {
    static char debug_buffer[256];
    va_list args;

    va_start(args, format);
    int len = vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);

    if (len > 0 && len < sizeof(debug_buffer)) {
        HAL_UART_Transmit(&huart2, (uint8_t*)debug_buffer, len, UART_TIMEOUT_MS);
    }
}

// I2C bus scan function for debugging
void platform_i2c_scan(void) {
    platform_debug_print("Scanning I2C bus...\r\n");

    uint8_t device_count = 0;
    for (uint8_t address = 1; address < 127; address++) {
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 1, 100);
        if (status == HAL_OK) {
            char buffer[64];
            sprintf(buffer, "I2C device found at address 0x%02X\r\n", address);
            platform_debug_print(buffer);
            device_count++;
        }
    }

    if (device_count == 0) {
        platform_debug_print("No I2C devices found\r\n");
    } else {
        char buffer[64];
        sprintf(buffer, "Found %d I2C device(s)\r\n", device_count);
        platform_debug_print(buffer);
    }
}

// I2C error recovery function
void platform_i2c_recover(void) {
    platform_debug_print("Attempting I2C recovery...\r\n");

    // Disable I2C peripheral
    __HAL_I2C_DISABLE(&hi2c1);

    // Reset I2C peripheral
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(1);
    __HAL_RCC_I2C1_RELEASE_RESET();

    // Reinitialize I2C
    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(1);
    if (HAL_I2C_Init(&hi2c1) == HAL_OK) {
        platform_debug_print("I2C recovery successful\r\n");
    } else {
        platform_debug_print("I2C recovery failed\r\n");
    }
}

// GPIO utility functions for status indication
void platform_set_status_led(uint8_t state) {
    GPIO_PinState pin_state = state ? GPIO_PIN_RESET : GPIO_PIN_SET; // LED is active low
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, pin_state);
}

void platform_toggle_status_led(void) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

// Power management functions
void platform_gnss_power_on(void) {
    // Enable GNSS power if using power control pin
    // HAL_GPIO_WritePin(GNSS_PWR_GPIO_Port, GNSS_PWR_Pin, GPIO_PIN_SET);
    // HAL_Delay(100); // Allow power to stabilize
}

void platform_gnss_power_off(void) {
    // Disable GNSS power if using power control pin
    // HAL_GPIO_WritePin(GNSS_PWR_GPIO_Port, GNSS_PWR_Pin, GPIO_PIN_RESET);
}

// System information functions
void platform_print_system_info(void) {
    char buffer[128];

    sprintf(buffer, "STM32F405 GNSS Data Logger\r\n");
    platform_debug_print(buffer);

    sprintf(buffer, "System Clock: %lu MHz\r\n", HAL_RCC_GetSysClockFreq() / 1000000);
    platform_debug_print(buffer);

    sprintf(buffer, "HAL Version: %lu\r\n", HAL_GetHalVersion());
    platform_debug_print(buffer);

    sprintf(buffer, "Tick Frequency: %lu Hz\r\n", HAL_GetTickFreq());
    platform_debug_print(buffer);
}

// Memory and resource monitoring
uint32_t platform_get_free_heap(void) {
    // For STM32Cube, you'd need to implement heap monitoring
    // This is a placeholder - actual implementation depends on your memory management
    return 0;
}

uint32_t platform_get_stack_usage(void) {
    // Stack usage monitoring - implementation depends on your requirements
    return 0;
}

// Error handling
void platform_error_handler(const char* error_message) {
    platform_debug_print("PLATFORM ERROR: ");
    platform_debug_print(error_message);
    platform_debug_print("\r\n");

    // Flash LED rapidly to indicate error
    for (int i = 0; i < 20; i++) {
        platform_toggle_status_led();
        HAL_Delay(100);
    }
}

// Watchdog functions (if using watchdog)
void platform_watchdog_init(void) {
    // Initialize independent watchdog if needed
    // IWDG configuration would go here
}

void platform_watchdog_refresh(void) {
    // Refresh watchdog timer
    // HAL_IWDG_Refresh(&hiwdg);
}

// Critical section management
void platform_enter_critical(void) {
    __disable_irq();
}

void platform_exit_critical(void) {
    __enable_irq();
}

// DMA and interrupt utilities (if needed for high-performance I2C)
void platform_i2c_enable_dma(void) {
    // Enable DMA for I2C if configured
    // This would require additional DMA setup in STM32CubeMX
}

void platform_i2c_disable_dma(void) {
    // Disable DMA for I2C
}

// Temperature monitoring (STM32F405 has internal temperature sensor)
float platform_get_cpu_temperature(void) {
    // Read internal temperature sensor
    // This requires ADC configuration for the temperature sensor channel
    return 25.0f; // Placeholder
}