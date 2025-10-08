/**
 * @file zedf9p_platform_stm32cube.c
 * @brief Platform abstraction layer implementation for ZEDF9P GNSS driver (STM32Cube)
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-09-25
 *
 * This file should be placed in the src folder and contains platform-specific
 * implementations for STM32Cube. Only one platform implementation should be
 * compiled based on your target platform.
 */

#include "main.h"
#include "zedf9p_platform.h"

// Platform configuration -- define these in main.c
static I2C_HandleTypeDef *zedf9p_i2c_handle = NULL;
static UART_HandleTypeDef *zedf9p_uart_handle = NULL;

/**
 * @brief Set the I2C handle for ZEDF9P communication
 * @param hi2c Pointer to I2C handle (e.g., &hi2c1)
 */
void zedf9p_platform_set_i2c_handle(I2C_HandleTypeDef *hi2c) {
    zedf9p_i2c_handle = hi2c;
}

/**
 * @brief Set the UART handle for ZEDF9P communication
 * @param huart Pointer to UART handle (e.g., &huart1)
 */
void zedf9p_platform_set_uart_handle(UART_HandleTypeDef *huart) {
    zedf9p_uart_handle = huart;
}

/**
 * @brief Platform I2C write function
 * @param dev_addr Pointer to initialized driver struct
 * @param data Pointer to data to write
 * @param len Number of bytes to write
 * @return 0 on success, -1 on error
 */
int platform_i2c_write(const uint8_t dev_addr, const uint8_t *data, const uint16_t len) {
    if (zedf9p_i2c_handle == NULL) return -1;

    const HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(zedf9p_i2c_handle, dev_addr << 1,
        (uint8_t*)data, len, 1000);
    return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief Platform I2C read function
 * @param dev_addr Pointer to initialized driver struct
 * @param data Pointer to buffer to received data
 * @param len Number of bytes to read
 * @return Number of bytes read on success, -1 on error
 */
int platform_i2c_read(const uint8_t dev_addr, uint8_t *data, const uint16_t len) {
    if (zedf9p_i2c_handle == NULL) return -1;

    HAL_Delay(1); // Small delay before read
    const HAL_StatusTypeDef status = HAL_I2C_Master_Receive(zedf9p_i2c_handle, dev_addr << 1,
        data, len, 1000);
    if (status == HAL_OK) {
        return len;
    }
    return -1;
}

/**
 * @brief Platform UART write function
 * @param data Pointer to data to write
 * @param len Number of bytes to write
 * @return 0 on success, -1 on error
 */
int platform_uart_write(const uint8_t *data, const uint16_t len) {
    if (zedf9p_uart_handle == NULL) return -1;

    const HAL_StatusTypeDef status = HAL_UART_Transmit(zedf9p_uart_handle, (uint8_t*)data, len, ZEDF9P_UART_TIMEOUT_MS);
    return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief Platform UART read function
 * @param data Pointer to buffer for received data
 * @param len Number of bytes to read
 * @return Number of bytes read on success, -1 on error, -2 if no data is available
 */
int platform_uart_read(uint8_t *data, const uint16_t len) {
    if (zedf9p_uart_handle == NULL) return -1;

    const HAL_StatusTypeDef status = HAL_UART_Receive(zedf9p_uart_handle, data, len, ZEDF9P_UART_TIMEOUT_MS);

    if (status == HAL_OK) {
        return (int)len;
    } else if (status == HAL_TIMEOUT) {
        return -2; // No data available (timeout)
    } else {
        return -1; // Other error
    }
}

/**
 * @brief Platform delay function
 * @param ms Delay in milliseconds
 */
void platform_delay_ms(const uint32_t ms) {
    HAL_Delay(ms);
}

/**
 * @brief Platform millisecond timestamp function
 * @return Current timestamp in milliseconds
 */
uint32_t platform_get_millis(void) {
    return HAL_GetTick();
}

/**
 * @brief Platform debug message function
 * @param message Debug message to print
 */
void platform_debug_print(const char *message) {
    // Only print debug messages if USB is ready and logging is active
    if (logging_stats.usb_ready) {
        usb_debug_print(message);
    }
}