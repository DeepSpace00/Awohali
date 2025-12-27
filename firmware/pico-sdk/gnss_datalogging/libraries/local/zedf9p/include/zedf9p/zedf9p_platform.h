/**
 * @file zedf9p_platform.h
 * @brief Platform abstraction layer for ZED-F9P GNSS module driver
 * @author Generated Driver
 * @version 1.0
 * 
 * Platform-specific functions that need to be implemented by the user
 * for their specific hardware platform (Arduino, STM32, Zephyr, etc.)
 */

#ifndef ZEDF9P_PLATFORM_H
#define ZEDF9P_PLATFORM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Platform-specific delay function
 * @param ms Delay time in milliseconds
 */
void zedf9p_platform_delay(uint32_t ms);

/**
 * @brief Get current timestamp in milliseconds
 * @return Current timestamp in milliseconds
 */
uint32_t zedf9p_platform_get_tick(void);

/**
 * @brief Platform-specific I2C initialization
 * @param i2c_addr I2C address of the device (typically 0x42)
 * @return true if successful, false otherwise
 */
bool zedf9p_platform_i2c_init(uint8_t i2c_addr);

/**
 * @brief Platform-specific I2C deinitialization
 * @return true if successful, false otherwise
 */
bool zedf9p_platform_i2c_deinit(void);

/**
 * @brief Platform-specific I2C write function
 * @param i2c_addr I2C address of the device
 * @param data Pointer to data buffer to write
 * @param len Number of bytes to write
 * @return true if successful, false otherwise
 */
int zedf9p_platform_i2c_write(uint8_t i2c_addr, const uint8_t *data, size_t len);

/**
 * @brief Platform-specific I2C read function
 * @param i2c_addr I2C address of the device
 * @param data Pointer to buffer to store read data
 * @param len Number of bytes to read
 * @return true if successful, false otherwise
 */
int zedf9p_platform_i2c_read(uint8_t i2c_addr, uint8_t *data, size_t len);

/**
 * @brief Platform-specific UART initialization
 * @param baudrate UART baudrate (typically 38400)
 * @return true if successful, false otherwise
 */
bool zedf9p_platform_uart_init(uint32_t baudrate);

/**
 * @brief Platform-specific UART deinitialization
 * @return true if successful, false otherwise
 */
bool zedf9p_platform_uart_deinit(void);

/**
 * @brief Platform-specific UART write function
 * @param data Pointer to data buffer to write
 * @param len Number of bytes to write
 * @return true if successful, false otherwise
 */
bool zedf9p_platform_uart_write(const uint8_t *data, size_t len);

/**
 * @brief Platform-specific UART read function
 * @param data Pointer to buffer to store read data
 * @param len Number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes actually read
 */
size_t zedf9p_platform_uart_read(uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Platform-specific UART data available check
 * @return Number of bytes available to read
 */
size_t zedf9p_platform_uart_available(void);

/**
 * @brief Platform-specific debug print function (optional)
 * @param format Printf-style format string
 * @param ... Variable arguments
 */
void zedf9p_platform_debug_print(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* ZEDF9P_PLATFORM_H */