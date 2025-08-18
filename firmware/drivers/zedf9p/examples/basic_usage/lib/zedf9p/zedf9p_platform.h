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

#ifdef __cplusplus
//extern "C" {
#endif

/**
 * @brief Platform-specific I2C write function
 * @param dev_addr I2C address of the device
 * @param data Pointer to data buffer to write
 * @param len Number of bytes to write
 * @return true if successful, false otherwise
 */
int platform_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len);

/**
 * @brief Platform-specific I2C read function
 * @param dev_addr I2C address of the device
 * @param data Pointer to buffer to store read data
 * @param len Number of bytes to read
 * @return true if successful, false otherwise
 */
int platform_i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t len);

int platform_i2c_available(uint8_t dev_addr);

/**
 * @brief Platform-specific UART write function
 * @param data Pointer to data buffer to write
 * @param len Number of bytes to write
 * @return true if successful, false otherwise
 */
int platform_uart_write(const uint8_t *data, uint16_t len);

/**
 * @brief Platform-specific UART read function
 * @param data Pointer to buffer to store read data
 * @param len Number of bytes to read
 * @return Number of bytes actually read
 */
int platform_uart_read(uint8_t *data, uint16_t len);

/**
 * @brief Platform-specific UART data available check
 * @return Number of bytes available to read
 */
int platform_uart_available();

void platform_uart_flush();

/**
 * @brief Platform-specific millisecond-delay function
 * @param ms Delay time in milliseconds
 */
void platform_delay_ms(uint32_t ms);

/**
 * @brief Platform-specific microsecond-delay function
 * @param us Delay time in microseconds
 */
void platform_delay_us(uint32_t us);

/**
 * @brief Platform-specific I2C initialization
 * @return true if successful, false otherwise
 */
int platform_i2c_init();

int platform_i2c_init_custom(void *wire_instance);

/**
 * @brief Platform-specific UART initialization
 * @param serial_instance Serial instance
 * @param baudrate UART baudrate (typically 38400)
 * @return true if successful, false otherwise
 */
int platform_uart_init_hardware(void *serial_instance, uint32_t baudrate);

/**
 * @brief Platform-specific UART initialization
 * @param rx_pin Software serial rx pin
 * @param tx_pin Software serial tx pin
 * @param baudrate UART baudrate (typically 38400)
 * @return true if successful, false otherwise
 */
int platform_uart_init_software(uint8_t rx_pin, uint8_t tx_pin, uint32_t baudrate);

/**
 * @brief Platform-specific UART deinitialization
 * @return true if successful, false otherwise
 */
void platform_uart_deinit();

/**
 * @brief Platform-specific I2C deinitialization
 * @return true if successful, false otherwise
 */
void platform_i2c_deinit();

uint32_t platform_millis();

uint32_t platform_micros();

/**
 * @brief Platform-specific debug print function (optional)
 * @param format Printf-style format string
 * @param ... Variable arguments
 */
void zedf9p_platform_debug_print(const char *format, ...);

#ifdef __cplusplus
// }
#endif

#endif /* ZEDF9P_PLATFORM_H */