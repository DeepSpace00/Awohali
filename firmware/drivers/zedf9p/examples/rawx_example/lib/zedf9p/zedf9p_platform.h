/*!
* @file zedf9p_platform.h
 * @brief Platform abstraction layer for ZEDF9P GNSS driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-19
 */

#ifndef ZEDF9P_PLATFORM_H
#define ZEDF9P_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

    // Platform-specific I2C functions
    int platform_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len);
    int platform_i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t len);

    // Platform-specific UART functions
    int platform_uart_write(const uint8_t *data, uint16_t len);
    int platform_uart_read(uint8_t *data, uint16_t len);

    // Platform-specific timing functions
    void platform_delay_ms(uint32_t ms);
    uint32_t platform_get_millis(void);

    void platform_set_i2c_interface(TwoWire *wire_interface);
    void platform_set_uart_interface(HardwareSerial *serial_interface);

#ifdef __cplusplus
}
#endif

#endif // ZEDF9P_PLATFORM_H