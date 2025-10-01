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
#include "usb2422_platform.h"

// Platform configuration -- define in main.c
static I2C_HandleTypeDef *usb2422_i2c_handle = NULL;

/**
 * @brief Set the I2C handle for ZEDF9P communication
 * @param hi2c Pointer to I2C handle (e.g., &hi2c1)
 */
void usb2422_platform_set_i2c_handle(I2C_HandleTypeDef *hi2c) {
    usb2422_i2c_handle = hi2c;
}

/**
 * @brief Platform-specific I2C write function for STM32Cube HAL
 * @details Implements SMBus block write protocol required by USB2422.
 *          Format: [register][length][data...]
 * @param dev_addr I2C device address (7-bit)
 * @param data Pointer to data buffer (first byte is register address)
 * @param len Total length including register address
 * @return 0 on success, -1 on failure
 */
int platform_i2c_write_reg(const uint8_t dev_addr, const uint8_t *data, const uint16_t len) {
    if (usb2422_i2c_handle == NULL) return -1;

    uint8_t smbus_buffer[64]; // Buffer for SMBus block write

    if (len < 2 || len > 63) {
        return -1; // Invalid length
    }

    // Build SMBus block write packet
    // Format: [register][length][data...]
    smbus_buffer[0] = data[0];           // Register address
    smbus_buffer[1] = len - 1;           // Data length (excluding register)

    // Copy data bytes
    for (uint16_t i = 1; i < len; i++) {
        smbus_buffer[i + 1] = data[i];
    }

    // Transmit via I2C (register + length + data)
    const HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(usb2422_i2c_handle, dev_addr << 1, smbus_buffer, len + 1, 1000);

    return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief Platform-specific I2C read function for STM32Cube HAL
 * @details Implements SMBus block read protocol required by USB2422.
 *          Device returns: [length][data...]
 * @param dev_addr I2C device address (7-bit)
 * @param reg Register address to read from
 * @param data Pointer to buffer for received data
 * @param len Number of data bytes to read
 * @return 0 on success, -1 on failure
 */
int platform_i2c_read_reg(const uint8_t dev_addr, uint8_t reg, uint8_t *data, const uint16_t len) {
    if (usb2422_i2c_handle == NULL) return -1;

    HAL_Delay(1); // Small delay before read

    uint8_t smbus_buffer[64]; // Buffer for SMBus block read

    if (len == 0 || len > 62) {
        return -1; // Invalid length
    }

    // Write register address
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(usb2422_i2c_handle, dev_addr << 1, &reg,  1,  1000);
    if (status != HAL_OK) {
        return -1;
    }

    // Small delay for device to prepare data
    HAL_Delay(2);

    // Read length byte + data bytes
    status = HAL_I2C_Master_Receive(usb2422_i2c_handle,
                                    dev_addr << 1,
                                    smbus_buffer,
                                    len + 1,  // +1 for length byte
                                    1000);
    if (status != HAL_OK) {
        return -1;
    }

    // First byte is length (discard it)
    const uint8_t returned_length = smbus_buffer[0];

    // Verify returned length matches expected
    if (returned_length != len) {
        return -1;
    }

    // Copy data bytes (skip length byte)
    for (uint16_t i = 0; i < len; i++) {
        data[i] = smbus_buffer[i + 1];
    }

    return 0;
}