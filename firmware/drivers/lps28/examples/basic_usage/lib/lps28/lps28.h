/**
 * @file lps28.h
 * @brief LPS28DFW Pressure and Temperature Sensor Driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-14
 * 
 * This driver supports the LPS28DFW sensor from STMicroelectronics
 */

#ifndef LPS28_H
#define LPS28_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define LPS28_I2C_ADDR_1 0x5C
#define LPS28_I2C_ADDR_2 0x5D

/**
 * @brief LPS28DFW sensor driver return status codes.
 */
typedef enum {
    LPS28_OK = 0,
    LPS28_ERR_I2C = -1,
    LPS28_ERR_TIMEOUT = -2,
    LPS28_ERR_NULL = -3,
    LPS28_ERR_INVALID_ARG = -4,
    LPS28_ERR_WHO_AM_I = -5
} lps28_status_t;

/**
 * @brief Human-readable description of an LPS28DFW status code.
 *
 * @param status The status code to translate.
 * @return A string describing the error.
 */
const char* lps28_stat_error(lps28_status_t status);

/**
 * @brief Output data rate configuration for LPS28DFW sensor.
 */
typedef enum {
    LPS28_ODR_ONE_SHOT = 0x00,
    LPS28_ODR_1_HZ = 0x01,
    LPS28_ODR_4_HZ = 0x02,
    LPS28_ODR_10_HZ = 0x03,
    LPS28_ODR_25_HZ = 0x04,
    LPS28_ODR_50_HZ = 0x05,
    LPS28_ODR_75_HZ = 0x06,
    LPS28_ODR_100_HZ = 0x07,
    LPS28_ODR_200_HZ = 0x08
} lps28_odr_t;

/**
 * @brief Average configuration for LPS28DFW sensor.
 */
typedef enum {
    LPS28_AVG_4 = 0x00,
    LPS28_AVG_8 = 0x01,
    LPS28_AVG_16 = 0x02,
    LPS28_AVG_32 = 0x03,
    LPS28_AVG_64 = 0x04,
    LPS28_AVG_128 = 0x05,
    LPS28_AVG_512 = 0x06
} lps28_avg_t;

/**
 * @brief Platform interface abstraction for LPS28DFW driver.
 *
 * The user must provide these functions to enable hardware-specific I2C and timing.
 */
typedef struct {
    int (*i2c_write)(uint8_t dev_addr, const uint8_t *data, uint16_t len);
    int (*i2c_read)(uint8_t dev_addr, uint8_t *data, uint16_t len);
    void (*delay_ms)(uint32_t ms);
} lps28_interface_t;

/**
 * @brief LPS28DFW driver instance.
 */
typedef struct {
    uint8_t i2c_address;
    lps28_interface_t io;
    bool initialized;
} lps28_t;

/**
 * @brief Pressure and Temperature measurements from the LPS28DFW sensor
 */
typedef struct {
    float pressure_hpa;
    float temperature_c;
} lps28_measurement_t;

/**
 * @brief Status register data from the LPS28DFW sensor
 */
typedef struct {
    bool temperature_overrun;
    bool pressure_overrun;
    bool temperature_data_available;
    bool pressure_data_available;
} lps28_status_reg_t;

// Public function prototypes
lps28_status_t lps28_init(lps28_t *dev, uint8_t address, lps28_interface_t io);

lps28_status_t lps28_who_am_i(lps28_t *dev, uint8_t *who_am_i);

lps28_status_t lps28_ctrl_reg1(lps28_t *dev, lps28_odr_t odr, lps28_avg_t avg);

lps28_status_t lps28_ctrl_reg2(lps28_t *dev, bool boot, bool reset, bool oneshot);

lps28_status_t lps28_reference_pressure(lps28_t *dev, float ref_pressure);

lps28_status_t lps28_pressure_offset(lps28_t *dev, float offset);

lps28_status_t lps28_status(lps28_t *dev, lps28_status_reg_t *status);

lps28_status_t lps28_read_pressure(lps28_t *dev, float *pressure);

lps28_status_t lps28_read_temperature(lps28_t *dev, float *temperature);

#ifdef __cplusplus
}
#endif

#endif // LPS28_H