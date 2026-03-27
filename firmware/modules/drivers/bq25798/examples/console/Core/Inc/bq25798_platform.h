#ifndef BQ25798_PLATFORM_H
#define BQ25798_PLATFORM_H

#include <stdint.h>

typedef enum {
    BQ25798_OK = 0,
    BQ25798_ERROR = -1
} BQ25798_Status;

BQ25798_Status bq25798_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
BQ25798_Status bq25798_i2c_write(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len);

#endif // BQ25798_PLATFORM_H