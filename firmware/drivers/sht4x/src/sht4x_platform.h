#ifndef SHT4X_PLATFORM_H
#define SHT4X_PLATFORM_H

#include <stdint.h>

typedef enum {
    SHT4X_OK = 0,
    SHT4X_ERROR = -1
} SHT4X_Status;

SHT4X_Status sht4x_i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t len);
SHT4X_Status sht4x_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len);
void sht4x_delay_ms(uint32_t ms);

#endif // SHT4X_PLATFORM_H