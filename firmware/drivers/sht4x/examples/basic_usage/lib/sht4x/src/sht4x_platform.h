#ifndef SHT4X_PLATFORM_H
#define SHT4X_PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SHT4X_OK = 0,
    SHT4X_ERROR = -1
} SHT4X_Status;

SHT4X_Status sht4x_i2c_read(uint8_t dev_addr, uint8_t *data, uint8_t len);
SHT4X_Status sht4x_i2c_write(uint8_t dev_addr, const uint8_t *data, uint8_t len);
void sht4x_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif // SHT4X_PLATFORM_H