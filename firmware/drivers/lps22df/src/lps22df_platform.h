#ifndef LPS22DF_PLATFORM_H
#define LPS22DF_PLATFORM_H

#include <stdint.h>

typedef enum {
    LPS22DF_OK = 0,
    LPS22DF_ERROR = -1
} LPS22DF_Status;

LPS22DF_Status lps22df_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
LPS22DF_Status lps22df_i2c_write(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len);
void lps22df_delay_ms(uint32_t ms);

#endif // LPS22DF_PLATFORM_H