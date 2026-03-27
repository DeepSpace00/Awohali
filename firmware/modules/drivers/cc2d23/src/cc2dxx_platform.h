#ifndef CC2DXX_PLATFORM_H
#define CC2DXX_PLATFORM_H

#include <stdint.h>

typedef enum {
    CC2DXX_OK = 0,
    CC2DXX_ERROR = -1
} CC2DXX_Status;

CC2DXX_Status cc2dxx_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
CC2DXX_Status cc2dxx_i2c_write(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len);

#endif // CC2DXX_PLATFORM_H