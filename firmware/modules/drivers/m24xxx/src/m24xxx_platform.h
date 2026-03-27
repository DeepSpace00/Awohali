#ifndef M24XXX_PLATFORM_H
#define M24XXX_PLATFORM_H

#include <stdint.h>

typedef enum {
    M24xxx_OK = 0,
    M24xxx_ERROR = -1
} M24xxx_Status;

M24xxx_Status m24xxx_i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t len);
M24xxx_Status m24xxx_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len);

#endif // M24XXX_PLATFORM_H