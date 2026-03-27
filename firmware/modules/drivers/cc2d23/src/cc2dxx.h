#ifndef CC2DXX_H
#define CC2DXX_H

#include <stdint.h>
#include "cc2dxx_platform.h"

#define CC2DXX_I2C_ADDR 0x28

typedef struct {
    float temperature_c;
    float humidity_rh;
} CC2DXX_Result;

CC2DXX_Status cc2dxx_read(CC2DXX_Result *result);

#endif // CC2DXX_H