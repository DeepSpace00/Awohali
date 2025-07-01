#include "cc2dxx.h"

CC2DXX_Status cc2dxx_read(CC2DXX_Result *result)
{
    if (!result) return CC2DXX_ERROR;

    uint8_t buffer[4];
    if (cc2dxx_i2c_read(CC2DXX_I2C_ADDR, 0x00, buffer, 4) != CC2DXX_OK)
        return CC2DXX_ERROR;

    uint16_t raw_temp = ((buffer[2] << 6) | (buffer[3] >> 2)) & 0x3FFF;
    uint16_t raw_rh = ((buffer[0] & 0x3F) << 8) | buffer[1];
    
    result->temperature_c = ((raw_temp / 16383.0f) * 165.0f) - 40.0f;
    result->humidity_rh = (raw_rh / 16383.0f) * 100.0f;

    return CC2DXX_OK;
}