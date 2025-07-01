#include "lps22df.h"

#define LPS22DF_WHO_AM_I_VAL 0xB4

LPS22DF_Status lps22df_init(uint8_t i2c_addr)
{
    uint8_t id = 0;
    if (lps22df_i2c_read(i2c_addr, LPS22DF_REG_WHO_AM_I, &id, 1) != LPS22DF_OK)
        return LPS22DF_ERROR;
    
    if (id != LPS22DF_WHO_AM_I_VAL)
        return LPS22DF_ERROR;

    // CTRL_REG1: ODR: 10HZ (0x04 << 4), BDU = 1
    uint8_t ctrl1 = (0x04 << 4) | (1 << 1);
    if (lps22df_i2c_write(i2c_addr, LPS22DF_REG_CTRL_REG1, &ctrl1, 1) != LPS22DF_OK)
        return LPS22DF_ERROR;
    
    return LPS22DF_OK;
}

LPS22DF_Status lps22df_read(uint8_t i2c_addr, LPS22DF_Result *result)
{
    if (!result) return LPS22DF_ERROR;

    uint8_t status;
    if (lps22df_i2c_read(i2c_addr, LPS22DF_REG_STATUS, &status, 1) != LPS22DF_OK)
        return LPS22DF_ERROR;

    if (!(status & 0x03))
        return LPS22DF_ERROR;
    
    uint8_t buffer[5];
    if (lps22df_i2c_read(i2c_addr, LPS22DF_REG_PRES_OUT_XL, buffer, 5) != LPS22DF_OK)
        return LPS22DF_ERROR;

    int32_t raw_pressure = ((int32_t)buffer[2] << 16) | ((int32_t)buffer[1] <<8) | buffer[0];
    int16_t raw_temp = ((int16_t)buffer[4] << 8) | buffer[3];

    result->pressure_hPa = raw_pressure / 4096.0f;
    result->temperature_c = raw_temp / 100.0f;

    return LPS22DF_OK;
}