#include "sht4x.h"

static uint16_t sht4x_crc_check(const uint8_t *data)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else 
                crc <<= 1;
        }
    }
    return crc;
}

SHT4X_Status sht4x_init(void)
{
    uint8_t cmd = SHT4X_REG_SOFT_RESET;
    if (sht4x_i2c_write(SHT4X_I2C_ADDR, &cmd, 1) != SHT4X_OK)
        return SHT4X_ERROR;
    
    sht4x_delay_ms(2);
    return SHT4X_OK;
}

SHT4X_Status sht4x_read(SHT4X_Result *result)
{
    uint8_t cmd = SHT4X_REG_MEASURE_HPM;
    if (sht4x_i2c_write(SHT4X_I2C_ADDR, &cmd, 1) != SHT4X_OK)
        return SHT4X_ERROR;

    sht4x_delay_ms(10);

    uint8_t data[6];
    if (sht4x_i2c_read(SHT4X_I2C_ADDR, data, 6) != SHT4X_OK)
        return SHT4X_ERROR;

    if (sht4x_crc_check(data) != data[2] || sht4x_crc_check(&data[3]) != data[5])
        return SHT4X_ERROR;
    
    uint16_t raw_temp = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_rh = ((uint16_t)data[3] << 8) | data[4];
    
    result->temperature_c = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    result->humidity_rh = -6.0f + 125.0f * ((float)raw_rh / 65535.0f);

    return SHT4X_OK;
}