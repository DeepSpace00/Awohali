#include <Arduino.h>
#include <Wire.h>
#include <sht4x_platform.h>

#define SHT4X_I2C_TIMEOUT 100

SHT4X_Status sht4x_i2c_read(uint8_t dev_addr, uint8_t *data, uint8_t len)
{
    Wire.requestFrom(dev_addr, len);
    uint16_t start = millis();
    uint16_t index = 0;
    while (Wire.available()) {
        data[index++] = Wire.read();
        if (index >= len)
            break;
        if (millis() - start > SHT4X_I2C_TIMEOUT)
            return SHT4X_ERROR;
    }

    return (SHT4X_OK);
}

SHT4X_Status sht4x_i2c_write(uint8_t dev_addr, const uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(data, len);

    if (Wire.endTransmission() == 0)
      return SHT4X_OK;

    else
      return SHT4X_ERROR;
}

void sht4x_delay_ms(uint32_t ms)
{
  delay(ms);
}