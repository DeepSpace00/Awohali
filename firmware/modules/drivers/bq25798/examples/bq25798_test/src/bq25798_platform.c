#include <Arduino.h>
#include <Wire.h>
#include "bq25798_platform.h"
#include "bq25798.h"

BQ25798_Status bq25798_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return BQ25798_ERROR;

    Wire.requestFrom(dev_addr, (uint8_t)len);
    for (int i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }

    return BQ25798_OK;
}

BQ25798_Status bq25798_i2c_write(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg);
    for (int i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    return Wire.endTransmission() == 0 ? BQ25798_OK : BQ25798_ERROR;
}
