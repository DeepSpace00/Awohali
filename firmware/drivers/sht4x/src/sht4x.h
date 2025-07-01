#ifndef SHT4X_H
#define SHT4X_H

#include <stdint.h>
#include "sht4x_platform.h"

#define SHT4X_I2C_ADDR  0x44

// Register Addresses
#define SHT4X_REG_MEASURE_HPM       0xFD
#define SHT4X_REG_MEASURE_MPM       0xF6
#define SHT4X_REG_MEASURE_LPM       0xE0
#define SHT4X_REG_HIGH_HEAT_1000MS  0x39
#define SHT4X_REG_HIGH_HEAT_100MS   0x32
#define SHT4X_REG_MED_HEAT_1000MS   0x2F
#define SHT4X_REG_MED_HEAT_100MS    0x24
#define SHT4X_REG_LOW_HEAT_1000MS   0x1E
#define SHT4X_REG_LOW_HEAT_100MS    0x15

#define SHT4X_REG_READ_SERIAL       0x89
#define SHT4X_REG_SOFT_RESET        0x94

typedef enum {
    SHT4X_HIGH_PRECISION,
    SHT4X_MED_PRECISION,
    SHT4X_LOW_PRECISION
} sht4x_precision_t;

typedef enum {
    SHT4X_NO_HEATER,
    SHT4X_HIGH_HEATER_1S,
    SHT4X_HIGH_HEATER_100MS,
    SHT4X_MED_HEATER_1S,
    SHT4X_MED_HEATER_100MS,
    SHT4X_LOW_HEATER_1S,
    SHT4X_LOW_HEATER_100MS
} sht4x_heater_t;

typedef struct {
    float temperature_c;
    float humidity_rh;
} SHT4X_Result;

// High-Level API
SHT4X_Status sht4x_init(void);
SHT4X_Status sht4x_read(SHT4X_Result *result);

#endif // SHT4X_H