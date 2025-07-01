#ifndef LPS22DF_H
#define LPS22DF_H

#include <stdint.h>
#include "lps22df_platform.h"

#define LPS22DF_I2C_ADDR1   0x5C
#define LPS22DF_I2C_ADDR2   0x5D

typedef struct {
    float pressure_hPa;
    float temperature_c;
} LPS22DF_Result;

// Register Addresses
#define LPS22DF_REG_INT_CFG             0x0B
#define LPS22DF_REG_THS_P_L             0x0C
#define LPS22DF_REG_THS_P_H             0x0D
#define LPS22DF_REG_IF_CTRL             0x0E
#define LPS22DF_REG_WHO_AM_I            0x0F
#define LPS22DF_REG_CTRL_REG1           0x10
#define LPS22DF_REG_CTRL_REG2           0x11
#define LPS22DF_REG_CTRL_REG3           0x12
#define LPS22DF_REG_CTRL_REG4           0x13
#define LPS22DF_REG_FIFO_CTRL           0x14
#define LPS22DF_REG_REF_P_L             0x16
#define LPS22DF_REG_REF_P_H             0x17
#define LPS22DF_REG_I3C_IF_CTRL         0x19
#define LPS22DF_REG_RPDS_L              0x1A
#define LPS22DF_REG_RPDS_H              0x1B
#define LPS22DF_REG_INT_SOURCE          0x24
#define LPS22DF_REG_FIFO_STAT1          0x25
#define LPS22DF_REG_FIFO_STAT2          0x26
#define LPS22DF_REG_STATUS              0x27
#define LPS22DF_REG_PRES_OUT_XL         0x28
#define LPS22DF_REG_PRES_OUT_L          0x29
#define LPS22DF_REG_PRES_OUT_H          0x2A
#define LPS22DF_REG_TEMP_OUT_L          0x2B
#define LPS22DF_REG_TEMP_OUT_H          0x2C
#define LPS22DF_REG_FIFO_PRES_OUT_XL    0x78
#define LPS22DF_REG_FIFO_PRES_OUT_L     0x79
#define LPS22DF_REG_FIFO_PRES_OUT_H     0x7A


// High-Level API
LPS22DF_Status lps22df_init(uint8_t i2c_addr);
LPS22DF_Status lps22df_read(uint8_t i2c_addr, LPS22DF_Result *result);

#endif // LPS22DF_H