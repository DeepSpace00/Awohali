#ifndef USB2422_PLATFORM_H
#define USB2422_PLATFORM_H

#include <stdint.h>

typedef enum {
    USB2422_OK = 0,
    USB2422_ERROR = -1
} USB2422_Status;

USB2422_Status usb2422_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
USB2422_Status usb2422_i2c_write(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len);

#endif // USB2422_PLATFORM_H