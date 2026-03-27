#ifndef M24XXX_H
#define M24XXX_H

#include <stdint.h>
#include "m24xxx_platform.h"

typedef enum {
    addr0 = 0x50,
    addr1 = 0x51,
    addr2 = 0x52,
    addr3 = 0x53
} M24xxx_i2c_address;

typedef struct {
    uint8_t page_size;
} M24xxx_Memory_Array_t;

void m24xxx_init(M24xxx_i2c_address i2c_addr);

M24xxx_Status m24xxx_read(uint16_t mem_addr, uint8_t *data, uint16_t len);
M24xxx_Status m24xxx_write(uint16_t mem_addr, const uint8_t *data, uint16_t len);

M24xxx_Status m24xxx_page_read(uint16_t page_number, uint8_t *data);
M24xxx_Status m24xxx_page_write(uint16_t page_number, const uint8_t *data);

#endif // M24XXX_H