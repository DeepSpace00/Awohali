#include "m24xxx.h"

static uint8_t m24xxx_i2c_addr = 0;
static uint8_t m24xxx_page_size = 0;

void m24xxx_init(M24xxx_i2c_address i2c_addr, M24xxx_Memory_Array_t memory_array)
{
    m24xxx_i2c_addr = i2c_addr;
    m24xxx_page_size = memory_array.page_size;
}

M24xxx_Status m24xxx_read(uint16_t mem_addr, uint8_t *data, uint16_t len)
{
    uint8_t addr_buffer[2] = {(uint8_t)((mem_addr >> 8) & 0xFF), (uint8_t)(mem_addr & 0xFF)};
    
    if (m24xxx_i2c_write(m24xxx_i2c_addr, addr_buffer, 2) != M24xxx_OK)
        return M24xxx_ERROR;
    
    return m24xxx_i2c_read(m24xxx_i2c_addr, data, len);
}

M24xxx_Status m24xxx_write(uint16_t mem_addr, const uint8_t *data, uint16_t len)
{
    if (len > m24xxx_page_size)
        return M24xxx_ERROR;

    uint8_t buffer[m24xxx_page_size + 2];
    buffer[0] = (mem_addr >> 8) & 0xFF; // MSB
    buffer[1] = (mem_addr & 0xFF);      // LSB

    for (uint16_t i = 0; i < len; i++)
        buffer[2 + i] = data[i];

    return m24xxx_i2c_write(m24xxx_i2c_addr, buffer, len);
}

M24xxx_Status m24xxx_page_read(uint16_t page_number, uint8_t *data)
{
    if (page_number >= (0x10000 / m24xxx_page_size))
        return M24xxx_ERROR;
    
    uint16_t mem_addr = page_number * m24xxx_page_size;
    
    return m24xxx_read(mem_addr, data, m24xxx_page_size);
}

M24xxx_Status m24xxx_page_write(uint16_t page_number, const uint8_t *data)
{
    if (page_number >= (0x10000 / m24xxx_page_size))
        return M24xxx_ERROR;

    uint16_t mem_addr = page_number * m24xxx_page_size;

    return m24xxx_write(mem_addr, data, m24xxx_page_size);
}