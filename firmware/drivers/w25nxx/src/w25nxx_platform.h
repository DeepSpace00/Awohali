#ifndef W25NXX_PLATFORM_H
#define W25NXX_PLATFORM_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    W25Nxx_OK = 0,
    W25Nxx_ERROR = -1,
} W25Nxx_Status;

// User must implement these functions for their platform
void spi_cs_low(void);
void spi_cs_high(void);
void spi_transfer(const uint8_t *tx_buf, uint8_t *rx_buf, size_t length);
void delay_ms(uint32_t ms);

#endif // W25NXX_PLATFORM_H