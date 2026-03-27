#ifndef LAN8742_PLATFORM_H
#define LAN8742_PLATFORM_H

#include <stdint.h>
#include <stdbool.h>

void lan8742_platform_delay_ms(uint32_t ms);

void lan8742_platform_delay_us(uint32_t us);

bool lan8742_platform_mdio_read(uint8_t phy_addr, uint8_t reg_addr, uint16_t* data);
bool lan8742_platform_mdio_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data);
bool lan8742_platform_mmd_read(uint8_t phy_addr, uint8_t mmd_addr, uint16_t reg_addr, uint16_t* data);
bool lan8742_platform_mmd_write(uint8_t phy_addr, uint8_t mmd_addr, uint16_t reg_addr, uint16_t data);

void lan8742_platform_debug_print(const char* format, ...);

#endif // LAN8742_PLATFORM_H