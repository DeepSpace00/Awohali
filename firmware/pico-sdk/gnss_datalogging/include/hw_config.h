#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "sd_card.h"
#include "../boards/adafruit_metro_rp2350.h"

// SD card hardware configuration
size_t sd_get_num(void);
sd_card_t *sd_get_by_num(size_t num);

// Debug function to check card detection
void sd_debug_card_detect(void);

#endif // HW_CONFIG_H