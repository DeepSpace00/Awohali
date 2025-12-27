//
// Created by deepspace on 2025-12-22.
//

#include "ff.h"
#include "hw_config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

DWORD get_fattime(void) {
    return ((DWORD)(2025 - 1980) << 25)
        | ((DWORD)1 << 21)
        | ((DWORD)1 << 16)
        | ((DWORD)1 << 11)
        | ((DWORD)1 << 5)
        | ((DWORD)1 << 1);
}

// Define the SD card using SPI
static spi_t spi = {
    .hw_inst = spi0,
    .sck_gpio = PICO_SD_SCK_PIN,
    .mosi_gpio = PICO_SD_MOSI_PIN,
    .miso_gpio = PICO_SD_MISO_PIN,
    .baud_rate = 12 * 1000 * 1000,  // 12 MHz

    // Enable DMA
    .use_static_dma_channels = false,
    .tx_dma = 0,
    .rx_dma = 1
};

static sd_spi_if_t spi_if = {
    .spi = &spi,
    .ss_gpio = PICO_SD_CS_PIN
};

static sd_card_t sd_card = {
    .type = SD_IF_SPI,
    .spi_if_p = &spi_if,
    .use_card_detect = true,
    .card_detect_gpio = PICO_SD_CD_PIN,
    .card_detected_true = 0,        // Card present when pin reads LOW
    .card_detect_use_pull = true,   // Use internal pull resistor
    .card_detect_pull_hi = true     // Pull HIGH (pin reads HIGH when card removed)
};

size_t sd_get_num() { return 1; }

sd_card_t *sd_get_by_num(const size_t num) {
    return (num == 0) ? &sd_card : NULL;
}

// Initialize card detect pin
void sd_init_card_detect(void) {
    if (sd_card.use_card_detect) {
        gpio_init(sd_card.card_detect_gpio);
        gpio_set_dir(sd_card.card_detect_gpio, GPIO_IN);

        if (sd_card.card_detect_use_pull) {
            if (sd_card.card_detect_pull_hi) {
                gpio_pull_up(sd_card.card_detect_gpio);
            } else {
                gpio_pull_down(sd_card.card_detect_gpio);
            }
        } else {
            gpio_disable_pulls(sd_card.card_detect_gpio);
        }

        // Small delay for pull resistor to stabilize
        sleep_ms(10);
    }
}

// Debug function to check card detection
void sd_debug_card_detect(void) {
    printf("=== SD Card Detection Debug ===\n");
    printf("CD Pin (GPIO %d) state: %d\n", PICO_SD_CD_PIN, gpio_get(PICO_SD_CD_PIN));
    printf("Card detected when pin = %d\n", sd_card.card_detected_true);
    printf("Pull resistor: %s\n", sd_card.card_detect_pull_hi ? "HIGH" : "LOW");

    const bool card_present = sd_card_detect(&sd_card);
    printf("Card detection result: %s\n", card_present ? "CARD PRESENT" : "NO CARD");
    printf("==============================\n");
}