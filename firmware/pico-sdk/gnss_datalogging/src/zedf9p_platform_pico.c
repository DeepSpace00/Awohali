/**
 * @file zedf9p_platform_pico.c
 * @brief Pico SDK Platform abstraction layer for ZED-F9P GNSS module driver
 * @author Generated Driver
 * @version 1.0
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "zedf9p/zedf9p_platform.h"

void zedf9p_platform_delay(const uint32_t ms) {
    sleep_ms(ms);
}

uint32_t zedf9p_platform_get_tick(void) {
    return to_ms_since_boot(get_absolute_time());
}

int zedf9p_platform_i2c_write(const uint8_t i2c_addr, const uint8_t *data, const size_t len) {
    const int status = i2c_write_blocking(i2c_default, i2c_addr, data, len, 1);
    return (status != PICO_ERROR_GENERIC) ? -1 : 0;
}

int zedf9p_platform_i2c_read(const uint8_t i2c_addr, uint8_t *data, const size_t len) {
    const int status = i2c_read_blocking(i2c_default, i2c_addr, data, len, 1);
    return (status != PICO_ERROR_GENERIC) ? -1 : 0;
}

bool zedf9p_platform_i2c_init(const uint8_t i2c_addr) {
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    const uint8_t data = 0;

    const int status = zedf9p_platform_i2c_write(i2c_addr, &data, sizeof(data));

    return (status == 0) ? true : false;
}

bool zedf9p_platform_uart_write(const uint8_t *data, const size_t len) {
    const int chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(chan);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, uart_get_dreq(uart0, true));

    dma_channel_configure(chan, &c, &uart_get_hw(uart0)->dr, data, len, true);

    dma_channel_wait_for_finish_blocking(chan);
    dma_channel_unclaim(chan);

    return 0;
}

size_t zed_f9p_platform_uart_read(uint8_t *data, const size_t len, const uint32_t timeout_ms) {
    const int chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(chan);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, uart_get_dreq(uart0, false));

    // Wait with timeout
    const absolute_time_t timeout_time = make_timeout_time_ms(timeout_ms);
    while (dma_channel_is_busy(chan)) {
        if (time_reached(timeout_time)) {
            dma_channel_abort(chan);
            const uint32_t bytes_remaining = dma_channel_hw_addr(chan)->transfer_count;
            const uint32_t bytes_read = len - bytes_remaining;
            dma_channel_unclaim(chan);
            return bytes_read;
        }
        tight_loop_contents();
    }

    dma_channel_unclaim(chan);
    return len;
}