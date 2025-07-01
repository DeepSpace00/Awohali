#include "w25nxx.h"

static W25Nxx_Status w25n_set_feature(uint8_t addr, uint8_t value);
static W25Nxx_Status w25n_get_feature(uint8_t addr, uint8_t *value);

W25Nxx_Status w25nxx_init(void) {
    return w25n_reset();
}

W25Nxx_Status w25nxx_reset(void) {
    uint8_t cmd = W25NXX_CMD_RESET;
    spi_cs_low();
    spi_transfer(&cmd, NULL, 1);
    spi_cs_high();
    delay_ms(1);
    return w25n_wait_ready(100);
}

W25Nxx_Status w25nxx_read_id(uint8_t *id, uint8_t len) {
    uint8_t cmd = W25NXX_JEDEC_ID;
    uint8_t dummy = 0x00;
    spi_cs_low();
    spi_transfer(&cmd, NULL, 1);
    spi_transfer(&dummy, NULL, 1);
    spi_transfer(NULL, id, len);
    spi_cs_high();
    return W25Nxx_OK;
}

W25Nxx_Status w25nxx_read_status(uint8_t *status) {
    return w25n_get_feature(W25NXX_REG_STATUS, status);
}

W25Nxx_Status w25nxx_write_enable(void) {
    uint8_t cmd = W25NXX_CMD_WRITE_ENABLE;
    spi_cs_low();
    spi_transfer(&cmd, NULL, 1);
    spi_cs_high();
    return W25Nxx_OK;
}

W25Nxx_Status w25nxx_chip_erase(W25Nxx_t flash_info)
{
    for (uint32_t i = 0; i < flash_info.BlockCount; i++)
    {
        if (w25nxx_block_erase(i) != W25Nxx_OK)
            return W25Nxx_ERROR;
    }
    return W25Nxx_OK;
}

W25Nxx_Status w25nxx_sector_erase(W25Nxx_t flash_info, uint32_t sector_addr)
{
    uint32_t block = sector_addr / flash_info.BlockSize;

    return w25nxx_block_erase(block);
}

W25Nxx_Status w25nxx_block_erase(uint32_t block_addr) {
    uint8_t cmd[4];
    cmd[0] = W25NXX_CMD_BLOCK_ERASE;
    cmd[1] = (block_addr >> 16) & 0xFF;
    cmd[2] = (block_addr >> 8) & 0xFF;
    cmd[3] = block_addr & 0xFF;

    if (w25nxx_write_enable() != W25Nxx_OK)
        return false;

    spi_cs_low();
    spi_transfer(cmd, NULL, 4);
    spi_cs_high();

    return w25n_wait_ready(2000);
}

W25Nxx_Status w25nxx_page_read(uint32_t page_addr) {
    uint8_t cmd[4];
    cmd[0] = W25NXX_CMD_PAGE_READ;
    cmd[1] = (page_addr >> 16) & 0xFF;
    cmd[2] = (page_addr >> 8) & 0xFF;
    cmd[3] = page_addr & 0xFF;

    spi_cs_low();
    spi_transfer(cmd, NULL, 4);
    spi_cs_high();

    return w25n_wait_ready(100);
}

W25Nxx_Status w25nxx_read_cache(uint16_t column_addr, uint8_t *buffer, uint16_t length) {
    uint8_t cmd[4];
    cmd[0] = W25NXX_CMD_READ_FROM_CACHE_X4;
    cmd[1] = (column_addr >> 8) & 0xFF;
    cmd[2] = column_addr & 0xFF;
    cmd[3] = 0x00; // Dummy byte

    spi_cs_low();
    spi_transfer(cmd, NULL, 4);
    spi_transfer(NULL, buffer, length);
    spi_cs_high();

    return W25Nxx_OK;
}

W25Nxx_Status w25nxx_program_load(uint16_t column_addr, const uint8_t *buffer, uint16_t length) {
    uint8_t cmd[3];
    cmd[0] = W25NXX_CMD_QUAD_DATA_LOAD;
    cmd[1] = (column_addr >> 8) & 0xFF;
    cmd[2] = column_addr & 0xFF;

    spi_cs_low();
    spi_transfer(cmd, NULL, 3);
    spi_transfer(buffer, NULL, length);
    spi_cs_high();

    return W25Nxx_OK;
}

W25Nxx_Status w25nxx_program_execute(uint32_t page_addr) {
    uint8_t cmd[4];
    cmd[0] = W25NXX_CMD_PROGRAM_EXECUTE;
    cmd[1] = (page_addr >> 16) & 0xFF;
    cmd[2] = (page_addr >> 8) & 0xFF;
    cmd[3] = page_addr & 0xFF;

    if (w25n_write_enable() != W25Nxx_OK)
        return W25Nxx_ERROR;

    spi_cs_low();
    spi_transfer(cmd, NULL, 4);
    spi_cs_high();

    return w25n_wait_ready(1000);
}

W25Nxx_Status w25nxx_wait_ready(uint32_t timeout_ms) {
    uint8_t status;
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        if (w25n_read_status(&status) != W25Nxx_OK) 
            return W25Nxx_ERROR;
        if ((status & W25NXX_STATUS_OIP) == 0)
            return W25Nxx_OK;
        delay_ms(1);
        elapsed++;
    }
    return W25Nxx_ERROR;
}

static W25Nxx_Status w25nxx_set_feature(uint8_t addr, uint8_t value) {
    uint8_t cmd[3];
    cmd[0] = W25NXX_CMD_WRITE_STATUS_REG;
    cmd[1] = addr;
    cmd[2] = value;

    spi_cs_low();
    spi_transfer(cmd, NULL, 3);
    spi_cs_high();

    return W25Nxx_OK;
}

static W25Nxx_Status w25nxx_get_feature(uint8_t addr, uint8_t *value) {
    uint8_t cmd[2];
    cmd[0] = W25NXX_CMD_READ_STATUS_REG;
    cmd[1] = addr;

    spi_cs_low();
    spi_transfer(cmd, NULL, 2);
    spi_transfer(NULL, value, 1);
    spi_cs_high();

    return W25Nxx_OK;
}