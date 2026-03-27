#ifndef W25NXX_H
#define W25NXX_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "w25nxx_platform.h"

// Command Definitions
#define W25NXX_CMD_RESET                0xFF
#define W25NXX_JEDEC_ID                 0x9F
#define W25NXX_CMD_READ_STATUS_REG      0x0F
#define W25NXX_CMD_WRITE_STATUS_REG     0x1F
#define W25NXX_CMD_WRITE_ENABLE         0x06
#define W25NXX_CMD_WRITE_DISABLE        0x04
#define W25NXX_CMD_SWAP_BLOCKS          0xA1
#define W25NXX_CMD_READ_BBM_LUT         0xA5
#define W25NXX_CMD_ECC_FAILURE_ADDR     0xA9
#define W25NXX_CMD_BLOCK_ERASE          0xD8
#define W25NXX_CMD_DATA_LOAD            0x02
#define W25NXX_CMD_QUAD_DATA_LOAD       0x32
#define W25NXX_CMD_PROGRAM_EXECUTE      0x10
#define W25NXX_CMD_PAGE_READ            0x13
#define W25NXX_CMD_READ                 0x03
#define W25NXX_CMD_FAST_READ            0x0B
#define W25NXX_CMD_READ_FROM_CACHE_X4   0x6B

// Feature Register Addresses
#define W25NXX_REG_PROTECTION           0xA0
#define W25NXX_REG_CONFIG               0xB0
#define W25NXX_REG_STATUS               0xC0
#define W25NXX_REG_DIE_SELECT           0xD0

// Status Register Bits
#define W25NXX_STATUS_OIP               (1 << 0)
#define W25NXX_STATUS_WEL               (1 << 1)
#define W25NXX_STATUS_E_FAIL            (1 << 2)
#define W25NXX_STATUS_P_FAIL            (1 << 3)
#define W25NXX_STATUS_ECC_MASK          (3 << 4)

typedef enum {
    W25N10 = 1,
    W25N20,
    W25N40,
    W25N80,
    W25N16,
    W25N32,
    W25N64,
    W25N128,
    W25N256,
    W25N512
} W25NXX_ID_t;

typedef struct {
    W25NXX_ID_t ID;
    uint8_t UniqID[8];
    uint16_t PageSize;
    uint32_t PageCount;
    uint32_t SectorSize;
    uint32_t SectorCount;
    uint32_t BlockSize;
    uint32_t BlockCount;
    uint32_t CapacityInKiloByte;
    uint8_t ProtectionRegister;
    uint8_t ConfigRegister;
    uint8_t StatusRegister;
    uint8_t DieSelectRegister;
    uint8_t Lock;
} W25Nxx_t;

// Configuration
#define W25NXX_PAGE_SIZE                2048
#define W25NXX_SPARE_SIZE               64
#define W25NXX_PAGE_TOTAL_SIZE          (W25N_PAGE_SIZE + W25N_SPARE_SIZE)
#define W25NXX_PAGES_PER_BLOCK          64
#define W25NXX_BLOCK_SIZE               (W25N_PAGE_TOTAL_SIZE * W25N_PAGES_PER_BLOCK)
#define W25NXX_TOTAL_BLOCKS             512

// Public API
W25Nxx_Status w25nxx_init(void);

W25Nxx_Status w25nxx_chip_erase(W25Nxx_t flash_info);
W25Nxx_Status w25nxx_sector_erase(W25Nxx_t flash_info, uint32_t sector_addr);
W25Nxx_Status w25nxx_block_erase(uint32_t block_addr);

W25Nxx_Status w25nxx_reset(void);
W25Nxx_Status w25nxx_read_id(uint8_t *id, uint8_t len);
W25Nxx_Status w25nxx_read_status(uint8_t *status);
W25Nxx_Status w25nxx_write_enable(void);
W25Nxx_Status w25nxx_page_read(uint32_t page_addr);
W25Nxx_Status w25nxx_read_cache(uint16_t column_addr, uint8_t *buffer, uint16_t length);
W25Nxx_Status w25nxx_program_load(uint16_t column_addr, const uint8_t *buffer, uint16_t length);
W25Nxx_Status w25nxx_program_execute(uint32_t page_addr);
W25Nxx_Status w25nxx_wait_ready(uint32_t timeout_ms);

#endif // W25NXX_H
