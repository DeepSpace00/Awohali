#include "ff.h"
#include "diskio.h"
#include "sd_card.h"
#include "sd_card_constants.h"  // For block_dev_err_t
#include "hw_config.h"

DSTATUS disk_status(const BYTE pdrv) {
    sd_card_t *pSD = sd_get_by_num(pdrv);
    if (!pSD) {
        return STA_NOINIT;
    }
    return pSD->state.m_Status;
}

DSTATUS disk_initialize(const BYTE pdrv) {
    sd_card_t *pSD = sd_get_by_num(pdrv);
    if (!pSD) {
        return STA_NOINIT;
    }
    return pSD->init(pSD);
}

DRESULT disk_read(const BYTE pdrv, BYTE *buff, const LBA_t sector, const UINT count) {
    sd_card_t *pSD = sd_get_by_num(pdrv);
    if (!pSD) {
        return RES_PARERR;
    }

    const block_dev_err_t err = pSD->read_blocks(pSD, buff, sector, count);
    return (err == SD_BLOCK_DEVICE_ERROR_NONE) ? RES_OK : RES_ERROR;
}

DRESULT disk_write(const BYTE pdrv, const BYTE *buff, const LBA_t sector, const UINT count) {
    sd_card_t *pSD = sd_get_by_num(pdrv);
    if (!pSD) {
        return RES_PARERR;
    }

    block_dev_err_t err = pSD->write_blocks(pSD, buff, sector, count);
    if (err != SD_BLOCK_DEVICE_ERROR_NONE) {
        return RES_ERROR;
    }

    // Sync after write
    err = pSD->sync(pSD);
    return (err == SD_BLOCK_DEVICE_ERROR_NONE) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(const BYTE pdrv, const BYTE cmd, void *buff) {
    sd_card_t *pSD = sd_get_by_num(pdrv);
    if (!pSD) {
        return RES_PARERR;
    }

    switch (cmd) {
        case CTRL_SYNC:
            return (pSD->sync(pSD) == SD_BLOCK_DEVICE_ERROR_NONE) ? RES_OK : RES_ERROR;

        case GET_SECTOR_COUNT:
            *(LBA_t *)buff = pSD->get_num_sectors(pSD);
            return RES_OK;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            return RES_OK;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            return RES_OK;

        default:
            return RES_PARERR;
    }
}