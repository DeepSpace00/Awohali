#include <stddef.h>
#include "lan8742.h"

// Private constants
#define LAN8742_TIMEOUT_MS              1000
#define LAN8742_RESET_TIMEOUT_MS        500
#define LAN8742_AUTONEG_TIMEOUT_MS      5000

// MMD register addresses
#define LAN8742_MMD_PCS_DEVICES_PRESENT1    5
#define LAN8742_MMD_PCS_DEVICES_PRESENT2    6
#define LAN8742_MMD_WUCSR                   32784
#define LAN8742_MMD_WUF_CFGA                32785
#define LAN8742_MMD_WUF_CFGB                32786
#define LAN8742_MMD_WUF_MASK_BASE           32801
#define LAN8742_MMD_RX_ADDRA                32865
#define LAN8742_MMD_RX_ADDRB                32866
#define LAN8742_MMD_RX_ADDRC                32867
#define LAN8742_MMD_MCFGR                   32868

// Private function prototypes
static lan8742_status_t lan8742_wait_for_bit_clear(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t bit_mask, uint32_t timeout_ms);
static lan8742_status_t lan8742_wait_for_bit_set(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t bit_mask, uint32_t timeout_ms);
static lan8742_status_t lan8742_modify_reg(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t mask, uint16_t value);
static lan8742_status_t lan8742_modify_mmd_reg(lan8742_handle_t* handle, uint8_t mmd_addr, uint16_t reg_addr, uint16_t mask, uint16_t value);

// Public function implementations

lan8742_status_t lan8742_init(lan8742_handle_t* handle, uint8_t phy_addr)
{
    if (handle == NULL || phy_addr > 31) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    memset(handle, 0, sizeof(lan8742_handle_t));
    handle->phy_address = phy_addr;

    /* Read PHY ID registers to verify device presence */
    lan8742_status_t status = lan8742_read_reg(handle, LAN8742_REG_PHY_ID1, &handle->phy_id1);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    status = lan8742_read_reg(handle, LAN8742_REG_PHY_ID2, &handle->phy_id2);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    /* Verify PHY ID */
    if (handle->phy_id1 != LAN8742_PHY_ID1 || 
        (handle->phy_id2 & LAN8742_PHY_ID2_MASK) != LAN8742_PHY_ID2_BASE) {
        return LAN8742_STATUS_NOT_FOUND;
    }

    handle->initialized = true;
    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_deinit(lan8742_handle_t* handle)
{
    if (handle == NULL) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    /* Put PHY in power down mode */
    lan8742_set_power_down(handle, true);
    
    memset(handle, 0, sizeof(lan8742_handle_t));
    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_soft_reset(lan8742_handle_t* handle)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    /* Set soft reset bit */
    lan8742_status_t status = lan8742_write_reg(handle, LAN8742_REG_BASIC_CONTROL, LAN8742_BCR_SOFT_RESET);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    /* Wait for reset to complete */
    return lan8742_wait_for_bit_clear(handle, LAN8742_REG_BASIC_CONTROL, LAN8742_BCR_SOFT_RESET, LAN8742_RESET_TIMEOUT_MS);
}

lan8742_status_t lan8742_read_reg(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t* data)
{
    if (handle == NULL || data == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    if (!lan8742_platform_mdio_read(handle->phy_address, reg_addr, data)) {
        return LAN8742_STATUS_ERROR;
    }

    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_write_reg(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t data)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    if (!lan8742_platform_mdio_write(handle->phy_address, reg_addr, data)) {
        return LAN8742_STATUS_ERROR;
    }

    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_read_mmd_reg(lan8742_handle_t* handle, uint8_t mmd_addr, uint16_t reg_addr, uint16_t* data)
{
    if (handle == NULL || data == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    if (!lan8742_platform_mmd_read(handle->phy_address, mmd_addr, reg_addr, data)) {
        return LAN8742_STATUS_ERROR;
    }

    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_write_mmd_reg(lan8742_handle_t* handle, uint8_t mmd_addr, uint16_t reg_addr, uint16_t data)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    if (!lan8742_platform_mmd_write(handle->phy_address, mmd_addr, reg_addr, data)) {
        return LAN8742_STATUS_ERROR;
    }

    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_get_link_status(lan8742_handle_t* handle, lan8742_link_status_t* status)
{
    if (handle == NULL || status == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t bsr, pscsr;
    lan8742_status_t result;

    /* Read basic status register */
    result = lan8742_read_reg(handle, LAN8742_REG_BASIC_STATUS, &bsr);
    if (result != LAN8742_STATUS_OK) {
        return result;
    }

    /* Read PHY special control/status register */
    result = lan8742_read_reg(handle, LAN8742_REG_PHY_SPECIAL_CTRL, &pscsr);
    if (result != LAN8742_STATUS_OK) {
        return result;
    }

    /* Extract link status information */
    status->link_up = (bsr & LAN8742_BSR_LINK_STATUS) != 0;
    status->autoneg_complete = (bsr & LAN8742_BSR_AUTONEG_COMPLETE) != 0;
    status->remote_fault = (bsr & LAN8742_BSR_REMOTE_FAULT) != 0;

    /* Extract speed and duplex from special control/status register */
    uint16_t speed_bits = (pscsr & LAN8742_PSCSR_SPEED_MASK) >> 2;
    switch (speed_bits) {
        case 1: /* 10Mbps Half Duplex */
            status->speed = LAN8742_SPEED_10M;
            status->duplex = LAN8742_DUPLEX_HALF;
            break;
        case 2: /* 100Mbps Half Duplex */
            status->speed = LAN8742_SPEED_100M;
            status->duplex = LAN8742_DUPLEX_HALF;
            break;
        case 5: /* 10Mbps Full Duplex */
            status->speed = LAN8742_SPEED_10M;
            status->duplex = LAN8742_DUPLEX_FULL;
            break;
        case 6: /* 100Mbps Full Duplex */
            status->speed = LAN8742_SPEED_100M;
            status->duplex = LAN8742_DUPLEX_FULL;
            break;
        default:
            status->speed = LAN8742_SPEED_10M;
            status->duplex = LAN8742_DUPLEX_HALF;
            break;
    }

    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_config_autoneg(lan8742_handle_t* handle, bool enable, uint16_t advertise)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    lan8742_status_t status;

    if (enable) {
        /* Write advertisement register */
        status = lan8742_write_reg(handle, LAN8742_REG_AUTONEG_ADV, advertise);
        if (status != LAN8742_STATUS_OK) {
            return status;
        }

        /* Enable auto-negotiation */
        status = lan8742_modify_reg(handle, LAN8742_REG_BASIC_CONTROL, 
                                   LAN8742_BCR_AUTONEG_ENABLE, LAN8742_BCR_AUTONEG_ENABLE);
    } else {
        /* Disable auto-negotiation */
        status = lan8742_modify_reg(handle, LAN8742_REG_BASIC_CONTROL, 
                                   LAN8742_BCR_AUTONEG_ENABLE, 0);
    }

    return status;
}

lan8742_status_t lan8742_restart_autoneg(lan8742_handle_t* handle)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    return lan8742_modify_reg(handle, LAN8742_REG_BASIC_CONTROL, 
                             LAN8742_BCR_RESTART_AUTONEG, LAN8742_BCR_RESTART_AUTONEG);
}

lan8742_status_t lan8742_set_manual_mode(lan8742_handle_t* handle, lan8742_speed_t speed, lan8742_duplex_t duplex)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t bcr_value = 0;

    /* Disable auto-negotiation */
    if (speed == LAN8742_SPEED_100M) {
        bcr_value |= LAN8742_BCR_SPEED_SELECT;
    }

    if (duplex == LAN8742_DUPLEX_FULL) {
        bcr_value |= LAN8742_BCR_DUPLEX_MODE;
    }

    return lan8742_modify_reg(handle, LAN8742_REG_BASIC_CONTROL, 
                             LAN8742_BCR_AUTONEG_ENABLE | LAN8742_BCR_SPEED_SELECT | LAN8742_BCR_DUPLEX_MODE,
                             bcr_value);
}

lan8742_status_t lan8742_set_power_down(lan8742_handle_t* handle, bool enable)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t value = enable ? LAN8742_BCR_POWER_DOWN : 0;
    return lan8742_modify_reg(handle, LAN8742_REG_BASIC_CONTROL, LAN8742_BCR_POWER_DOWN, value);
}

lan8742_status_t lan8742_set_isolate(lan8742_handle_t* handle, bool enable)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t value = enable ? LAN8742_BCR_ISOLATE : 0;
    return lan8742_modify_reg(handle, LAN8742_REG_BASIC_CONTROL, LAN8742_BCR_ISOLATE, value);
}

lan8742_status_t lan8742_set_loopback(lan8742_handle_t* handle, bool enable)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t value = enable ? LAN8742_BCR_LOOPBACK : 0;
    return lan8742_modify_reg(handle, LAN8742_REG_BASIC_CONTROL, LAN8742_BCR_LOOPBACK, value);
}

lan8742_status_t lan8742_get_interrupt_status(lan8742_handle_t* handle, lan8742_interrupt_status_t* status)
{
    if (handle == NULL || status == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t isr;
    lan8742_status_t result = lan8742_read_reg(handle, LAN8742_REG_INTERRUPT_SOURCE, &isr);
    if (result != LAN8742_STATUS_OK) {
        return result;
    }

    status->wol_event = (isr & LAN8742_ISR_WOL_EVENT) != 0;
    status->energyon = (isr & LAN8742_ISR_ENERGYON) != 0;
    status->autoneg_complete = (isr & LAN8742_ISR_AUTONEG_COMPLETE) != 0;
    status->remote_fault = (isr & LAN8742_ISR_REMOTE_FAULT) != 0;
    status->link_down = (isr & LAN8742_ISR_LINK_DOWN) != 0;
    status->autoneg_lp_ack = (isr & LAN8742_ISR_AUTONEG_LP_ACK) != 0;
    status->parallel_detect_fault = (isr & LAN8742_ISR_PARALLEL_DETECT_FAULT) != 0;
    status->autoneg_page_rx = (isr & LAN8742_ISR_AUTONEG_PAGE_RX) != 0;

    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_set_interrupt_mask(lan8742_handle_t* handle, uint16_t mask)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    return lan8742_write_reg(handle, LAN8742_REG_INTERRUPT_MASK, mask);
}

lan8742_status_t lan8742_set_edpd(lan8742_handle_t* handle, bool enable)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t value = enable ? LAN8742_MCSR_EDPWRDOWN : 0;
    return lan8742_modify_reg(handle, LAN8742_REG_MODE_CTRL_STATUS, LAN8742_MCSR_EDPWRDOWN, value);
}

lan8742_status_t lan8742_get_symbol_error_count(lan8742_handle_t* handle, uint16_t* count)
{
    if (handle == NULL || count == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    return lan8742_read_reg(handle, LAN8742_REG_SYMBOL_ERR_COUNTER, count);
}

lan8742_status_t lan8742_start_tdr_test(lan8742_handle_t* handle)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    /* Enable TDR test */
    return lan8742_modify_reg(handle, LAN8742_REG_TDR_CTRL_STATUS, 0x8000, 0x8000);
}

lan8742_status_t lan8742_get_tdr_result(lan8742_handle_t* handle, lan8742_tdr_result_t* result)
{
    if (handle == NULL || result == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t tdr_ctrl, tdr_patterns;
    lan8742_status_t status;

    status = lan8742_read_reg(handle, LAN8742_REG_TDR_CTRL_STATUS, &tdr_ctrl);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    status = lan8742_read_reg(handle, LAN8742_REG_TDR_PATTERNS_DELAY, &tdr_patterns);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    result->test_complete = (tdr_ctrl & 0x0002) != 0;
    
    /* Extract cable type from TDR control register */
    uint16_t cable_fault = (tdr_ctrl >> 8) & 0x03;
    switch (cable_fault) {
        case 0:
            result->cable_type = LAN8742_TDR_CABLE_MATCH;
            break;
        case 1:
            result->cable_type = LAN8742_TDR_CABLE_SHORT;
            break;
        case 2:
            result->cable_type = LAN8742_TDR_CABLE_OPEN;
            break;
        default:
            result->cable_type = LAN8742_TDR_CABLE_DEFAULT;
            break;
    }

    result->cable_length = tdr_patterns & 0xFF;

    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_get_cable_length(lan8742_handle_t* handle, uint8_t* length)
{
    if (handle == NULL || length == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t cable_reg;
    lan8742_status_t status = lan8742_read_reg(handle, LAN8742_REG_CABLE_LENGTH, &cable_reg);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    *length = cable_reg & 0xFF;
    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_config_wol(lan8742_handle_t* handle, bool magic_packet_enable, 
                                   bool broadcast_enable, bool perfect_da_enable, bool wakeup_frame_enable)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t wucsr = 0;

    if (magic_packet_enable) {
        wucsr |= 0x0001;
    }
    if (broadcast_enable) {
        wucsr |= 0x0008;
    }
    if (perfect_da_enable) {
        wucsr |= 0x0004;
    }
    if (wakeup_frame_enable) {
        wucsr |= 0x0002;
    }

    return lan8742_write_mmd_reg(handle, LAN8742_MMD_VENDOR_SPECIFIC, LAN8742_MMD_WUCSR, wucsr);
}

lan8742_status_t lan8742_set_wol_mac_addr(lan8742_handle_t* handle, const uint8_t* mac_addr)
{
    if (handle == NULL || mac_addr == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    lan8742_status_t status;

    /* Write MAC address to RX_ADDR registers */
    uint16_t addr_low = (mac_addr[1] << 8) | mac_addr[0];
    uint16_t addr_mid = (mac_addr[3] << 8) | mac_addr[2];
    uint16_t addr_high = (mac_addr[5] << 8) | mac_addr[4];

    status = lan8742_write_mmd_reg(handle, LAN8742_MMD_VENDOR_SPECIFIC, LAN8742_MMD_RX_ADDRA, addr_low);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    status = lan8742_write_mmd_reg(handle, LAN8742_MMD_VENDOR_SPECIFIC, LAN8742_MMD_RX_ADDRB, addr_mid);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    return lan8742_write_mmd_reg(handle, LAN8742_MMD_VENDOR_SPECIFIC, LAN8742_MMD_RX_ADDRC, addr_high);
}

lan8742_status_t lan8742_config_wol_filter(lan8742_handle_t* handle, const lan8742_wol_filter_t* filter)
{
    if (handle == NULL || filter == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    uint16_t wuf_cfg = 0;
    
    if (filter->enable) {
        wuf_cfg |= 0x8000;
    }
    if (filter->addr_match_enable) {
        wuf_cfg |= 0x0080;
    }
    if (filter->multicast_enable) {
        wuf_cfg |= 0x0040;
    }
    if (filter->broadcast_enable) {
        wuf_cfg |= 0x0020;
    }

    wuf_cfg |= (filter->pattern_offset & 0x3F);

    lan8742_status_t status = lan8742_write_mmd_reg(handle, LAN8742_MMD_VENDOR_SPECIFIC, 
                                                   LAN8742_MMD_WUF_CFGA, wuf_cfg);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    /* Write filter CRC */
    status = lan8742_write_mmd_reg(handle, LAN8742_MMD_VENDOR_SPECIFIC, 
                                  LAN8742_MMD_WUF_CFGB, filter->filter_crc);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    /* Write byte mask (128 bits = 8 x 16-bit registers) */
    for (int i = 0; i < 8; i++) {
        uint16_t mask_word = (filter->byte_mask[i*2 + 1] << 8) | filter->byte_mask[i*2];
        status = lan8742_write_mmd_reg(handle, LAN8742_MMD_VENDOR_SPECIFIC, 
                                      LAN8742_MMD_WUF_MASK_BASE + i, mask_word);
        if (status != LAN8742_STATUS_OK) {
            return status;
        }
    }

    return LAN8742_STATUS_OK;
}

lan8742_status_t lan8742_set_auto_mdix(lan8742_handle_t* handle, bool enable)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    /* Auto-MDIX is typically controlled via Special Control Status register */
    uint16_t value = enable ? 0x0008 : 0;
    return lan8742_modify_reg(handle, LAN8742_REG_SPECIAL_CTRL_STATUS, 0x0008, value);
}

lan8742_status_t lan8742_set_manual_mdix(lan8742_handle_t* handle, bool mdix)
{
    if (handle == NULL || !handle->initialized) {
        return LAN8742_STATUS_INVALID_PARAM;
    }

    /* Disable Auto-MDIX first, then set manual mode */
    lan8742_status_t status = lan8742_set_auto_mdix(handle, false);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    /* Set MDIX mode */
    uint16_t value = mdix ? 0x0010 : 0;
    return lan8742_modify_reg(handle, LAN8742_REG_SPECIAL_CTRL_STATUS, 0x0010, value);
}

/* Private function implementations */

static lan8742_status_t lan8742_wait_for_bit_clear(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t bit_mask, uint32_t timeout_ms)
{
    uint32_t start_time = 0; /* Platform should provide a tick counter */
    uint16_t reg_value;

    do {
        lan8742_status_t status = lan8742_read_reg(handle, reg_addr, &reg_value);
        if (status != LAN8742_STATUS_OK) {
            return status;
        }

        if ((reg_value & bit_mask) == 0) {
            return LAN8742_STATUS_OK;
        }

        lan8742_platform_delay_ms(10);
        start_time += 10;

    } while (start_time < timeout_ms);

    return LAN8742_STATUS_TIMEOUT;
}

static lan8742_status_t lan8742_wait_for_bit_set(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t bit_mask, uint32_t timeout_ms)
{
    uint32_t start_time = 0; /* Platform should provide a tick counter */
    uint16_t reg_value;

    do {
        lan8742_status_t status = lan8742_read_reg(handle, reg_addr, &reg_value);
        if (status != LAN8742_STATUS_OK) {
            return status;
        }

        if ((reg_value & bit_mask) != 0) {
            return LAN8742_STATUS_OK;
        }

        lan8742_platform_delay_ms(10);
        start_time += 10;

    } while (start_time < timeout_ms);

    return LAN8742_STATUS_TIMEOUT;
}

static lan8742_status_t lan8742_modify_reg(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t mask, uint16_t value)
{
    uint16_t reg_value;
    
    lan8742_status_t status = lan8742_read_reg(handle, reg_addr, &reg_value);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    reg_value = (reg_value & ~mask) | (value & mask);
    
    return lan8742_write_reg(handle, reg_addr, reg_value);
}

static lan8742_status_t lan8742_modify_mmd_reg(lan8742_handle_t* handle, uint8_t mmd_addr, uint16_t reg_addr, uint16_t mask, uint16_t value)
{
    uint16_t reg_value;
    
    lan8742_status_t status = lan8742_read_mmd_reg(handle, mmd_addr, reg_addr, &reg_value);
    if (status != LAN8742_STATUS_OK) {
        return status;
    }

    reg_value = (reg_value & ~mask) | (value & mask);
    
    return lan8742_write_mmd_reg(handle, mmd_addr, reg_addr, reg_value);
}