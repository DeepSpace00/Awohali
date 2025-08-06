/**
 * @file usb2422.c
 * @brief USB2422 USB Hub Controller Driver Implementation
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-06
 */

#include <math.h>
#include <stdbool.h>
#include "usb2422.h"

// Register Addresses
#define USB2422_REG_VID_LSB         0x00    // Vendor ID Least Significant Bit
#define USB2422_REG_VID_MSB         0x01    // Vendor ID Most Significant Bit
#define USB2422_REG_PID_LSB         0x02    // Product ID Least Significant Bit
#define USB2422_REG_PID_MSB         0x03    // Product ID Most Significant Bit
#define USB2422_REG_DID_LSB         0x04    // Device ID Least Significant Bit
#define USB2422_REG_DID_MSB         0x05    // Device ID Most Significant Bit
#define USB2422_REG_CFG1            0x06    // Configuration Data Byte 1
#define USB2422_REG_CFG2            0x07    // Configuration Data Byte 2
#define USB2422_REG_CFG3            0x08    // Configuration Data Byte 3
#define USB2422_REG_NRD             0x09    // Non-Removable Device
#define USB2422_REG_PDS             0x0A    // Port Disable for Self-Powered Operation
#define USB2422_REG_PDB             0x0B    // Port Disable for Bus-Powered Operation
#define USB2422_REG_MAXPS           0x0C    // Max Power for Self-Powered Operation
#define USB2422_REG_MAXPB           0x0D    // Max Power for Bus-Powered Operation
#define USB2422_REG_HCMCS           0x0E    // Hub Controller Max Current for Self-Powered Operation
#define USB2422_REG_HCMCB           0x0F    // Hub Controller Max Power for Bus-Powered Operation
#define USB2422_REG_PWRT            0x10    // Power-On Time
#define USB2422_REG_LANGIDH         0x11    // Language ID High
#define USB2422_REG_LANGIDL         0x12    // Language ID Low
#define USB2422_REG_MFRSL           0x13    // Manufacturer String Length
#define USB2422_REG_PRDSL           0x14    // Product String Length
#define USB2422_REG_SERSL           0x15    // Serial String Length
#define USB2422_REG_MANSTR_START    0x16    // Manufacturer String Start (0x16-0x53)
#define USB2422_REG_PRDSTR_START    0x54    // Product String Start (0x54-0x91)
#define USB2422_REG_SERSTR_START    0x92    // Serial String Start (0x92-0xCF)
#define USB2422_REG_BC_EN           0xD0    // Battery Charging Enable
#define USB2422_REG_BOOSTUP         0xF6    // Boost Upstream
#define USB2422_REG_BOOST40         0xF8    // Boost Downstream
#define USB2422_REG_PRTSP           0xFA    // Port Swap
#define USB2422_REG_PRTR12          0xFB    // Port 1/2 Remap
#define USB2422_REG_STCD            0xFF    // Status/Command

// CFG1 Register Bit Masks
#define USB2422_CFG1_SELF_BUS_PWR   (1 << 7)
#define USB2422_CFG1_HS_DISABLE     (1 << 5)
#define USB2422_CFG1_MTT_ENABLE     (1 << 4)
#define USB2422_CFG1_EOP_DISABLE    (1 << 3)
#define USB2422_CFG1_CURRENT_SNS_MASK   (0x03 << 1)
#define USB2422_CFG1_PORT_PWR       (1 << 0)

// CFG2 Register Bit Masks
#define USB2422_CFG2_DYNAMIC        (1 << 7)
#define USB2422_CFG2_OC_TIMER_MASK  (0x03 << 4)
#define USB2422_CFG2_COMPOUND       (1 << 3)

// CFG3 Register Bit Masks
#define USB2422_CFG3_PRTMAP_EN      (1 << 3)
#define USB2422_CFG3_STRING_EN      (1 << 0)

// NRD Register Bit Masks
#define USB2422_NRD_PORT2_NR        (1 << 2)
#define USB2422_NRD_PORT1_NR        (1 << 1)

// PDS/PDB Register Bit Masks
#define USB2422_PDS_PORT2_DIS       (1 << 2)
#define USB2422_PDS_PORT1_DIS       (1 << 1)

// BC_EN Register Bit Masks
#define USB2422_BC_EN_PORT2         (1 << 2)
#define USB2422_BC_EN_PORT1         (1 << 1)

// BOOST40 Register Bit Masks
#define USB2422_BOOST40_PORT2_MASK  (0x03 << 2)
#define USB2422_BOOST40_PORT1_MASK  (0x03 << 0)

// PRTSP Register Bit Masks
#define USB2422_PRTSP_PORT2_SWAP    (1 << 2)
#define USB2422_PRTSP_PORT1_SWAP    (1 << 1)
#define USB2422_PRTSP_UPSTREAM_SWAP (1 << 0)

// PRTR12 Register Bit Masks
#define USB2422_PRTR12_PORT2_MASK   (0x0F << 4)
#define USB2422_PRTR12_PORT1_MASK   (0x0F << 0)

// STCD Register Bit Masks
#define USB2422_STCD_INTF_PW_DN     (1 << 2)
#define USB2422_STCD_RESET          (1 << 1)
#define USB2422_STCD_USB_ATTACH     (1 << 0)

// Private helper functions
/**
 * @brief Helper function to write to a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Value to write to a register
 * @return
 */
static usb2422_status_t usb2422_write_register(const usb2422_t *dev, const uint8_t reg, const uint8_t value) {
    const uint8_t data[2] = {reg, value};
    return dev->io.i2c_write(dev->i2c_address, data, 2) == 0 ? USB2422_OK : USB2422_ERR_I2C;
}

/**
 * @brief Helper function to read data from a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Pointer to data variable
 * @return
 */
static usb2422_status_t usb2422_read_register(const usb2422_t *dev, const uint8_t reg, uint8_t *value) {
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return USB2422_ERR_I2C;
    return dev->io.i2c_read(dev->i2c_address, value, 1) == 0 ? USB2422_OK : USB2422_ERR_I2C;
}

static usb2422_status_t usb2422_read_cfg_registers(const usb2422_t *dev, usb2422_cfg_regs_t *regs) {
    uint8_t cfg1_val, cfg2_val, cfg3_val;

    usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_CFG1, &cfg1_val);
    if (status != USB2422_OK) return status;

    status = usb2422_read_register(dev, USB2422_REG_CFG2, &cfg2_val);
    if (status != USB2422_OK) return status;

    status = usb2422_read_register(dev, USB2422_REG_CFG3, &cfg3_val);
    if (status != USB2422_OK) return status;

    // Parse CFG1
    regs->cfg1.self_bus_pwr = (cfg1_val & USB2422_CFG1_SELF_BUS_PWR) ? 1 : 0;
    regs->cfg1.hs_disable = (cfg1_val & USB2422_CFG1_HS_DISABLE) ? 1 : 0;
    regs->cfg1.mtt_enable = (cfg1_val & USB2422_CFG1_MTT_ENABLE) ? 1 : 0;
    regs->cfg1.eop_disable = (cfg1_val & USB2422_CFG1_EOP_DISABLE) ? 1 : 0;
    regs->cfg1.current_sns = (cfg1_val & USB2422_CFG1_CURRENT_SNS_MASK) >> 1;
    regs->cfg1.port_pwr = (cfg1_val & USB2422_CFG1_PORT_PWR) ? 1 : 0;

    // Parse CFG2
    regs->cfg2.dynamic = (cfg2_val & USB2422_CFG2_DYNAMIC) ? 1 : 0;
    regs->cfg2.oc_timer = (cfg2_val & USB2422_CFG2_OC_TIMER_MASK) >> 4;
    regs->cfg2.compound = (cfg2_val & USB2422_CFG2_COMPOUND) ? 1 : 0;

    // Parse CFG3
    regs->cfg3.prtmap_en = (cfg3_val & USB2422_CFG3_PRTMAP_EN) ? 1 : 0;
    regs->cfg3.string_en = (cfg3_val & USB2422_CFG3_STRING_EN) ? 1 : 0;

    return USB2422_OK;
}

static usb2422_status_t usb2422_write_cfg_registers(const usb2422_t *dev, const usb2422_cfg_regs_t *regs) {
    uint8_t cfg1_val = 0, cfg2_val = 0, cfg3_val = 0;

    // Build CFG1
    if (regs->cfg1.self_bus_pwr) cfg1_val |= USB2422_CFG1_SELF_BUS_PWR;
    if (regs->cfg1.hs_disable) cfg1_val |= USB2422_CFG1_HS_DISABLE;
    if (regs->cfg1.mtt_enable) cfg1_val |= USB2422_CFG1_MTT_ENABLE;
    if (regs->cfg1.eop_disable) cfg1_val |= USB2422_CFG1_EOP_DISABLE;
    cfg1_val |= (regs->cfg1.current_sns & 0x03) << 1;
    if (regs->cfg1.port_pwr) cfg1_val |= USB2422_CFG1_PORT_PWR;

    // Build CFG2
    if (regs->cfg2.dynamic) cfg2_val |= USB2422_CFG2_DYNAMIC;
    cfg2_val |= (regs->cfg2.oc_timer & 0x03) << 4;
    if (regs->cfg2.compound) cfg2_val |= USB2422_CFG2_COMPOUND;

    // Build CFG3
    if (regs->cfg3.prtmap_en) cfg3_val |= USB2422_CFG3_PRTMAP_EN;
    if (regs->cfg3.string_en) cfg3_val |= USB2422_CFG3_STRING_EN;

    usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_CFG1, cfg1_val);
    if (status != USB2422_OK) return status;

    status = usb2422_write_register(dev, USB2422_REG_CFG2, cfg2_val);
    if (status != USB2422_OK) return status;

    return usb2422_write_register(dev, USB2422_REG_CFG3, cfg3_val);
}

// Public function implementations (in order of .h file)

const char* usb2422_stat_error(const usb2422_status_t status) {
    switch (status) {
        case USB2422_OK:              return "OK";
        case USB2422_ERR_I2C:         return "I2C communication failed";
        case USB2422_ERR_TIMEOUT:     return "Timeout occurred";
        case USB2422_ERR_NULL:        return "Null pointer";
        case USB2422_ERR_INVALID_ARG: return "Invalid argument";
        default:                      return "Unknown error";
    }
}

usb2422_status_t usb2422_init(usb2422_t *dev, const uint8_t address, const usb2422_interface_t io) {
    if (!dev || !io.i2c_write || !io.i2c_read || !io.delay_ms) return USB2422_ERR_NULL;

    dev->i2c_address = address ? address : USB2422_SMBUS_ADDRESS;
    dev->io = io;
    dev->initialized = false;

    // Test communication by reading a register
    uint8_t test_val;
    const usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_VID_LSB, &test_val);
    if (status != USB2422_OK) return status;

    dev->initialized = true;
    return USB2422_OK;
}

usb2422_status_t usb2422_get_vendor_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    uint8_t lsb, msb;

    usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_VID_LSB, &lsb);
    if (status != USB2422_OK) return status;

    status = usb2422_read_register(dev, USB2422_REG_VID_MSB, &msb);
    if (status != USB2422_OK) return status;

    hub_settings->vendor_id = (uint16_t)msb << 8 | lsb;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_vendor_id(usb2422_t *dev, const uint16_t vendor_id) {
    if (!dev) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_VID_LSB, vendor_id & 0xFF);
    if (status != USB2422_OK) return status;

    return usb2422_write_register(dev, USB2422_REG_VID_MSB, (vendor_id >> 8) & 0xFF);
}

usb2422_status_t usb2422_get_product_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    uint8_t lsb, msb;

    usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_PID_LSB, &lsb);
    if (status != USB2422_OK) return status;

    status = usb2422_read_register(dev, USB2422_REG_PID_MSB, &msb);
    if (status != USB2422_OK) return status;

    hub_settings->product_id = (uint16_t)msb << 8 | lsb;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_product_id(usb2422_t *dev, const uint16_t product_id) {
    if (!dev) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_PID_LSB, product_id & 0xFF);
    if (status != USB2422_OK) return status;

    return usb2422_write_register(dev, USB2422_REG_PID_MSB, (product_id >> 8) & 0xFF);
}

usb2422_status_t usb2422_get_device_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    uint8_t lsb, msb;

    usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_DID_LSB, &lsb);
    if (status != USB2422_OK) return status;

    status = usb2422_read_register(dev, USB2422_REG_DID_MSB, &msb);
    if (status != USB2422_OK) return status;

    hub_settings->device_id = (uint16_t)msb << 8 | lsb;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_device_id(usb2422_t *dev, const uint16_t device_id) {
    if (!dev) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_DID_LSB, device_id & 0xFF);
    if (status != USB2422_OK) return status;

    return usb2422_write_register(dev, USB2422_REG_DID_MSB, (device_id >> 8) & 0xFF);
}

usb2422_status_t usb2422_get_config_registers(usb2422_t *dev, usb2422_cfg_regs_t *regs) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    return usb2422_read_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_port_pwr(usb2422_t *dev, usb2422_cfg_regs_t *regs, const bool value) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg1.port_pwr = value ? 1 : 0;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_current_sns(usb2422_t *dev, usb2422_cfg_regs_t *regs, const usb2422_config_current_sns_value_t value) {
    if (!dev || !regs) return USB2422_ERR_NULL;
    if (value > USB2422_CURRENT_SNS_OC_NOT_SUP) return USB2422_ERR_INVALID_ARG;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg1.current_sns = (uint8_t)value;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_eop_disable(usb2422_t *dev, usb2422_cfg_regs_t *regs, const bool value) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg1.eop_disable = value ? 1 : 0;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_mtt_enable(usb2422_t *dev, usb2422_cfg_regs_t *regs, const bool value) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg1.mtt_enable = value ? 1 : 0;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_hs_disable(usb2422_t *dev, usb2422_cfg_regs_t *regs, const bool value) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg1.hs_disable = value ? 1 : 0;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_self_bus_pwr(usb2422_t *dev, usb2422_cfg_regs_t *regs, const bool value) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg1.self_bus_pwr = value ? 1 : 0;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_compound(usb2422_t *dev, usb2422_cfg_regs_t *regs, const bool value) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg2.compound = value ? 1 : 0;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_oc_timer(usb2422_t *dev, usb2422_cfg_regs_t *regs, const usb2422_config_oc_timer_value_t value) {
    if (!dev || !regs) return USB2422_ERR_NULL;
    if (value > USB2422_OC_TIMER_16MS) return USB2422_ERR_INVALID_ARG;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg2.oc_timer = (uint8_t)value;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_dynamic(usb2422_t *dev, usb2422_cfg_regs_t *regs, const bool value) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg2.dynamic = value ? 1 : 0;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_string_en(usb2422_t *dev, usb2422_cfg_regs_t *regs, const bool value) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg3.string_en = value ? 1 : 0;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_set_prtmap_en(usb2422_t *dev, usb2422_cfg_regs_t *regs, const bool value) {
    if (!dev || !regs) return USB2422_ERR_NULL;

    const usb2422_status_t status = usb2422_read_cfg_registers(dev, regs);
    if (status != USB2422_OK) return status;

    regs->cfg3.prtmap_en = value ? 1 : 0;
    return usb2422_write_cfg_registers(dev, regs);
}

usb2422_status_t usb2422_get_non_removable_device(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings) {
    if (!dev || !downstream_port_settings) return USB2422_ERR_NULL;

    uint8_t nrd_val;
    const usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_NRD, &nrd_val);
    if (status != USB2422_OK) return status;

    downstream_port_settings->port_1_non_removable = (nrd_val & USB2422_NRD_PORT1_NR) ? true : false;
    downstream_port_settings->port_2_non_removable = (nrd_val & USB2422_NRD_PORT2_NR) ? true : false;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_non_removable_device(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings, const bool port_1_non_removable, const bool port_2_non_removable) {
    if (!dev || !downstream_port_settings) return USB2422_ERR_NULL;

    uint8_t nrd_val = 0;

    if (port_1_non_removable) nrd_val |= USB2422_NRD_PORT1_NR;
    if (port_2_non_removable) nrd_val |= USB2422_NRD_PORT2_NR;

    const usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_NRD, nrd_val);
    if (status != USB2422_OK) return status;

    downstream_port_settings->port_1_non_removable = port_1_non_removable ? 1 : 0;
    downstream_port_settings->port_2_non_removable = port_2_non_removable ? 1 : 0;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_port_disable_self_powered(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings) {
    if (!dev || !downstream_port_settings) return USB2422_ERR_NULL;

    uint8_t pds_val;
    const usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_PDS, &pds_val);
    if (status != USB2422_OK) return status;

    downstream_port_settings->port_1_disable_self_powered = (pds_val & USB2422_PDS_PORT1_DIS) ? true : false;
    downstream_port_settings->port_2_disable_self_powered = (pds_val & USB2422_PDS_PORT2_DIS) ? true : false;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_port_disable_self_powered(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings, const bool port_1_disable_self_powered, const bool port_2_disable_self_powered) {
    if (!dev || !downstream_port_settings) return USB2422_ERR_NULL;

    uint8_t pds_val = 0;

    if (port_1_disable_self_powered) pds_val |= USB2422_PDS_PORT1_DIS;
    if (port_2_disable_self_powered) pds_val |= USB2422_PDS_PORT2_DIS;

    const usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_PDS, pds_val);
    if (status != USB2422_OK) return status;

    downstream_port_settings->port_1_disable_self_powered = port_1_disable_self_powered ? 1 : 0;
    downstream_port_settings->port_2_disable_self_powered = port_2_disable_self_powered ? 1 : 0;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_port_disable_bus_powered(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings) {
    if (!dev || !downstream_port_settings) return USB2422_ERR_NULL;

    uint8_t pdb_val;
    const usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_PDB, &pdb_val);
    if (status != USB2422_OK) return status;

    downstream_port_settings->port_1_disable_bus_powered = (pdb_val & USB2422_PDS_PORT1_DIS) ? true : false;
    downstream_port_settings->port_2_disable_bus_powered = (pdb_val & USB2422_PDS_PORT2_DIS) ? true : false;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_port_disable_bus_powered(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings, const bool port_1_disable_bus_powered, const bool port_2_disable_bus_powered) {
    if (!dev || !downstream_port_settings) return USB2422_ERR_NULL;

    uint8_t pdb_val = 0;

    if (port_1_disable_bus_powered) pdb_val |= USB2422_PDS_PORT1_DIS;
    if (port_2_disable_bus_powered) pdb_val |= USB2422_PDS_PORT2_DIS;

    const usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_PDB, pdb_val);
    if (status != USB2422_OK) return status;

    downstream_port_settings->port_1_disable_bus_powered = port_1_disable_bus_powered ? 1 : 0;
    downstream_port_settings->port_2_disable_bus_powered = port_2_disable_bus_powered ? 1 : 0;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_max_power_self_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    uint8_t val;

    static usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_MAXPS, &val);
    if (status != USB2422_OK) return status;

    power_settings->max_curr_bus_powered = val * 2;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_max_power_self_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings, const uint8_t max_curr_self_powered) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    if (max_curr_self_powered > 100) return USB2422_ERR_INVALID_ARG; // Max 100mA (50 * 2mA increments)

    const uint8_t val = round(max_curr_self_powered / 2);

    static usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_MAXPS, val);
    if (status != USB2422_OK) return status;

    power_settings->max_curr_self_powered = max_curr_self_powered;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_max_power_bus_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    uint8_t val;

    static usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_MAXPB, &val);
    if (status != USB2422_OK) return status;

    power_settings->max_curr_bus_powered = val * 2;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_max_power_bus_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings, const uint8_t max_curr_bus_powered) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    const uint8_t val = round(max_curr_bus_powered / 2);

    static usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_MAXPB, val);
    if (status != USB2422_OK) return status;

    power_settings->max_curr_bus_powered = max_curr_bus_powered;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_hub_controller_max_current_self_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    uint8_t val;

    static usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_HCMCS, &val);
    if (status != USB2422_OK) return status;

    power_settings->hub_max_curr_self_powered = val * 2;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_hub_controller_max_current_self_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings, const uint8_t hub_max_curr_self_powered) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    if (hub_max_curr_self_powered > 100) return USB2422_ERR_INVALID_ARG; // Max 100mA (50 * 2mA increments)

    const uint8_t val = round(hub_max_curr_self_powered / 2);

    static usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_HCMCS, val);
    if (status != USB2422_OK) return status;

    power_settings->hub_max_curr_self_powered = hub_max_curr_self_powered;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_hub_controller_max_current_bus_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    uint8_t val;

    static usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_HCMCB, &val);
    if (status != USB2422_OK) return status;

    power_settings->hub_max_curr_bus_powered = val * 2;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_hub_controller_max_current_bus_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings, const uint8_t hub_max_curr_bus_powered) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    const uint8_t val = round(hub_max_curr_bus_powered / 2);

    static usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_HCMCB, val);
    if (status != USB2422_OK) return status;

    power_settings->hub_max_curr_bus_powered = hub_max_curr_bus_powered;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_power_on_time(usb2422_t *dev, usb2422_power_settings_t *power_settings) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    uint8_t val;

    static usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_PWRT, &val);
    if (status != USB2422_OK) return status;

    power_settings->power_on_time = val * 2;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_power_on_time(usb2422_t *dev, usb2422_power_settings_t *power_settings, const uint8_t power_on_time) {
    if (!dev || !power_settings) return USB2422_ERR_NULL;

    const uint8_t val = round(power_on_time / 2);

    static usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_PWRT, val);
    if (status != USB2422_OK) return status;

    power_settings->power_on_time = power_on_time;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_language_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    uint8_t lsb, msb;

    usb2422_status_t status = usb2422_read_register(dev, USB2422_REG_LANGIDL, &lsb);
    if (status != USB2422_OK) return status;

    status = usb2422_read_register(dev, USB2422_REG_LANGIDH, &msb);
    if (status != USB2422_OK) return status;

    hub_settings->language_id = (uint16_t)msb << 8 | lsb;

    return USB2422_OK;
}

usb2422_status_t usb2422_set_language_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, const uint16_t language_id) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_LANGIDL, language_id & 0xFF);
    if (status != USB2422_OK) return status;

    status = usb2422_write_register(dev, USB2422_REG_LANGIDH, (language_id >> 8) & 0xFF);
    if (status != USB2422_OK) return status;

    hub_settings->language_id = (uint16_t)language_id & 0xFF;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_manufacturer_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    return usb2422_read_register(dev, USB2422_REG_MFRSL, &hub_settings->manufacturer_name_len);
}

usb2422_status_t usb2422_set_manufacturer_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, const uint8_t manufacturer_name_len) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    if (manufacturer_name_len > 31) return USB2422_ERR_INVALID_ARG; // Max 31 characters

    static usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_MFRSL, manufacturer_name_len);
    if (status != USB2422_OK) return status;

    hub_settings->manufacturer_name_len = manufacturer_name_len;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_product_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    return usb2422_read_register(dev, USB2422_REG_PRDSL, &hub_settings->product_name_len);
}

usb2422_status_t usb2422_set_product_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, const uint8_t product_name_len) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    if (product_name_len > 31) return USB2422_ERR_INVALID_ARG; // Max 31 characters

    static usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_PRDSL, product_name_len);
    if (status != USB2422_OK) return status;

    hub_settings->product_name_len = product_name_len;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_serial_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    return usb2422_read_register(dev, USB2422_REG_SERSL, &hub_settings->serial_number_len);
}

usb2422_status_t usb2422_set_serial_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, const uint8_t serial_number_len) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    if (serial_number_len > 31) return USB2422_ERR_INVALID_ARG; // Max 31 characters

    static usb2422_status_t status = usb2422_write_register(dev, USB2422_REG_SERSL, serial_number_len);
    if (status != USB2422_OK) return status;

    hub_settings->serial_number_len = serial_number_len;

    return USB2422_OK;
}

usb2422_status_t usb2422_get_manufacturer_name(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    usb2422_status_t status = usb2422_get_manufacturer_string_length(dev, hub_settings);
    if (status != USB2422_OK) return status;

    if (hub_settings->manufacturer_name_len == 0) {
        (hub_settings->manufacturer_name)[0] = '\0';
        return USB2422_OK;
    }

    // Read UTF-16LE string data (2 bytes per character)
    uint8_t utf16_data[62]; // Max 31 characters * 2 bytes
    for (uint8_t i = 0; i < hub_settings->manufacturer_name_len * 2; i++) {
        status = usb2422_read_register(dev, USB2422_REG_MANSTR_START + i, &utf16_data[i]);
        if (status != USB2422_OK) return status;
    }

    // Convert UTF-16LE to ASCII (simple conversion, ignoring high byte)
    for (uint8_t i = 0; i < hub_settings->manufacturer_name_len && i < 31; i++) {
        (hub_settings->manufacturer_name)[i] = (char)utf16_data[i * 2]; // Take LSB only
    }
    (hub_settings->manufacturer_name)[hub_settings->manufacturer_name_len] = '\0';

    return USB2422_OK;
}

usb2422_status_t usb2422_set_manufacturer_name(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, char manufacturer_name[32]) {
    if (!dev || !hub_settings || !manufacturer_name) return USB2422_ERR_NULL;

    uint8_t length = 0;
    while (manufacturer_name[length] != '\0' && length < 31) {
        length++;
    }

    // Set string length first
    usb2422_status_t status = usb2422_set_manufacturer_string_length(dev, hub_settings, length);
    if (status != USB2422_OK) return status;

    // Write UTF-16LE string data (ASCII to UTF-16LE conversion)
    for (uint8_t i = 0; i < length; i++) {
        // Write LSB (ASCII character)
        status = usb2422_write_register(dev, USB2422_REG_MANSTR_START + (i * 2), (uint8_t)manufacturer_name[i]);
        if (status != USB2422_OK) return status;
        hub_settings->manufacturer_name[i] = manufacturer_name[i];

        // Write MSB (0 for ASCII)
        status = usb2422_write_register(dev, USB2422_REG_MANSTR_START + (i * 2) + 1, 0x00);
        if (status != USB2422_OK) return status;
    }

    return USB2422_OK;
}

usb2422_status_t usb2422_get_product_name(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    usb2422_status_t status = usb2422_get_product_string_length(dev, hub_settings);
    if (status != USB2422_OK) return status;

    if (hub_settings->product_name_len == 0) {
        (hub_settings->product_name)[0] = '\0';
        return USB2422_OK;
    }

    // Read UTF-16LE string data (2 bytes per character)
    uint8_t utf16_data[62]; // Max 31 characters * 2 bytes
    for (uint8_t i = 0; i < hub_settings->product_name_len * 2; i++) {
        status = usb2422_read_register(dev, USB2422_REG_PRDSTR_START + i, &utf16_data[i]);
        if (status != USB2422_OK) return status;
    }

    // Convert UTF-16LE to ASCII (simple conversion, ignoring high byte)
    for (uint8_t i = 0; i < hub_settings->product_name_len && i < 31; i++) {
        (hub_settings->product_name)[i] = (char)utf16_data[i * 2]; // Take LSB only
    }
    (hub_settings->product_name)[hub_settings->product_name_len] = '\0';

    return USB2422_OK;
}

usb2422_status_t usb2422_set_product_name(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, char product_name[32]) {
    if (!dev || !hub_settings || !product_name) return USB2422_ERR_NULL;

    uint8_t length = 0;
    while (product_name[length] != '\0' && length < 31) {
        length++;
    }

    // Set string length first
    usb2422_status_t status = usb2422_set_product_string_length(dev, hub_settings, length);
    if (status != USB2422_OK) return status;

    // Write UTF-16LE string data (ASCII to UTF-16LE conversion)
    for (uint8_t i = 0; i < length; i++) {
        // Write LSB (ASCII character)
        status = usb2422_write_register(dev, USB2422_REG_PRDSTR_START + (i * 2), (uint8_t)product_name[i]);
        if (status != USB2422_OK) return status;
        hub_settings->product_name[i] = product_name[i];

        // Write MSB (0 for ASCII)
        status = usb2422_write_register(dev, USB2422_REG_PRDSTR_START + (i * 2) + 1, 0x00);
        if (status != USB2422_OK) return status;
    }

    return USB2422_OK;
}

usb2422_status_t usb2422_get_serial_number(usb2422_t *dev, usb2422_hub_settings_t *hub_settings) {
    if (!dev || !hub_settings) return USB2422_ERR_NULL;

    usb2422_status_t status = usb2422_get_serial_string_length(dev, hub_settings);
    if (status != USB2422_OK) return status;

    if (hub_settings->serial_number_len == 0) {
        (hub_settings->serial_number)[0] = '\0';
        return USB2422_OK;
    }

    // Read UTF-16LE string data (2 bytes per character)
    uint8_t utf16_data[62]; // Max 31 characters * 2 bytes
    for (uint8_t i = 0; i < hub_settings->serial_number_len * 2; i++) {
        status = usb2422_read_register(dev, USB2422_REG_SERSTR_START + i, &utf16_data[i]);
        if (status != USB2422_OK) return status;
    }

    // Convert UTF-16LE to ASCII (simple conversion, ignoring high byte)
    for (uint8_t i = 0; i < hub_settings->serial_number_len && i < 31; i++) {
        (hub_settings->serial_number)[i] = (char)utf16_data[i * 2]; // Take LSB only
    }
    (hub_settings->serial_number)[hub_settings->serial_number_len] = '\0';

    return USB2422_OK;
}

usb2422_status_t usb2422_set_serial_number(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, char serial_number[32]) {
    if (!dev || !hub_settings || !serial_number) return USB2422_ERR_NULL;

    uint8_t length = 0;
    while (serial_number[length] != '\0' && length < 31) {
        length++;
    }

    // Set string length first
    usb2422_status_t status = usb2422_set_serial_string_length(dev, hub_settings, length);
    if (status != USB2422_OK) return status;

    // Write UTF-16LE string data (ASCII to UTF-16LE conversion)
    for (uint8_t i = 0; i < length; i++) {
        // Write LSB (ASCII character)
        status = usb2422_write_register(dev, USB2422_REG_SERSTR_START + (i * 2), (uint8_t)serial_number[i]);
        if (status != USB2422_OK) return status;
        hub_settings->serial_number[i] = serial_number[i];

        // Write MSB (0 for ASCII)
        status = usb2422_write_register(dev, USB2422_REG_SERSTR_START + (i * 2) + 1, 0x00);
        if (status != USB2422_OK) return status;
    }

    return USB2422_OK;
}