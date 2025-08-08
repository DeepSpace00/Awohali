/**
 * @file usb2422.h
 * @brief USB2422 USB Hub Controller Driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-06
 *
 * This driver supports the USB2422 USB hub controller from Microchip Technology
 */

#ifndef USB2422_H
#define USB2422_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB2422_SMBUS_ADDRESS 0x2C      // Default SMBus/I2C address

/**
 * @brief USB2422 USB hub driver return status codes.
 */
typedef enum {
    USB2422_OK = 0,
    USB2422_ERR_I2C = -1,
    USB2422_ERR_TIMEOUT = -2,
    USB2422_ERR_NULL = -3,
    USB2422_ERR_INVALID_ARG = -4,
} usb2422_status_t;

typedef enum {
    USB2422_PORT_PWR,       ///< Enables power switching on all ports simultaneously, or port power is individually switched on/off
    USB2422_CURRENT_SNS,    ///< Selects current sensing on (2-bits)
    USB2422_EOP_DISABLE,    ///< Disables EOP generation of EOF1 when in FS mode
    USB2422_MTT_ENABLE,     ///< Enables one transaction translator per port operation
    USB2422_HS_DISABLE,     ///< Disables the capability to attach as either an HS/FS device and forces FS only
    USB2422_SELF_BUS_PWR,   ///< Selects between Self- and Bus-Powered operation
    USB2422_COMPOUND,       ///< Allows the OEM to indicate that the hub is part of a compound device
    USB2422_OC_TIMER,       ///< Over current timer delay (2-bit)
    USB2422_DYNAMIC,        ///< Changes the ability of the hub to automatically change from Self- to Bus-Powered if local power source is removed
    USB2422_STRING_EN,      ///< Enables string descriptor support
    USB2422_PRTMAP_EN       ///< Selects the method used by the hub to assign port numbers and disable ports
} usb2422_config_parameters_t;

typedef enum {
    USB2422_CURRENT_SNS_GANGED = 0x00,
    USB2422_CURRENT_SNS_INDIVIDUAL = 0x01,
    USB2422_CURRENT_SNS_OC_NOT_SUP = 0x02,
} usb2422_config_current_sns_value_t;

typedef enum {
    USB2422_OC_TIMER_100US = 0x00,
    USB2422_OC_TIMER_4MS = 0x01,
    USB2422_OC_TIMER_8MS = 0x02,
    USB2422_OC_TIMER_16MS = 0x03
} usb2422_config_oc_timer_value_t;

typedef enum {
    USB2422_BOOST_PORT_1_NORMAL = 0x00,     ///< Normal electrical drive strength
    USB2422_BOOST_PORT_1_4PCT = 0x01,       ///< Boost +4%
    USB2422_BOOST_PORT_1_8PCT = 0x02,       ///< Boost +8%
    USB2422_BOOST_PORT_1_12PCT = 0x03,      ///< Boost +12%
} usb2422_boost_iout_1;

typedef enum {
    USB2422_BOOST_PORT_2_NORMAL = 0x00,     ///< Normal electrical drive strength
    USB2422_BOOST_PORT_2_4PCT = 0x01,       ///< Boost +4%
    USB2422_BOOST_PORT_2_8PCT = 0x02,       ///< Boost +8%
    USB2422_BOOST_PORT_2_12PCT = 0x03,      ///< Boost +12%
} usb2422_boost_iout_2;

typedef enum {
    USB2422_PHY_PORT_1_DISABLED = 0x00,     ///< Physical Port 1 is disabled
    USB2422_PHY_PORT_1_LOGICAL_1 = 0x01,    ///< Physical Port 1 is mapped to Logical Port 1
    USB2422_PHY_PORT_1_LOGICAL_2 = 0x02,    ///< Physical Port 1 is mapped to Logical Port 2
} usb2422_port_1_remap_t;

typedef enum {
    USB2422_PHY_PORT_2_DISABLED = 0x00,     ///< Physical Port 2 is disabled
    USB2422_PHY_PORT_2_LOGICAL_1 = 0x01,    ///< Physical Port 2 is mapped to Logical Port 1
    USB2422_PHY_PORT_2_LOGICAL_2 = 0x02,    ///< Physical Port 2 is mapped to Logical Port 2
} usb2422_port_2_remap_t;

/**
 * @brief Platform interface abstraction for USB2422 driver.
 *
 * The user must provide these functions to enable hardware-specific I2C and timing.
 */
typedef struct {
    int (*i2c_write)(uint8_t dev_addr, const uint8_t *data, uint16_t len);
    int (*i2c_read)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);
    void (*delay_ms)(uint32_t ms);
} usb2422_interface_t;

/**
 * @brief USB2422 driver instance.
 */
typedef struct {
    uint8_t i2c_address;
    usb2422_interface_t io;
    bool initialized;
} usb2422_t;

/**
 * @brief USB2422 configuration settings
 */
typedef struct {
    uint16_t vendor_id;             ///< Vendor ID
    uint16_t product_id;            ///< Product ID
    uint16_t device_id;             ///< Device ID
    bool port_remapping_enabled;    ///< Enable port remapping
    bool string_enabled;            ///< Enables string descriptor support
    uint16_t language_id;           ///< Language ID
    char manufacturer_name[32];     ///< Manufacturer name
    char product_name[32];          ///< Product name
    char serial_number[32];         ///< Serial number
    uint8_t manufacturer_name_len;  ///< Manufacturer name length
    uint8_t product_name_len;       ///< Product name length
    uint8_t serial_number_len;      ///< Serial number length
    bool smbus_powered_down;        ///< SMBus interface power down
    bool reset;                     ///< Resets the SMBus interface and memory
    bool usb_write_protect;         ///< Signal a USB attach event
} usb2422_hub_settings_t;

typedef struct {
    uint8_t max_curr_self_powered;      ///< How much current consumed in self-powered mode (mA)
    uint8_t max_curr_bus_powered;       ///< How much current consumed in bus-powered mode (mA)
    uint8_t hub_max_curr_self_powered;  ///< How much current consumed in self-powered mode (mA)
    uint8_t hub_max_curr_bus_powered;   ///< How much current consumed in bus-powered mode (mA)
    uint8_t power_on_time;              ///< Time until power is good on the port (ms)
} usb2422_power_settings_t;

typedef struct {
    bool port_1_non_removable;              ///< Set port 1 as non-removable
    bool port_2_non_removable;              ///< Set port 2 as non-removable
    bool port_1_disable_self_powered;       ///< Disable port 1 when self-powered
    bool port_2_disable_self_powered;       ///< Disable port 2 when self-powered
    bool port_1_disable_bus_powered;        ///< Disable port 1 when bus-powered
    bool port_2_disable_bus_powered;        ///< Disable port 2 when bus-powered
    bool port_1_enable_batt_charging;       ///< Enable port 1 battery charging
    bool port_2_enable_batt_charging;       ///< Enable port 2 battery charging
    usb2422_boost_iout_1 port_1_boost;      ///< USB electrical signaling drive strength
    usb2422_boost_iout_2 port_2_boost;      ///< USB electrical signaling drive strength
    bool port_1_swap_dp_dm;                 ///< Swap DP/DM on port 1
    bool port_2_swap_dp_dm;                 ///< Swap DP/DM on port 2
    usb2422_port_1_remap_t port_1_remap;    ///< Remap port 1
    usb2422_port_2_remap_t port_2_remap;    ///< Remap port 2
} usb2422_downstream_port_settings_t;

/**
 * @brief CFG1 Register (0x06) bits
 */
typedef struct {
    uint8_t self_bus_pwr    : 1;    ///< Bit 7
    uint8_t reserved1       : 1;    ///< Bit 6 (reserved)
    uint8_t hs_disable      : 1;    ///< Bit 5
    uint8_t mtt_enable      : 1;    ///< Bit 4
    uint8_t eop_disable     : 1;    ///< Bit 3
    uint8_t current_sns     : 2;    ///< Bits 2-1
    uint8_t port_pwr        : 1;    ///< Bit 0
} usb2422_cfg1_t;

/**
 * @brief CFG2 Register (0x07) bits
 */
typedef struct {
    uint8_t dynamic     : 1;    ///< Bit 7
    uint8_t reserved2   : 1;    ///< Bit 6
    uint8_t oc_timer    : 2;    ///< Bits 5-4
    uint8_t compound    : 1;    ///< Bit 3
    uint8_t reserved3   : 3;    ///< Bits 2-0
} usb2422_cfg2_t;

/**
 * @brief CFG3 Register (0x08) bits
 */
typedef struct {
    uint8_t reserved4   : 4;    ///< Bits 7-4
    uint8_t prtmap_en   : 1;    ///< Bit 3
    uint8_t reserved5   : 2;    ///< Bits 2-1
    uint8_t string_en   : 1;    ///< Bit 0
} usb2422_cfg3_t;

/**
 * @brief Combined CFG registers structure
 */
typedef struct {
    usb2422_cfg1_t cfg1;
    usb2422_cfg2_t cfg2;
    usb2422_cfg3_t cfg3;
} usb2422_cfg_regs_t;

///< @section Function Prototypes

/**
 * @brief Human-readable description of an USB2422 status code.
 *
 * @param status The status code to translate.
 * @return A string describing the error.
 */
const char* usb2422_stat_error(usb2422_status_t status);

usb2422_status_t usb2422_init(usb2422_t *dev, uint8_t address, usb2422_interface_t io);

usb2422_status_t usb2422_get_vendor_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_vendor_id(usb2422_t *dev, uint16_t vendor_id);

usb2422_status_t usb2422_get_product_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_product_id(usb2422_t *dev, uint16_t product_id);

usb2422_status_t usb2422_get_device_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_device_id(usb2422_t *dev, uint16_t device_id);

usb2422_status_t usb2422_get_config_registers(usb2422_t *dev, usb2422_cfg_regs_t *regs);

usb2422_status_t usb2422_set_port_pwr(usb2422_t *dev, usb2422_cfg_regs_t *regs, bool value);

usb2422_status_t usb2422_set_current_sns(usb2422_t *dev, usb2422_cfg_regs_t *regs, usb2422_config_current_sns_value_t value);

usb2422_status_t usb2422_set_eop_disable(usb2422_t *dev, usb2422_cfg_regs_t *regs, bool value);

usb2422_status_t usb2422_set_mtt_enable(usb2422_t *dev, usb2422_cfg_regs_t *regs, bool value);

usb2422_status_t usb2422_set_hs_disable(usb2422_t *dev, usb2422_cfg_regs_t *regs, bool value);

usb2422_status_t usb2422_set_self_bus_pwr(usb2422_t *dev, usb2422_cfg_regs_t *regs, bool value);

usb2422_status_t usb2422_set_compound(usb2422_t *dev, usb2422_cfg_regs_t *regs, bool value);

usb2422_status_t usb2422_set_oc_timer(usb2422_t *dev, usb2422_cfg_regs_t *regs, usb2422_config_oc_timer_value_t value);

usb2422_status_t usb2422_set_dynamic(usb2422_t *dev, usb2422_cfg_regs_t *regs, bool value);

usb2422_status_t usb2422_set_string_en(usb2422_t *dev, usb2422_cfg_regs_t *regs, bool value);

usb2422_status_t usb2422_set_prtmap_en(usb2422_t *dev, usb2422_cfg_regs_t *regs, bool value);

usb2422_status_t usb2422_get_non_removable_device(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings);
usb2422_status_t usb2422_set_non_removable_device(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings, bool port_1_non_removable, bool port_2_non_removable);

usb2422_status_t usb2422_get_port_disable_self_powered(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings);
usb2422_status_t usb2422_set_port_disable_self_powered(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings, bool port_1_disable_self_powered, bool port_2_disable_self_powered);

usb2422_status_t usb2422_get_port_disable_bus_powered(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings);
usb2422_status_t usb2422_set_port_disable_bus_powered(usb2422_t *dev, usb2422_downstream_port_settings_t *downstream_port_settings, bool port_1_disable_bus_powered, bool port_2_disable_bus_powered);

usb2422_status_t usb2422_get_max_power_self_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings);
usb2422_status_t usb2422_set_max_power_self_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings, uint8_t max_curr_self_powered);

usb2422_status_t usb2422_get_max_power_bus_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings);
usb2422_status_t usb2422_set_max_power_bus_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings, uint8_t max_curr_bus_powered);

usb2422_status_t usb2422_get_hub_controller_max_current_self_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings);
usb2422_status_t usb2422_set_hub_controller_max_current_self_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings, uint8_t hub_max_curr_self_powered);

usb2422_status_t usb2422_get_hub_controller_max_current_bus_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings);
usb2422_status_t usb2422_set_hub_controller_max_current_bus_powered(usb2422_t *dev, usb2422_power_settings_t *power_settings, uint8_t hub_max_curr_bus_powered);

usb2422_status_t usb2422_get_power_on_time(usb2422_t *dev, usb2422_power_settings_t *power_settings);
usb2422_status_t usb2422_set_power_on_time(usb2422_t *dev, usb2422_power_settings_t *power_settings, uint8_t power_on_time);

usb2422_status_t usb2422_get_language_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_language_id(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, uint16_t language_id);

usb2422_status_t usb2422_get_manufacturer_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_manufacturer_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, uint8_t manufacturer_name_len);

usb2422_status_t usb2422_get_product_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_product_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, uint8_t product_name_len);

usb2422_status_t usb2422_get_serial_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_serial_string_length(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, uint8_t serial_number_len);

usb2422_status_t usb2422_get_manufacturer_name(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_manufacturer_name(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, char manufacturer_name[32]);

usb2422_status_t usb2422_get_product_name(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_product_name(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, char product_name[32]);

usb2422_status_t usb2422_get_serial_number(usb2422_t *dev, usb2422_hub_settings_t *hub_settings);
usb2422_status_t usb2422_set_serial_number(usb2422_t *dev, usb2422_hub_settings_t *hub_settings, char serial_number[32]);

#ifdef __cplusplus
}
#endif

#endif //USB2422_H