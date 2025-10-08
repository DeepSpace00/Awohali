//
// Created by deepspace on 10/1/25.
//

#include "main.h"
#include "stdbool.h"
#include "usb2422.h"
#include "usbd_cdc_if.h"

// Set string descriptors
char manufacturer[] = "DeepSpace Engineering";
char product[] = "Awohali";
char serial[] = "0001";

/**
 * @brief Configure the USB2422 for basic USB hub enumeration
 * @param dev Pointer to USB2422 device instance
 * @return USB2422_OK on success, error code otherwise
 */
usb2422_status_t configure_usb2422_for_enumeration(usb2422_t *dev) {
    usb_debug_print("Configuring USB2422 for enumeration...\r\n");

    // 1. Configure USB descriptor values
    usb_debug_print("Setting USB descriptor values...\r\n");

    usb2422_status_t status = usb2422_set_vendor_id(dev, 0x0424);  // Microchip VID
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set vendor ID: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_product_id(dev, 0x2422);  // USB2422 PID
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set product ID: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_device_id(dev, 0x0100);  // Device revision v1.00
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set device ID: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    // 2. Configure basic hub operation registers
    usb_debug_print("\r\nConfiguring hub operation settings...\r\n");

    status = usb2422_get_config_registers(dev, &config_registers);
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to read config registers: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    // Configure for self-powered operation (typical for desktop hubs)
    status = usb2422_set_self_bus_pwr(dev, &config_registers, true);  // Self-powered
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set self-powered mode: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_port_pwr(dev, &config_registers, false);  // Ganged power switching
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set port power mode: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_current_sns(dev, &config_registers, USB2422_CURRENT_SNS_GANGED);
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set current sensing: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_mtt_enable(dev, &config_registers, true);  // Multi-TT enabled
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to enable Multi-TT: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_hs_disable(dev, &config_registers, true);  // High-speed disabled
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to configure high-speed: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_oc_timer(dev, &config_registers, USB2422_OC_TIMER_8MS);
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set overcurrent timer: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_compound(dev, &config_registers, true);  // hub is part of a compound device
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set compound mode: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_dynamic(dev, &config_registers, false);  // No dynamic power switching
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set dynamic power: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_prtmap_en(dev, &config_registers, false);// Standard mode
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set port remapping: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    // 3. Configure power characteristics
    usb_debug_print("\r\nSetting power characteristics...\r\n");

    status = usb2422_set_max_power_self_powered(dev, &power_settings, 2);  // 2mA for self-powered
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set max power self-powered: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_max_power_bus_powered(dev, &power_settings, 100);  // 100mA for bus-powered
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set max power bus-powered: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_hub_controller_max_current_self_powered(dev, &power_settings, 2);
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set hub controller max current self-powered: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_hub_controller_max_current_bus_powered(dev, &power_settings, 100);
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set hub controller max current bus-powered: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_power_on_time(dev, &power_settings, 100);  // 100ms power-on time
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set power-on time: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    // 4. Configure port settings (enable both ports)
    usb_debug_print("\r\nConfiguring port settings...");

    status = usb2422_set_non_removable_device(dev, &port_settings, true, true);  // Both ports non-removable
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set non-removable device: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_port_disable_self_powered(dev, &port_settings, false, false);  // Enable self-power for both ports
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set port disable self-powered: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_port_disable_bus_powered(dev, &port_settings, false, false);  // Enable bus-power for both ports
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set port disable bus-powered: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    // 5. Optional: Configure string descriptors
    usb_debug_print("\r\nSetting up string descriptors...");

    status = usb2422_set_string_en(dev, &config_registers, true);  // Enable string descriptors
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to enable string descriptors: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_language_id(dev, &hub_settings, 0x0409);  // English (US)
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set language ID: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_manufacturer_name(dev, &hub_settings, manufacturer);
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set manufacturer name: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_product_name(dev, &hub_settings, product);
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set product name: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_serial_number(dev, &hub_settings, serial);
    if (status != USB2422_OK) {
        usb_debug_print("\r\nFailed to set serial number: ");
        usb_debug_print(usb2422_stat_error(status));
        return status;
    }

    usb_debug_print("\r\nUSB2422 configuration completed successfully!\r\n");
    return USB2422_OK;
}

/**
 * @brief Print current hub configuration for verification
 * @param dev Pointer to USB2422 device instance
 */
void print_hub_configuration(usb2422_t *dev) {
    usb2422_hub_settings_t current_hub_settings = {0};
    usb2422_power_settings_t current_power_settings = {0};

    char debug_buffer[512];

    usb_debug_print("\r\n=== Current Hub Configuration ===\r\n");

    // Read and display USB descriptors
    usb2422_status_t status = usb2422_get_vendor_id(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer), "\r\nVendor ID: 0x%02x", current_hub_settings.vendor_id);
        usb_debug_print(debug_buffer);
    }

    status = usb2422_get_product_id(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer), "\r\nProduct ID: 0x%02x", current_hub_settings.product_id);
        usb_debug_print(debug_buffer);
    }

    status = usb2422_get_device_id(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer), "\r\nDevice ID: 0x%02x", current_hub_settings.device_id);
        usb_debug_print(debug_buffer);
    }

    status = usb2422_get_language_id(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer), "\r\nLanguage ID: 0x%02x", current_hub_settings.language_id);
        usb_debug_print(debug_buffer);
    }

    // Read and display power settings
    status = usb2422_get_max_power_self_powered(dev, &current_power_settings);
    if (status == USB2422_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer), "\r\nMax Power Self-Powered: %d mA", current_power_settings.max_curr_self_powered);
        usb_debug_print(debug_buffer);
    }

    status = usb2422_get_power_on_time(dev, &current_power_settings);
    if (status == USB2422_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer), "\r\nPower-On Time: %d ms", current_power_settings.power_on_time);
        usb_debug_print(debug_buffer);
    }

    // Read and display string descriptors
    status = usb2422_get_manufacturer_name(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer), "\r\nManufacturer: ");
        usb_debug_print(current_hub_settings.manufacturer_name);
    }

    status = usb2422_get_product_name(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer), "\r\nProduct: ");
        usb_debug_print(current_hub_settings.product_name);
    }

    status = usb2422_get_serial_number(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        snprintf(debug_buffer, sizeof(debug_buffer), "\r\nSerial Number: ");
        usb_debug_print(current_hub_settings.serial_number);
    }

    usb_debug_print("\r\n=====================================\r\n");
}