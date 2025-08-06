/**
* @file main.cpp
 * @brief Arduino test program for USB2422 USB hub controller driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-06
 *
 * This test program demonstrates comprehensive functionality of the USB2422 driver
 * including initialization, configuration, and status monitoring.
 */

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include "usb2422.h"
#include "usb2422_platform.h"
}

// Global USB2422 instance
usb2422_t usb_hub;
usb2422_hub_settings_t hub_settings = {0};
usb2422_power_settings_t power_settings = {0};
usb2422_downstream_port_settings_t port_settings = {false};
usb2422_cfg_regs_t config_registers = {0};

/**
 * @brief Configure the USB2422 for basic USB hub enumeration
 * @param dev Pointer to USB2422 device instance
 * @return USB2422_OK on success, error code otherwise
 */
usb2422_status_t configure_usb2422_for_enumeration(usb2422_t *dev) {
    Serial.println("Configuring USB2422 for enumeration...");

    // 1. Configure USB descriptor values
    Serial.println("Setting USB descriptor values...");

    usb2422_status_t status = usb2422_set_vendor_id(dev, 0x0424);  // Microchip VID
    if (status != USB2422_OK) {
        Serial.print("Failed to set vendor ID: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_product_id(dev, 0x2422);  // USB2422 PID
    if (status != USB2422_OK) {
        Serial.print("Failed to set product ID: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_device_id(dev, 0x0100);  // Device revision v1.00
    if (status != USB2422_OK) {
        Serial.print("Failed to set device ID: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    // 2. Configure basic hub operation registers
    Serial.println("Configuring hub operation settings...");

    status = usb2422_get_config_registers(dev, &config_registers);
    if (status != USB2422_OK) {
        Serial.print("Failed to read config registers: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    // Configure for self-powered operation (typical for desktop hubs)
    status = usb2422_set_self_bus_pwr(dev, &config_registers, true);  // Self-powered
    if (status != USB2422_OK) {
        Serial.print("Failed to set self-powered mode: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_port_pwr(dev, &config_registers, false);  // Ganged power switching
    if (status != USB2422_OK) {
        Serial.print("Failed to set port power mode: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_current_sns(dev, &config_registers, USB2422_CURRENT_SNS_GANGED);
    if (status != USB2422_OK) {
        Serial.print("Failed to set current sensing: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_mtt_enable(dev, &config_registers, true);  // Multi-TT enabled
    if (status != USB2422_OK) {
        Serial.print("Failed to enable Multi-TT: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_hs_disable(dev, &config_registers, true);  // High-speed disabled
    if (status != USB2422_OK) {
        Serial.print("Failed to configure high-speed: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_oc_timer(dev, &config_registers, USB2422_OC_TIMER_8MS);
    if (status != USB2422_OK) {
        Serial.print("Failed to set overcurrent timer: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_compound(dev, &config_registers, true);  // hub is part of a compound device
    if (status != USB2422_OK) {
        Serial.print("Failed to set compound mode: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_dynamic(dev, &config_registers, false);  // No dynamic power switching
    if (status != USB2422_OK) {
        Serial.print("Failed to set dynamic power: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    // 3. Configure power characteristics
    Serial.println("Setting power characteristics...");

    status = usb2422_set_max_power_self_powered(dev, &power_settings, 2);  // 2mA for self-powered
    if (status != USB2422_OK) {
        Serial.print("Failed to set max power self-powered: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_max_power_bus_powered(dev, &power_settings, 100);  // 100mA for bus-powered
    if (status != USB2422_OK) {
        Serial.print("Failed to set max power bus-powered: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_hub_controller_max_current_self_powered(dev, &power_settings, 2);
    if (status != USB2422_OK) {
        Serial.print("Failed to set hub controller max current self-powered: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_hub_controller_max_current_bus_powered(dev, &power_settings, 100);
    if (status != USB2422_OK) {
        Serial.print("Failed to set hub controller max current bus-powered: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_power_on_time(dev, &power_settings, 100);  // 100ms power-on time
    if (status != USB2422_OK) {
        Serial.print("Failed to set power-on time: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    // 4. Configure port settings (enable both ports)
    Serial.println("Configuring port settings...");

    status = usb2422_set_non_removable_device(dev, &port_settings, true, true);  // Both ports non-removable
    if (status != USB2422_OK) {
        Serial.print("Failed to set non-removable device: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_port_disable_self_powered(dev, &port_settings, false, false);  // Enable self-power for both ports
    if (status != USB2422_OK) {
        Serial.print("Failed to set port disable self-powered: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_port_disable_bus_powered(dev, &port_settings, true, true);  // Disable bus-power for both ports
    if (status != USB2422_OK) {
        Serial.print("Failed to set port disable bus-powered: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    // 5. Optional: Configure string descriptors
    Serial.println("Setting up string descriptors...");

    status = usb2422_set_string_en(dev, &config_registers, true);  // Enable string descriptors
    if (status != USB2422_OK) {
        Serial.print("Failed to enable string descriptors: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_language_id(dev, &hub_settings, 0x0409);  // English (US)
    if (status != USB2422_OK) {
        Serial.print("Failed to set language ID: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    // Set string descriptors
    char manufacturer[] = "Awohali Hub";
    char product[] = "USB2422 Hub";
    char serial[] = "12345";

    status = usb2422_set_manufacturer_name(dev, &hub_settings, manufacturer);
    if (status != USB2422_OK) {
        Serial.print("Failed to set manufacturer name: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_product_name(dev, &hub_settings, product);
    if (status != USB2422_OK) {
        Serial.print("Failed to set product name: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    status = usb2422_set_serial_number(dev, &hub_settings, serial);
    if (status != USB2422_OK) {
        Serial.print("Failed to set serial number: ");
        Serial.println(usb2422_stat_error(status));
        return status;
    }

    Serial.println("USB2422 configuration completed successfully!");
    return USB2422_OK;
}

/**
 * @brief Print current hub configuration for verification
 * @param dev Pointer to USB2422 device instance
 */
void print_hub_configuration(usb2422_t *dev) {
    usb2422_hub_settings_t current_hub_settings = {0};
    usb2422_power_settings_t current_power_settings = {0};

    Serial.println("\n=== Current Hub Configuration ===");

    // Read and display USB descriptors
    usb2422_status_t status = usb2422_get_vendor_id(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        Serial.print("Vendor ID: 0x");
        Serial.println(current_hub_settings.vendor_id, HEX);
    }

    status = usb2422_get_product_id(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        Serial.print("Product ID: 0x");
        Serial.println(current_hub_settings.product_id, HEX);
    }

    status = usb2422_get_device_id(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        Serial.print("Device ID: 0x");
        Serial.println(current_hub_settings.device_id, HEX);
    }

    status = usb2422_get_language_id(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        Serial.print("Language ID: 0x");
        Serial.println(current_hub_settings.language_id, HEX);
    }

    // Read and display power settings
    status = usb2422_get_max_power_self_powered(dev, &current_power_settings);
    if (status == USB2422_OK) {
        Serial.print("Max Power Self-Powered: ");
        Serial.print(current_power_settings.max_curr_self_powered);
        Serial.println(" mA");
    }

    status = usb2422_get_power_on_time(dev, &current_power_settings);
    if (status == USB2422_OK) {
        Serial.print("Power-On Time: ");
        Serial.print(current_power_settings.power_on_time);
        Serial.println(" ms");
    }

    // Read and display string descriptors
    status = usb2422_get_manufacturer_name(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        Serial.print("Manufacturer: ");
        Serial.println(current_hub_settings.manufacturer_name);
    }

    status = usb2422_get_product_name(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        Serial.print("Product: ");
        Serial.println(current_hub_settings.product_name);
    }

    status = usb2422_get_serial_number(dev, &current_hub_settings);
    if (status == USB2422_OK) {
        Serial.print("Serial Number: ");
        Serial.println(current_hub_settings.serial_number);
    }

    Serial.println("=====================================\n");
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Wait for serial port to connect (needed for native USB boards)
    while (!Serial) {
        delay(10);
    }

    delay(100);  // Allow everything to settle

    Serial.println("USB2422 Hub Configuration Example");
    Serial.println("==================================");

    // Define the IO interface for the USB2422 driver
    constexpr usb2422_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .delay_ms = platform_delay_ms
    };

    // Initialize the USB2422 driver
    Serial.println("Initializing USB2422 driver...");
    usb2422_status_t status = usb2422_init(&usb_hub, USB2422_SMBUS_ADDRESS, io);

    if (status != USB2422_OK) {
        Serial.print("Failed to initialize USB2422: ");
        Serial.println(usb2422_stat_error(status));
        Serial.println("Check I2C connections and try again.");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("USB2422 driver initialized successfully!");

    // Configure the hub for USB enumeration
    status = configure_usb2422_for_enumeration(&usb_hub);

    if (status != USB2422_OK) {
        Serial.print("Failed to configure USB2422: ");
        Serial.println(usb2422_stat_error(status));
        Serial.println("Configuration failed. Check connections and try again.");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("\nUSB2422 configuration completed successfully!");
    Serial.println("The hub should now enumerate when connected to a USB host.");

    // Print the current configuration for verification
    print_hub_configuration(&usb_hub);

    Serial.println("Setup complete. Connect the hub to a USB host to test enumeration.");
    Serial.println("Monitor will show periodic status updates...\n");
}

void loop() {
    // Periodic status check every 10 seconds
    static unsigned long last_status_check = 0;
    unsigned long current_time = millis();

    if (current_time - last_status_check >= 10000) {
        last_status_check = current_time;

        Serial.println("Hub is configured and ready for USB enumeration.");
        Serial.print("Runtime: ");
        Serial.print(current_time / 1000);
        Serial.println(" seconds");

        // You could add additional monitoring here, such as:
        // - Reading hub status registers
        // - Checking for configuration changes
        // - Monitoring power consumption
    }

    // Small delay to prevent overwhelming the serial output
    delay(100);
}