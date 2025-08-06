# USB2422 Arduino Library

This library provides a complete driver for the Microchip USB2422 USB Hub Controller, designed for Arduino and other embedded platforms.

## Features

- Complete register access for all USB2422 configuration registers
- Platform-agnostic design with Arduino platform implementation
- USB descriptor configuration (Vendor ID, Product ID, Device ID)
- Power management settings (self/bus powered modes)
- Port configuration and control
- String descriptor support (UTF-16LE)
- Comprehensive error handling

## Hardware Setup

### Connections
- **SDA** → Arduino SDA pin (A4 on Uno, D21 on ESP32)
- **SCL** → Arduino SCL pin (A5 on Uno, D22 on ESP32)  
- **VDD** → 3.3V
- **GND** → GND
- **Pull-up resistors** (4.7kΩ) on SDA and SCL lines

### I2C Address
The USB2422 uses I2C address `0x2C` (44 decimal) by default.

## Library Structure

```
usb2422_arduino_library/
├── src/
│   ├── usb2422.h                    # Main driver header
│   ├── usb2422.c                    # Main driver implementation
│   ├── usb2422_platform.h          # Platform abstraction header
│   └── usb2422_platform_arduino.cpp # Arduino platform implementation
└── examples/
    └── usb2422_configuration_example/
        └── usb2422_configuration_example.ino
```

## Installation

1. Copy the library files to your Arduino libraries folder:
   - `Arduino/libraries/USB2422/src/`

2. Include the library in your sketch:
```cpp
extern "C" {
#include "usb2422.h" 
#include "usb2422_platform.h"
}
```

## Basic Usage

### 1. Initialize the Driver
```cpp
#include <Wire.h>
extern "C" {
#include "usb2422.h"
#include "usb2422_platform.h"
}

usb2422_t usb_hub;

void setup() {
    Wire.begin();
    
    usb2422_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read, 
        .delay_ms = platform_delay_ms
    };
    
    usb2422_status_t status = usb2422_init(&usb_hub, USB2422_SMBUS_ADDRESS, io);
    if (status != USB2422_OK) {
        Serial.println("Failed to initialize USB2422");
        return;
    }
}
```

### 2. Configure USB Descriptors
```cpp
// Set USB identifiers
usb2422_set_vendor_id(&usb_hub, 0x0424);   // Microchip VID
usb2422_set_product_id(&usb_hub, 0x2422);  // USB2422 PID
usb2422_set_device_id(&usb_hub, 0x0100);   // v1.00
```

### 3. Configure Hub Operation
```cpp
usb2422_cfg_regs_t config_regs;
usb2422_get_config_registers(&usb_hub, &config_regs);

// Configure for self-powered operation
usb2422_set_self_bus_pwr(&usb_hub, &config_regs, true);
usb2422_set_mtt_enable(&usb_hub, &config_regs, true);
usb2422_set_hs_disable(&usb_hub, &config_regs, false);
```

### 4. Set Power Characteristics
```cpp
usb2422_power_settings_t power_settings;

usb2422_set_max_power_self_powered(&usb_hub, &power_settings, 2);    // 2mA
usb2422_set_max_power_bus_powered(&usb_hub, &power_settings, 100);   // 100mA
usb2422_set_power_on_time(&usb_hub, &power_settings, 100);           // 100ms
```

### 5. Configure String Descriptors (Optional)
```cpp
usb2422_hub_settings_t hub_settings;

usb2422_set_string_en(&usb_hub, &config_regs, true);
usb2422_set_language_id(&usb_hub, &hub_settings, 0x0409);  // English (US)

char manufacturer[] = "My Company";
char product[] = "USB Hub"; 
char serial[] = "12345";

usb2422_set_manufacturer_name(&usb_hub, &hub_settings, manufacturer);
usb2422_set_product_name(&usb_hub, &hub_settings, product);
usb2422_set_serial_number(&usb_hub, &hub_settings, serial);
```

## Configuration Options

### Power Modes
- **Self-Powered**: Hub has its own power supply (`usb2422_set_self_bus_pwr(true)`)
- **Bus-Powered**: Hub draws power from USB host (`usb2422_set_self_bus_pwr(false)`)

### Current Sensing
- `USB2422_CURRENT_SNS_GANGED`: All ports together
- `USB2422_CURRENT_SNS_INDIVIDUAL`: Port-by-port sensing
- `USB2422_CURRENT_SNS_OC_NOT_SUP`: No overcurrent sensing

### Hub Features
- **Multi-TT**: Enable with `usb2422_set_mtt_enable(true)` for better performance
- **High-Speed**: Enable with `usb2422_set_hs_disable(false)` for USB 2.0 speeds
- **String Descriptors**: Enable with `usb2422_set_string_en(true)`

## Error Handling

All functions return `usb2422_status_t`:
- `USB2422_OK`: Success
- `USB2422_ERR_I2C`: I2C communication failed  
- `USB2422_ERR_NULL`: Null pointer passed
- `USB2422_ERR_INVALID_ARG`: Invalid argument
- `USB2422_ERR_TIMEOUT`: Operation timeout

Use `usb2422_stat_error()` to get human-readable error messages:

```cpp
usb2422_status_t status = usb2422_init(&usb_hub, USB2422_SMBUS_ADDRESS, io);
if (status != USB2422_OK) {
    Serial.print("Error: ");
    Serial.println(usb2422_stat_error(status));
}
```

## Example Output

```
USB2422 Hub Configuration Example
==================================
Initializing USB2422 driver...
USB2422 driver initialized successfully!
Configuring USB2422 for enumeration...
Setting USB descriptor values...
Configuring hub operation settings...
Setting power characteristics...
Configuring port settings...
Setting up string descriptors...
USB2422 configuration completed successfully!

=== Current Hub Configuration ===
Vendor ID: 0x424
Product ID: 0x2422  
Device ID: 0x100
Language ID: 0x409
Max Power Self-Powered: 2 mA
Power-On Time: 100 ms
Manufacturer: Arduino Hub
Product: USB2422 Hub
Serial Number: 12345
=====================================

Setup complete. Connect the hub to a USB host to test enumeration.
```

## Troubleshooting

### Common Issues

1. **Initialization fails**: Check I2C connections and pull-up resistors
2. **Configuration errors**: Verify power supply is stable (3.3V)
3. **Hub doesn't enumerate**: Ensure all required settings are configured before USB connection

### Debug Tips

- Enable verbose serial output to monitor configuration steps
- Use an I2C scanner to verify the device is responding at address 0x2C
- Check that SDA/SCL have proper pull-up resistors (4.7kΩ recommended)
- Ensure stable 3.3V power supply to the USB2422

## Advanced Usage

For more advanced features like port remapping, boost control, and battery charging, refer to the complete function list in `usb2422.h`.

## License

This library follows the same licensing as your original driver framework.