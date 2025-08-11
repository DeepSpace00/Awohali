/**
 * @file main.cpp
 * @brief Arduino CLI for BQ25798 MPPT Battery Charger Driver
 * @author Generated for BQ25798 Driver
 * @date 2025-08-11
 *
 * Comprehensive command-line interface for all BQ25798 driver functions
 *
 * Usage:
 * - Connect BQ25798 to I2C pins (SDA/SCL)
 * - Open Serial Monitor at 115200 baud
 * - Use menu system to access all driver functions
 *
 * Commands are organized by category:
 * 1. Device Management
 * 2. Basic Settings
 * 3. Charge Control
 * 4. MPPT Control
 * 5. Temperature Control
 * 6. NTC Control
 * 7. Status Monitoring
 * 8. Flag Monitoring
 * 9. ADC Control
 * 0. Measurements
 */

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include "bq25798.h"
#include "bq25798_platform.h"
}

// Global driver instance
bq25798_t bq_device;
bool device_initialized = false;

// Function declarations
void init_device();
void print_main_menu();
void process_command(const String &cmd);

// Menu function declarations
void device_management_menu();
void basic_settings_menu();
void charge_control_menu();
void mppt_control_menu();
void temperature_control_menu();
void ntc_control_menu();
void status_monitoring_menu();
void flag_monitoring_menu();
void adc_control_menu();
void measurements_menu();

// Basic settings functions
void reset_settings();
void get_device_status();
void vsysmin_menu();
void charge_voltage_limit_menu();
void charge_current_limit_menu();
void input_voltage_limit_menu();
void input_current_limit_menu();
void vbat_lowv_menu();
void precharge_current_menu();
void termination_current_menu();
void cell_count_menu();
void deglitch_time_menu();
void recharge_threshold_menu();

// Charge control functions
void auto_battery_discharge_menu();
void force_battery_discharge_menu();
void charge_enable_menu();
void ico_enable_menu();
void force_ico_menu();
void hiz_mode_menu();
void termination_enable_menu();
void backup_mode_menu();
void vac_ovp_menu();
void watchdog_menu();
void hvdcp_menu();
void sfet_control_menu();
void acdrv_menu();
void power_management_menu();

// MPPT control functions
void voc_percentage_menu();
void voc_delay_menu();
void voc_rate_menu();
void mppt_enable_menu();

// Temperature control functions
void thermal_reg_menu();
void thermal_shutdown_menu();
void vbus_pulldown_menu();
void vac1_pulldown_menu();
void vac2_pulldown_menu();
void backup_acfet1_menu();

// NTC control functions
void jeita_high_voltage_menu();
void jeita_high_current_menu();
void jeita_low_current_menu();
void ts_cool_menu();
void ts_warm_menu();
void ts_hot_menu();
void ts_cold_menu();
void ts_ignore_menu();

// Status and flag monitoring functions
void get_all_charger_status();
void get_all_fault_status();
void individual_status_menu();
void get_all_charger_flags();
void get_all_fault_flags();
void individual_flags_menu();

// ADC control functions
void adc_enable_menu();
void adc_rate_menu();
void adc_resolution_menu();
void adc_averaging_menu();
void adc_channels_menu();

// Measurement functions
void get_all_measurements();
void individual_measurements_menu();
void continuous_monitoring();

// Helper function declarations
String wait_for_input();
int get_int_input(const char* prompt);
float get_float_input(const char* prompt);
bool get_bool_input(const char* prompt);
void print_status_result(bq25798_status_t status, const char* operation);

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Wire.begin();
    Wire.setClock(100000); // 100kHz I2C
    
    Serial.println("\n=== BQ25798 MPPT Battery Charger CLI ===");
    Serial.println("Driver by Madison Gleydura (DeepSpace00)");
    Serial.println("CLI Interface Ready\n");
    
    // Try to initialize device
    init_device();
    print_main_menu();
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        process_command(input);  // This should work now
    }
}

void init_device() {
    Serial.println("Initializing BQ25798...");
    
    constexpr bq25798_interface_t interface = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .delay_ms = platform_delay_ms
    };

    if (const bq25798_status_t status = bq25798_init(&bq_device, 0, interface); status == BQ25798_OK) {
        device_initialized = true;
        Serial.println("✓ BQ25798 initialized successfully!");
    } else {
        device_initialized = false;
        Serial.print("✗ Initialization failed: ");
        Serial.println(bq25798_stat_error(status));
    }
    Serial.println();
}

void print_main_menu() {
    Serial.println("=== MAIN MENU ===");
    Serial.println("1. Device Management");
    Serial.println("2. Basic Settings");
    Serial.println("3. Charge Control");
    Serial.println("4. MPPT Control");
    Serial.println("5. Temperature Control");
    Serial.println("6. NTC Control");
    Serial.println("7. Status Monitoring");
    Serial.println("8. Flag Monitoring");
    Serial.println("9. ADC Control");
    Serial.println("0. Measurements");
    Serial.println("h. Help (show this menu)");
    Serial.println("Enter selection:");
}

void process_command(const String &cmd) {
    if (!device_initialized && cmd != "1" && cmd != "h") {
        Serial.println("Device not initialized! Use menu 1 to initialize.");
        return;
    }
    
    if (cmd == "h") {
        print_main_menu();
    } else if (cmd == "1") {
        device_management_menu();
    } else if (cmd == "2") {
        basic_settings_menu();
    } else if (cmd == "3") {
        charge_control_menu();
    } else if (cmd == "4") {
        mppt_control_menu();
    } else if (cmd == "5") {
        temperature_control_menu();
    } else if (cmd == "6") {
        ntc_control_menu();
    } else if (cmd == "7") {
        status_monitoring_menu();
    } else if (cmd == "8") {
        flag_monitoring_menu();
    } else if (cmd == "9") {
        adc_control_menu();
    } else if (cmd == "0") {
        measurements_menu();
    } else {
        Serial.println("Invalid command. Type 'h' for help.");
    }
}

// Device Management Menu
void device_management_menu() {
    Serial.println("\n=== DEVICE MANAGEMENT ===");
    Serial.println("1. Initialize device");
    Serial.println("2. Reset all settings to default");
    Serial.println("3. Get device status");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        init_device();
    } else if (cmd == "2") {
        reset_settings();
    } else if (cmd == "3") {
        get_device_status();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// Basic Settings Menu
void basic_settings_menu() {
    Serial.println("\n=== BASIC SETTINGS ===");
    Serial.println("1. VSYSMIN (Minimum System Voltage)");
    Serial.println("2. Charge Voltage Limit");
    Serial.println("3. Charge Current Limit");
    Serial.println("4. Input Voltage Limit");
    Serial.println("5. Input Current Limit");
    Serial.println("6. VBAT Low Voltage Threshold");
    Serial.println("7. Precharge Current Limit");
    Serial.println("8. Termination Current");
    Serial.println("9. Cell Count");
    Serial.println("10. Deglitch Time");
    Serial.println("11. Recharge Threshold Offset");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        vsysmin_menu();
    } else if (cmd == "2") {
        charge_voltage_limit_menu();
    } else if (cmd == "3") {
        charge_current_limit_menu();
    } else if (cmd == "4") {
        input_voltage_limit_menu();
    } else if (cmd == "5") {
        input_current_limit_menu();
    } else if (cmd == "6") {
        vbat_lowv_menu();
    } else if (cmd == "7") {
        precharge_current_menu();
    } else if (cmd == "8") {
        termination_current_menu();
    } else if (cmd == "9") {
        cell_count_menu();
    } else if (cmd == "10") {
        deglitch_time_menu();
    } else if (cmd == "11") {
        recharge_threshold_menu();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// Charge Control Menu
void charge_control_menu() {
    Serial.println("\n=== CHARGE CONTROL ===");
    Serial.println("1. Auto Battery Discharge on OVP");
    Serial.println("2. Force Battery Discharge");
    Serial.println("3. Charge Enable");
    Serial.println("4. ICO Enable");
    Serial.println("5. Force ICO Enable");
    Serial.println("6. HIZ Mode");
    Serial.println("7. Termination Enable");
    Serial.println("8. Backup Mode");
    Serial.println("9. VAC OVP Settings");
    Serial.println("10. Watchdog Timer");
    Serial.println("11. HVDCP Settings");
    Serial.println("12. SFET Control");
    Serial.println("13. ACDRV Settings");
    Serial.println("14. Power Management");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        auto_battery_discharge_menu();
    } else if (cmd == "2") {
        force_battery_discharge_menu();
    } else if (cmd == "3") {
        charge_enable_menu();
    } else if (cmd == "4") {
        ico_enable_menu();
    } else if (cmd == "5") {
        force_ico_menu();
    } else if (cmd == "6") {
        hiz_mode_menu();
    } else if (cmd == "7") {
        termination_enable_menu();
    } else if (cmd == "8") {
        backup_mode_menu();
    } else if (cmd == "9") {
        vac_ovp_menu();
    } else if (cmd == "10") {
        watchdog_menu();
    } else if (cmd == "11") {
        hvdcp_menu();
    } else if (cmd == "12") {
        sfet_control_menu();
    } else if (cmd == "13") {
        acdrv_menu();
    } else if (cmd == "14") {
        power_management_menu();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// MPPT Control Menu
void mppt_control_menu() {
    Serial.println("\n=== MPPT CONTROL ===");
    Serial.println("1. VOC Percentage Setting");
    Serial.println("2. VOC Delay Setting");
    Serial.println("3. VOC Measurement Rate");
    Serial.println("4. MPPT Enable");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        voc_percentage_menu();
    } else if (cmd == "2") {
        voc_delay_menu();
    } else if (cmd == "3") {
        voc_rate_menu();
    } else if (cmd == "4") {
        mppt_enable_menu();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// Temperature Control Menu
void temperature_control_menu() {
    Serial.println("\n=== TEMPERATURE CONTROL ===");
    Serial.println("1. Thermal Regulation Threshold");
    Serial.println("2. Thermal Shutdown Threshold");
    Serial.println("3. VBUS Pulldown");
    Serial.println("4. VAC1 Pulldown");
    Serial.println("5. VAC2 Pulldown");
    Serial.println("6. Backup ACFET1");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        thermal_reg_menu();
    } else if (cmd == "2") {
        thermal_shutdown_menu();
    } else if (cmd == "3") {
        vbus_pulldown_menu();
    } else if (cmd == "4") {
        vac1_pulldown_menu();
    } else if (cmd == "5") {
        vac2_pulldown_menu();
    } else if (cmd == "6") {
        backup_acfet1_menu();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// NTC Control Menu
void ntc_control_menu() {
    Serial.println("\n=== NTC CONTROL ===");
    Serial.println("1. JEITA High Temp Voltage Setting");
    Serial.println("2. JEITA High Temp Current Setting");
    Serial.println("3. JEITA Low Temp Current Setting");
    Serial.println("4. TS Cool Setting");
    Serial.println("5. TS Warm Setting");
    Serial.println("6. TS Hot Setting");
    Serial.println("7. TS Cold Setting");
    Serial.println("8. TS Ignore");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        jeita_high_voltage_menu();
    } else if (cmd == "2") {
        jeita_high_current_menu();
    } else if (cmd == "3") {
        jeita_low_current_menu();
    } else if (cmd == "4") {
        ts_cool_menu();
    } else if (cmd == "5") {
        ts_warm_menu();
    } else if (cmd == "6") {
        ts_hot_menu();
    } else if (cmd == "7") {
        ts_cold_menu();
    } else if (cmd == "8") {
        ts_ignore_menu();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// Status Monitoring Menu
void status_monitoring_menu() {
    Serial.println("\n=== STATUS MONITORING ===");
    Serial.println("1. Get All Charger Status");
    Serial.println("2. Get All Fault Status");
    Serial.println("3. Individual Status Values");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        get_all_charger_status();
    } else if (cmd == "2") {
        get_all_fault_status();
    } else if (cmd == "3") {
        individual_status_menu();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// Flag Monitoring Menu
void flag_monitoring_menu() {
    Serial.println("\n=== FLAG MONITORING ===");
    Serial.println("1. Get All Charger Flags");
    Serial.println("2. Get All Fault Flags");
    Serial.println("3. Individual Flag Values");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        get_all_charger_flags();
    } else if (cmd == "2") {
        get_all_fault_flags();
    } else if (cmd == "3") {
        individual_flags_menu();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// ADC Control Menu
void adc_control_menu() {
    Serial.println("\n=== ADC CONTROL ===");
    Serial.println("1. ADC Enable/Disable");
    Serial.println("2. ADC Rate Control");
    Serial.println("3. ADC Resolution");
    Serial.println("4. ADC Averaging Control");
    Serial.println("5. Individual ADC Channel Control");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        adc_enable_menu();
    } else if (cmd == "2") {
        adc_rate_menu();
    } else if (cmd == "3") {
        adc_resolution_menu();
    } else if (cmd == "4") {
        adc_averaging_menu();
    } else if (cmd == "5") {
        adc_channels_menu();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// Measurements Menu
void measurements_menu() {
    Serial.println("\n=== MEASUREMENTS ===");
    Serial.println("1. Get All Measurements");
    Serial.println("2. Individual Measurements");
    Serial.println("3. Continuous Monitoring");
    Serial.println("m. Main menu");
    Serial.println("Enter selection:");

    if (const String cmd = wait_for_input(); cmd == "1") {
        get_all_measurements();
    } else if (cmd == "2") {
        individual_measurements_menu();
    } else if (cmd == "3") {
        continuous_monitoring();
    } else if (cmd == "m") {
        print_main_menu();
    }
}

// Helper Functions
String wait_for_input() {
    while (!Serial.available()) { delay(10); }
    String input = Serial.readStringUntil('\n');
    input.trim();
    return input;
}

int get_int_input(const char* prompt) {
    Serial.print(prompt);
    return wait_for_input().toInt();
}

float get_float_input(const char* prompt) {
    Serial.print(prompt);
    return wait_for_input().toFloat();
}

bool get_bool_input(const char* prompt) {
    Serial.print(prompt);
    Serial.print(" (1=true, 0=false): ");
    return wait_for_input().toInt() != 0;
}

void print_status_result(const bq25798_status_t status, const char* operation) {
    if (status == BQ25798_OK) {
        Serial.print("✓ ");
        Serial.print(operation);
        Serial.println(" successful");
    } else {
        Serial.print("✗ ");
        Serial.print(operation);
        Serial.print(" failed: ");
        Serial.println(bq25798_stat_error(status));
    }
}

// Implementation of specific menu functions

void reset_settings() {
    const bq25798_status_t status = bq25798_reset_settings(&bq_device);
    print_status_result(status, "Reset settings");
}

void get_device_status() {
    Serial.println("\n=== DEVICE STATUS ===");
    Serial.print("Initialized: ");
    Serial.println(device_initialized ? "Yes" : "No");
    Serial.print("I2C Address: 0x");
    Serial.println(bq_device.i2c_address, HEX);
}

void vsysmin_menu() {
    Serial.println("\n--- VSYSMIN Control ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        int vsysmin;
        if (const bq25798_status_t status = bq25798_get_vsysmin(&bq_device, &vsysmin); status == BQ25798_OK) {
            Serial.print("VSYSMIN: ");
            Serial.print(vsysmin);
            Serial.println(" mV");
        } else {
            print_status_result(status, "Get VSYSMIN");
        }
    } else if (cmd == "2") {
        const int vsysmin = get_int_input("Enter VSYSMIN (2500-16000 mV): ");
        const bq25798_status_t status = bq25798_set_vsysmin(&bq_device, vsysmin);
        print_status_result(status, "Set VSYSMIN");
    }
}

void charge_voltage_limit_menu() {
    Serial.println("\n--- Charge Voltage Limit ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        int vreg_lim;
        if (const bq25798_status_t status = bq25798_get_charge_limit_v(&bq_device, &vreg_lim); status == BQ25798_OK) {
            Serial.print("Charge Voltage Limit: ");
            Serial.print(vreg_lim);
            Serial.println(" mV");
        } else {
            print_status_result(status, "Get charge voltage limit");
        }
    } else if (cmd == "2") {
        const int vreg_lim = get_int_input("Enter charge voltage limit (3000-18800 mV): ");
        const bq25798_status_t status = bq25798_set_charge_limit_v(&bq_device, vreg_lim);
        print_status_result(status, "Set charge voltage limit");
    }
}

void charge_current_limit_menu() {
    Serial.println("\n--- Charge Current Limit ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        int ireg_lim;
        if (const bq25798_status_t status = bq25798_get_charge_limit_i(&bq_device, &ireg_lim); status == BQ25798_OK) {
            Serial.print("Charge Current Limit: ");
            Serial.print(ireg_lim);
            Serial.println(" mA");
        } else {
            print_status_result(status, "Get charge current limit");
        }
    } else if (cmd == "2") {
        const int ireg_lim = get_int_input("Enter charge current limit (50-5000 mA): ");
        const bq25798_status_t status = bq25798_set_charge_limit_i(&bq_device, ireg_lim);
        print_status_result(status, "Set charge current limit");
    }
}

void input_voltage_limit_menu() {
    Serial.println("\n--- Input Voltage Limit ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        int vin_lim;
        if (const bq25798_status_t status = bq25798_get_input_limit_v(&bq_device, &vin_lim); status == BQ25798_OK) {
            Serial.print("Input Voltage Limit: ");
            Serial.print(vin_lim);
            Serial.println(" mV");
        } else {
            print_status_result(status, "Get input voltage limit");
        }
    } else if (cmd == "2") {
        const int vin_lim = get_int_input("Enter input voltage limit (3600-22000 mV): ");
        const bq25798_status_t status = bq25798_set_input_limit_v(&bq_device, vin_lim);
        print_status_result(status, "Set input voltage limit");
    }
}

void input_current_limit_menu() {
    Serial.println("\n--- Input Current Limit ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        int iin_lim;
        if (const bq25798_status_t status = bq25798_get_input_limit_i(&bq_device, &iin_lim); status == BQ25798_OK) {
            Serial.print("Input Current Limit: ");
            Serial.print(iin_lim);
            Serial.println(" mA");
        } else {
            print_status_result(status, "Get input current limit");
        }
    } else if (cmd == "2") {
        const int iin_lim = get_int_input("Enter input current limit (100-3300 mA): ");
        const bq25798_status_t status = bq25798_set_input_limit_i(&bq_device, iin_lim);
        print_status_result(status, "Set input current limit");
    }
}

void vbat_lowv_menu() {
    Serial.println("\n--- VBAT Low Voltage Threshold ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bq25798_vbat_lowv_t threshold;
        if (const bq25798_status_t status = bq25798_get_vbat_lowv(&bq_device, &threshold); status == BQ25798_OK) {
            Serial.print("VBAT Low Voltage Threshold: ");
            switch(threshold) {
                case BQ25798_VBAT_LOWV_15_PERCENT: Serial.println("15% of VREG"); break;
                case BQ25798_VBAT_LOWV_62_2_PERCENT: Serial.println("62.5% of VREG"); break;
                case BQ25798_VBAT_LOWV_66_7_PERCENT: Serial.println("66.7% of VREG"); break;
                case BQ25798_VBAT_LOWV_71_4_PERCENT: Serial.println("71.4% of VREG"); break;
            }
        } else {
            print_status_result(status, "Get VBAT low voltage threshold");
        }
    } else if (cmd == "2") {
        Serial.println("Select threshold:");
        Serial.println("0: 15% of VREG");
        Serial.println("1: 62.5% of VREG");
        Serial.println("2: 66.7% of VREG");
        Serial.println("3: 71.4% of VREG");
        int selection = get_int_input("Enter selection (0-3): ");
        const auto setting = static_cast<bq25798_ts_cool_t>(selection);
        const bq25798_status_t status = bq25798_set_ts_cool_setting(&bq_device, setting);
        print_status_result(status, "Set TS cool setting");
    }
}

void ts_warm_menu() {
    Serial.println("\n--- TS Warm Setting ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bq25798_ts_warm_t setting;
        if (const bq25798_status_t status = bq25798_get_ts_warm_setting(&bq_device, &setting); status == BQ25798_OK) {
            Serial.print("TS Warm Setting: ");
            switch(setting) {
                case BQ25798_TS_WARM_48_4_PERCENT: Serial.println("48.4% of REGN"); break;
                case BQ25798_TS_WARM_44_8_PERCENT: Serial.println("44.8% of REGN"); break;
                case BQ25798_TS_WARM_41_2_PERCENT: Serial.println("41.2% of REGN"); break;
                case BQ25798_TS_WARM_37_7_PERCENT: Serial.println("37.7% of REGN"); break;
            }
        } else {
            print_status_result(status, "Get TS warm setting");
        }
    } else if (cmd == "2") {
        Serial.println("Select TS warm setting:");
        Serial.println("0: 48.4% of REGN");
        Serial.println("1: 44.8% of REGN");
        Serial.println("2: 41.2% of REGN");
        Serial.println("3: 37.7% of REGN");
        int selection = get_int_input("Enter selection (0-3): ");
        const auto setting = static_cast<bq25798_ts_warm_t>(selection);
        const bq25798_status_t status = bq25798_set_ts_warm_setting(&bq_device, setting);
        print_status_result(status, "Set TS warm setting");
    }
}

void ts_hot_menu() {
    Serial.println("\n--- TS Hot Setting ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bq25798_bhot_t setting;
        if (const bq25798_status_t status = bq25798_get_ts_hot_setting(&bq_device, &setting); status == BQ25798_OK) {
            Serial.print("TS Hot Setting: ");
            switch(setting) {
                case BQ25798_BHOT_55C: Serial.println("55°C"); break;
                case BQ25798_BHOT_60C: Serial.println("60°C"); break;
                case BQ25798_BHOT_65C: Serial.println("65°C"); break;
                case BQ25798_BHOT_DIS: Serial.println("Disabled"); break;
            }
        } else {
            print_status_result(status, "Get TS hot setting");
        }
    } else if (cmd == "2") {
        Serial.println("Select TS hot setting:");
        Serial.println("0: 55°C");
        Serial.println("1: 60°C");
        Serial.println("2: 65°C");
        Serial.println("3: Disabled");
        int selection = get_int_input("Enter selection (0-3): ");
        const auto setting = static_cast<bq25798_bhot_t>(selection);
        const bq25798_status_t status = bq25798_set_ts_hot_setting(&bq_device, setting);
        print_status_result(status, "Set TS hot setting");
    }
}

void ts_cold_menu() {
    Serial.println("\n--- TS Cold Setting ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bq25798_bcold_t setting;
        if (const bq25798_status_t status = bq25798_get_ts_cold_setting(&bq_device, &setting); status == BQ25798_OK) {
            Serial.print("TS Cold Setting: ");
            switch(setting) {
                case BQ25798_BCOLD_N10C: Serial.println("-10°C"); break;
                case BQ25798_BCOLD_N20C: Serial.println("-20°C"); break;
            }
        } else {
            print_status_result(status, "Get TS cold setting");
        }
    } else if (cmd == "2") {
        Serial.println("Select TS cold setting:");
        Serial.println("0: -10°C");
        Serial.println("1: -20°C");
        int selection = get_int_input("Enter selection (0-1): ");
        const auto setting = static_cast<bq25798_bcold_t>(selection);
        const bq25798_status_t status = bq25798_set_ts_cold_setting(&bq_device, setting);
        print_status_result(status, "Set TS cold setting");
    }
}

void ts_ignore_menu() {
    Serial.println("\n--- TS Ignore ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bool ignore;
        if (const bq25798_status_t status = bq25798_get_ts_ignore(&bq_device, &ignore); status == BQ25798_OK) {
            Serial.print("TS Ignore: ");
            Serial.println(ignore ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get TS ignore");
        }
    } else if (cmd == "2") {
        const bool ignore = get_bool_input("Ignore TS feedback?");
        const bq25798_status_t status = bq25798_set_ts_ignore(&bq_device, ignore);
        print_status_result(status, "Set TS ignore");
    }
}

// Status and Flag Monitoring Functions
void get_all_charger_status() {
    Serial.println("\n=== ALL CHARGER STATUS ===");
    bq25798_charger_status_t status;

    if (const bq25798_status_t result = bq25798_check_charger_status(&bq_device, &status); result == BQ25798_OK) {
        Serial.print("IINDPM Status: "); Serial.println(status.iindpm_stat ? "Active" : "Inactive");
        Serial.print("VINDPM Status: "); Serial.println(status.vindpm_stat ? "Active" : "Inactive");
        Serial.print("Watchdog Status: "); Serial.println(status.watchdog_stat ? "Expired" : "Normal");
        Serial.print("Power Good: "); Serial.println(status.pg_stat ? "Good" : "Not Good");
        Serial.print("AC2 Present: "); Serial.println(status.ac2_present_stat ? "Yes" : "No");
        Serial.print("AC1 Present: "); Serial.println(status.ac1_present_stat ? "Yes" : "No");
        Serial.print("VBUS Present: "); Serial.println(status.vbus_present_stat ? "Yes" : "No");
        Serial.print("Charger Status: ");
        switch(status.chg_stat) {
            case BQ25798_CHG_STAT_NOT_CHARGING: Serial.println("Not charging"); break;
            case BQ25798_CHG_STAT_TRICKLE_CHARGE: Serial.println("Trickle charge"); break;
            case BQ25798_CHG_STAT_PRE_CHARGE: Serial.println("Pre-charge"); break;
            case BQ25798_CHG_STAT_FAST_CHARGE: Serial.println("Fast charge"); break;
            case BQ25798_CHG_STAT_TAPER_CHARGE: Serial.println("Taper charge"); break;
            case BQ25798_CHG_STAT_TOP_OFF: Serial.println("Top-off"); break;
            case BQ25798_CHG_STAT_CHARGE_TERM: Serial.println("Charge terminated"); break;
        }
        Serial.print("VBUS Status: ");
        switch(status.vbus_stat) {
            case BQ25798_VBUS_STAT_NO_INPUT: Serial.println("No input"); break;
            case BQ25798_VBUS_STAT_USB_SDP: Serial.println("USB SDP"); break;
            case BQ25798_VBUS_STAT_USB_CDP: Serial.println("USB CDP"); break;
            case BQ25798_VBUS_STAT_USB_DCP: Serial.println("USB DCP"); break;
            case BQ25798_VBUS_STAT_HVDCP: Serial.println("HVDCP"); break;
            case BQ25798_VBUS_STAT_UNKNOWN: Serial.println("Unknown adapter"); break;
            case BQ25798_VBUS_STAT_NON_STANDARD: Serial.println("Non-standard"); break;
            case BQ25798_VBUS_STAT_OTG_MODE: Serial.println("OTG mode"); break;
            case BQ25798_VBUS_STAT_NON_QUAL: Serial.println("Non-qualified"); break;
            case BQ25798_VBUS_STAT_DIRECT_VBUS: Serial.println("Direct VBUS"); break;
            case BQ25798_VBUS_STAT_BACKUP_MODE: Serial.println("Backup mode"); break;
        }
        Serial.print("BC1.2 Done: "); Serial.println(status.bc_1_2_done_stat ? "Yes" : "No");
        Serial.print("ICO Status: ");
        switch(status.ico_stat) {
            case BQ25798_ICO_STAT_DISABLE: Serial.println("Disabled"); break;
            case BQ25798_ICO_STAT_RUNNING: Serial.println("Running"); break;
            case BQ25798_ICO_STAT_MAX_CURR: Serial.println("Max current"); break;
        }
        Serial.print("Thermal Regulation: "); Serial.println(status.treg_stat ? "Active" : "Inactive");
        Serial.print("D+/D- Detection: "); Serial.println(status.dpdm_stat ? "Done" : "In progress");
        Serial.print("Battery Present: "); Serial.println(status.vbat_present_stat ? "Yes" : "No");
        Serial.print("ACRB2 Status: "); Serial.println(status.acrb2_stat ? "On" : "Off");
        Serial.print("ACRB1 Status: "); Serial.println(status.acrb1_stat ? "On" : "Off");
        Serial.print("ADC Done: "); Serial.println(status.adc_done_stat ? "Yes" : "No");
        Serial.print("VSYS Regulation: "); Serial.println(status.vsys_stat ? "Active" : "Inactive");
        Serial.print("Charge Timer: "); Serial.println(status.chg_tmr_stat ? "Expired" : "Normal");
        Serial.print("Trickle Timer: "); Serial.println(status.trichg_tmr_stat ? "Expired" : "Normal");
        Serial.print("Precharge Timer: "); Serial.println(status.prechg_tmr_stat ? "Expired" : "Normal");
        Serial.print("VBAT OTG Low: "); Serial.println(status.vbat_otg_low_stat ? "Yes" : "No");
        Serial.print("TS Cold: "); Serial.println(status.ts_cold_stat ? "Yes" : "No");
        Serial.print("TS Cool: "); Serial.println(status.ts_cool_stat ? "Yes" : "No");
        Serial.print("TS Warm: "); Serial.println(status.ts_warm_stat ? "Yes" : "No");
        Serial.print("TS Hot: "); Serial.println(status.ts_hot_stat ? "Yes" : "No");
    } else {
        print_status_result(result, "Get charger status");
    }
}

void get_all_fault_status() {
    Serial.println("\n=== ALL FAULT STATUS ===");
    bq25798_fault_status_t status;

    if (const bq25798_status_t result = bq25798_check_fault_status(&bq_device, &status); result == BQ25798_OK) {
        Serial.print("IBAT Regulation: "); Serial.println(status.ibat_reg_stat ? "Active" : "Inactive");
        Serial.print("VBUS OVP: "); Serial.println(status.vbus_ovp_stat ? "Fault" : "Normal");
        Serial.print("VBAT OVP: "); Serial.println(status.vbat_ovp_stat ? "Fault" : "Normal");
        Serial.print("IBUS OCP: "); Serial.println(status.ibus_ocp_stat ? "Fault" : "Normal");
        Serial.print("IBAT OCP: "); Serial.println(status.ibat_ocp_stat ? "Fault" : "Normal");
        Serial.print("Converter OCP: "); Serial.println(status.conv_ocp_stat ? "Fault" : "Normal");
        Serial.print("VAC2 OVP: "); Serial.println(status.vac2_ovp_stat ? "Fault" : "Normal");
        Serial.print("VAC1 OVP: "); Serial.println(status.vac1_ovp_stat ? "Fault" : "Normal");
        Serial.print("VSYS Short: "); Serial.println(status.vsys_short_stat ? "Fault" : "Normal");
        Serial.print("VSYS OVP: "); Serial.println(status.vsys_ovp_stat ? "Fault" : "Normal");
        Serial.print("OTG OVP: "); Serial.println(status.otg_ovp_stat ? "Fault" : "Normal");
        Serial.print("OTG UVP: "); Serial.println(status.otg_uvp_stat ? "Fault" : "Normal");
        Serial.print("Thermal Shutdown: "); Serial.println(status.tshut_stat ? "Active" : "Normal");
    } else {
        print_status_result(result, "Get fault status");
    }
}

void individual_status_menu() {
    Serial.println("\n--- Individual Status Values ---");
    Serial.println("Select status to check:");
    Serial.println("1. Power Good");
    Serial.println("2. Charger Status");
    Serial.println("3. VBUS Status");
    Serial.println("4. ICO Status");
    Serial.println("5. Thermal Regulation");
    Serial.println("6. Battery Present");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bool pg_stat;
        if (const bq25798_status_t status = bq25798_get_power_good_status(&bq_device, &pg_stat); status == BQ25798_OK) {
            Serial.print("Power Good: ");
            Serial.println(pg_stat ? "Good" : "Not Good");
        } else {
            print_status_result(status, "Get power good status");
        }
    }
    // Add more individual status checks as needed...
}

void get_all_charger_flags() {
    Serial.println("\n=== ALL CHARGER FLAGS ===");
    uint32_t flags;

    if (const bq25798_status_t status = bq25798_check_charger_flags(&bq_device, &flags); status == BQ25798_OK) {
        Serial.print("Charger Flags (hex): 0x");
        Serial.println(flags, HEX);
        Serial.println("Use individual flag menu for detailed breakdown");
    } else {
        print_status_result(status, "Get charger flags");
    }
}

void get_all_fault_flags() {
    Serial.println("\n=== ALL FAULT FLAGS ===");
    uint16_t flags;

    if (const bq25798_status_t status = bq25798_check_fault_flags(&bq_device, &flags); status == BQ25798_OK) {
        Serial.print("Fault Flags (hex): 0x");
        Serial.println(flags, HEX);
        Serial.println("Use individual flag menu for detailed breakdown");
    } else {
        print_status_result(status, "Get fault flags");
    }
}

void individual_flags_menu() {
    Serial.println("\n--- Individual Flag Values ---");
    Serial.println("1. IINDPM Flag");
    Serial.println("2. VINDPM Flag");
    Serial.println("3. Watchdog Flag");
    Serial.println("4. Power Good Flag");
    Serial.println("5. VBUS Present Flag");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bool flag;
        if (const bq25798_status_t status = bq25798_get_iindpm_flag(&bq_device, &flag); status == BQ25798_OK) {
            Serial.print("IINDPM Flag: ");
            Serial.println(flag ? "Set" : "Clear");
        } else {
            print_status_result(status, "Get IINDPM flag");
        }
    }
    // Add more individual flag checks as needed...
}

// ADC Control Functions
void adc_enable_menu() {
    Serial.println("\n--- ADC Enable ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bool enable;
        if (const bq25798_status_t status = bq25798_get_adc_enable(&bq_device, &enable); status == BQ25798_OK) {
            Serial.print("ADC Enable: ");
            Serial.println(enable ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get ADC enable");
        }
    } else if (cmd == "2") {
        const bool enable = get_bool_input("Enable ADC?");
        const bq25798_status_t status = bq25798_set_adc_enable(&bq_device, enable);
        print_status_result(status, "Set ADC enable");
    }
}

void adc_rate_menu() {
    Serial.println("\n--- ADC Rate ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bool rate;
        if (const bq25798_status_t status = bq25798_get_adc_rate(&bq_device, &rate); status == BQ25798_OK) {
            Serial.print("ADC Rate: ");
            Serial.println(rate ? "Continuous" : "One-shot");
        } else {
            print_status_result(status, "Get ADC rate");
        }
    } else if (cmd == "2") {
        const bool rate = get_bool_input("Set continuous mode? (0=one-shot, 1=continuous)");
        const bq25798_status_t status = bq25798_set_adc_rate(&bq_device, rate);
        print_status_result(status, "Set ADC rate");
    }
}

void adc_resolution_menu() {
    Serial.println("\n--- ADC Resolution ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bq25798_adc_speed_t resolution;
        if (const bq25798_status_t status = bq25798_get_adc_resolution(&bq_device, &resolution); status == BQ25798_OK) {
            Serial.print("ADC Resolution: ");
            switch(resolution) {
                case BQ25798_ADC_15BIT: Serial.println("15-bit"); break;
                case BQ25798_ADC_14BIT: Serial.println("14-bit"); break;
                case BQ25798_ADC_13BIT: Serial.println("13-bit"); break;
                case BQ25798_ADC_12BIT: Serial.println("12-bit"); break;
            }
        } else {
            print_status_result(status, "Get ADC resolution");
        }
    } else if (cmd == "2") {
        Serial.println("Select ADC resolution:");
        Serial.println("0: 15-bit");
        Serial.println("1: 14-bit");
        Serial.println("2: 13-bit");
        Serial.println("3: 12-bit");
        int selection = get_int_input("Enter selection (0-3): ");
        const auto resolution = static_cast<bq25798_adc_speed_t>(selection);
        const bq25798_status_t status = bq25798_set_adc_resolution(&bq_device, resolution);
        print_status_result(status, "Set ADC resolution");
    }
}

void adc_averaging_menu() {
    Serial.println("\n--- ADC Averaging Control ---");
    Serial.println("1. Get averaging enable");
    Serial.println("2. Set averaging enable");
    Serial.println("3. Get averaging init value");
    Serial.println("4. Set averaging init value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bool avg;
        if (const bq25798_status_t status = bq25798_get_adc_averageing_ctrl(&bq_device, &avg); status == BQ25798_OK) {
            Serial.print("ADC Averaging: ");
            Serial.println(avg ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get ADC averaging");
        }
    } else if (cmd == "2") {
        const bool avg = get_bool_input("Enable ADC averaging?");
        const bq25798_status_t status = bq25798_set_adc_averageing_ctrl(&bq_device, avg);
        print_status_result(status, "Set ADC averaging");
    } else if (cmd == "3") {
        bool init;
        if (const bq25798_status_t status = bq25798_get_adc_avg_init_val(&bq_device, &init); status == BQ25798_OK) {
            Serial.print("ADC Avg Init Value: ");
            Serial.println(init ? "Use current reading" : "Use zero");
        } else {
            print_status_result(status, "Get ADC avg init value");
        }
    } else if (cmd == "4") {
        const bool init = get_bool_input("Use current reading as init value? (0=zero, 1=current)");
        const bq25798_status_t status = bq25798_set_adc_avg_init_val(&bq_device, init);
        print_status_result(status, "Set ADC avg init value");
    }
}

void adc_channels_menu() {
    Serial.println("\n--- Individual ADC Channel Control ---");
    Serial.println("1. IBUS ADC");
    Serial.println("2. IBAT ADC");
    Serial.println("3. VBUS ADC");
    Serial.println("4. VBAT ADC");
    Serial.println("5. VSYS ADC");
    Serial.println("6. TS ADC");
    Serial.println("7. TDIE ADC");
    Serial.println("8. D+ ADC");
    Serial.println("9. D- ADC");
    Serial.println("10. VAC1 ADC");
    Serial.println("11. VAC2 ADC");

    // Example for IBUS ADC
    if (const String cmd = wait_for_input(); cmd == "1") {
        Serial.println("IBUS ADC Control:");
        Serial.println("1. Get disable status");
        Serial.println("2. Set disable status");

        if (const String subcmd = wait_for_input(); subcmd == "1") {
            bool disable;
            if (const bq25798_status_t status = bq25798_get_ibus_adc_disable(&bq_device, &disable); status == BQ25798_OK) {
                Serial.print("IBUS ADC: ");
                Serial.println(disable ? "Disabled" : "Enabled");
            } else {
                print_status_result(status, "Get IBUS ADC disable");
            }
        } else if (subcmd == "2") {
            const bool disable = get_bool_input("Disable IBUS ADC?");
            const bq25798_status_t status = bq25798_set_ibus_adc_disable(&bq_device, disable);
            print_status_result(status, "Set IBUS ADC disable");
        }
    }
    // Add similar implementations for other ADC channels...
}

// Measurement Functions
void get_all_measurements() {
    Serial.println("\n=== ALL MEASUREMENTS ===");
    bq25798_measurements_t measurements;

    if (const bq25798_status_t status = bq25798_get_adc_mesurements(&bq_device, &measurements); status == BQ25798_OK) {
        Serial.print("IBUS: "); Serial.print(measurements.ibus); Serial.println(" mA");
        Serial.print("IBAT: "); Serial.print(measurements.ibat); Serial.println(" mA");
        Serial.print("VBUS: "); Serial.print(measurements.vbus); Serial.println(" mV");
        Serial.print("VAC1: "); Serial.print(measurements.vac1); Serial.println(" mV");
        Serial.print("VAC2: "); Serial.print(measurements.vac2); Serial.println(" mV");
        Serial.print("VBAT: "); Serial.print(measurements.vbat); Serial.println(" mV");
        Serial.print("VSYS: "); Serial.print(measurements.vsys); Serial.println(" mV");
        Serial.print("TS: "); Serial.print(measurements.ts); Serial.println(" %");
        Serial.print("TDIE: "); Serial.print(measurements.tdie); Serial.println(" °C");
        Serial.print("D+: "); Serial.print(measurements.dp); Serial.println(" mV");
        Serial.print("D-: "); Serial.print(measurements.dm); Serial.println(" mV");
    } else {
        print_status_result(status, "Get all measurements");
    }
}

void individual_measurements_menu() {
    Serial.println("\n--- Individual Measurements ---");
    Serial.println("1. IBUS");
    Serial.println("2. IBAT");
    Serial.println("3. VBUS");
    Serial.println("4. VBAT");
    Serial.println("5. VSYS");
    Serial.println("6. TS");
    Serial.println("7. TDIE");

    if (const String cmd = wait_for_input(); cmd == "1") {
        int ibus;
        if (const bq25798_status_t status = bq25798_get_ibus_measurement(&bq_device, &ibus); status == BQ25798_OK) {
            Serial.print("IBUS: ");
            Serial.print(ibus);
            Serial.println(" mA");
        } else {
            print_status_result(status, "Get IBUS measurement");
        }
    }
    // Add other individual measurements...
}

void continuous_monitoring() {
    Serial.println("\n=== CONTINUOUS MONITORING ===");
    Serial.println("Press any key to stop...");
    
    while (!Serial.available()) {
        bq25798_measurements_t measurements;

        if (const bq25798_status_t status = bq25798_get_adc_mesurements(&bq_device, &measurements); status == BQ25798_OK) {
            Serial.print("VBUS: "); Serial.print(measurements.vbus); Serial.print("mV | ");
            Serial.print("VBAT: "); Serial.print(measurements.vbat); Serial.print("mV | ");
            Serial.print("IBUS: "); Serial.print(measurements.ibus); Serial.print("mA | ");
            Serial.print("IBAT: "); Serial.print(measurements.ibat); Serial.print("mA | ");
            Serial.print("TDIE: "); Serial.print(measurements.tdie); Serial.println("°C");
        } else {
            Serial.println("Error reading measurements");
        }
        
        delay(1000);
    }
    
    // Clear the input buffer
    while (Serial.available()) {
        Serial.read();
    }
    
    Serial.println("Monitoring stopped.");
}

void precharge_current_menu() {
    Serial.println("\n--- Precharge Current Limit ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        int precharge_lim;
        if (const bq25798_status_t status = bq25798_get_precharge_lim_i(&bq_device, &precharge_lim); status == BQ25798_OK) {
            Serial.print("Precharge Current Limit: ");
            Serial.print(precharge_lim);
            Serial.println(" mA");
        } else {
            print_status_result(status, "Get precharge current limit");
        }
    } else if (cmd == "2") {
        const int precharge_lim = get_int_input("Enter precharge current limit (40-2000 mA): ");
        const bq25798_status_t status = bq25798_set_precharge_lim_i(&bq_device, precharge_lim);
        print_status_result(status, "Set precharge current limit");
    }
}

void termination_current_menu() {
    Serial.println("\n--- Termination Current ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        int term_curr;
        if (const bq25798_status_t status = bq25798_get_termination_curr(&bq_device, &term_curr); status == BQ25798_OK) {
            Serial.print("Termination Current: ");
            Serial.print(term_curr);
            Serial.println(" mA");
        } else {
            print_status_result(status, "Get termination current");
        }
    } else if (cmd == "2") {
        const int term_curr = get_int_input("Enter termination current (40-1000 mA): ");
        const bq25798_status_t status = bq25798_set_termination_curr(&bq_device, term_curr);
        print_status_result(status, "Set termination current");
    }
}

void cell_count_menu() {
    Serial.println("\n--- Cell Count ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bq25798_cell_count_t cellCount;
        if (const bq25798_status_t status = bq25798_get_cell_count(&bq_device, &cellCount); status == BQ25798_OK) {
            Serial.print("Cell Count: ");
            switch(cellCount) {
                case BQ25798_CELL_COUNT_1S: Serial.println("1S"); break;
                case BQ25798_CELL_COUNT_2S: Serial.println("2S"); break;
                case BQ25798_CELL_COUNT_3S: Serial.println("3S"); break;
                case BQ25798_CELL_COUNT_4S: Serial.println("4S"); break;
            }
        } else {
            print_status_result(status, "Get cell count");
        }
    } else if (cmd == "2") {
        Serial.println("Select cell count:");
        Serial.println("0: 1S");
        Serial.println("1: 2S");
        Serial.println("2: 3S");
        Serial.println("3: 4S");
        int selection = get_int_input("Enter selection (0-3): ");
        const auto cellCount = static_cast<bq25798_cell_count_t>(selection);
        const bq25798_status_t status = bq25798_set_cell_count(&bq_device, cellCount);
        print_status_result(status, "Set cell count");
    }
}

void deglitch_time_menu() {
    Serial.println("\n--- Deglitch Time ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");

    if (const String cmd = wait_for_input(); cmd == "1") {
        bq25798_trechg_time_t deglitchTime;
        if (const bq25798_status_t status = bq25798_get_deglitch_time(&bq_device, &deglitchTime); status == BQ25798_OK) {
            Serial.print("Deglitch Time: ");
            switch(deglitchTime) {
            case BQ25798_TRECHG_64MS: Serial.println("64ms"); break;
            case BQ25798_TRECHG_256MS: Serial.println("256ms"); break;
            case BQ25798_TRECHG_1024MS: Serial.println("1024ms"); break;
            case BQ25798_TRECHG_2048MS: Serial.println("2048ms"); break;
            }
        } else {
            print_status_result(status, "Get deglitch time");
        }
    } else if (cmd == "2") {
        Serial.println("Select deglitch time:");
        Serial.println("0: 64ms");
        Serial.println("1: 256ms");
        Serial.println("2: 1024ms");
        Serial.println("3: 2048ms");
        int selection = get_int_input("Enter selection (0-3): ");
        const auto deglitchTime = static_cast<bq25798_trechg_time_t>(selection);
        const bq25798_status_t status = bq25798_set_deglitch_time(&bq_device, deglitchTime);
        print_status_result(status, "Set deglitch time");
    }
}

void recharge_threshold_menu() {
    Serial.println("\n--- Recharge Threshold Offset ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");
    String cmd = wait_for_input();

    if (cmd == "1") {
        int thresholdOffset;
        bq25798_status_t status = bq25798_get_recharge_threshold_offset(&bq_device, &thresholdOffset);
        if (status == BQ25798_OK) {
            Serial.print("Recharge Threshold Offset: ");
            Serial.print(thresholdOffset);
            Serial.println(" mV below VREG");
        } else {
            print_status_result(status, "Get recharge threshold offset");
        }
    } else if (cmd == "2") {
        int thresholdOffset = get_int_input("Enter recharge threshold offset (50-800 mV): ");
        bq25798_status_t status = bq25798_set_recharge_threshold_offset(&bq_device, thresholdOffset);
        print_status_result(status, "Set recharge threshold offset");
    }
}

void auto_battery_discharge_menu() {
    Serial.println("\n--- Auto Battery Discharge on OVP ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool auto_ibatdis;
        bq25798_status_t status = bq25798_get_auto_ovp_batt_discharge(&bq_device, &auto_ibatdis);
        if (status == BQ25798_OK) {
            Serial.print("Auto Battery Discharge on OVP: ");
            Serial.println(auto_ibatdis ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get auto battery discharge");
        }
    } else if (cmd == "2") {
        bool enable = get_bool_input("Enable auto battery discharge on OVP?");
        bq25798_status_t status = bq25798_set_auto_ovp_batt_discharge(&bq_device, enable);
        print_status_result(status, "Set auto battery discharge");
    }
}

void force_battery_discharge_menu() {
    Serial.println("\n--- Force Battery Discharge ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool force_ibatdis;
        bq25798_status_t status = bq25798_get_force_ovp_batt_discharge(&bq_device, &force_ibatdis);
        if (status == BQ25798_OK) {
            Serial.print("Force Battery Discharge: ");
            Serial.println(force_ibatdis ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get force battery discharge");
        }
    } else if (cmd == "2") {
        bool enable = get_bool_input("Enable force battery discharge?");
        bq25798_status_t status = bq25798_set_force_ovp_batt_discharge(&bq_device, enable);
        print_status_result(status, "Set force battery discharge");
    }
}

void charge_enable_menu() {
    Serial.println("\n--- Charge Enable ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool en_chg;
        bq25798_status_t status = bq25798_get_charge_enable(&bq_device, &en_chg);
        if (status == BQ25798_OK) {
            Serial.print("Charge Enable: ");
            Serial.println(en_chg ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get charge enable");
        }
    } else if (cmd == "2") {
        bool enable = get_bool_input("Enable charging?");
        bq25798_status_t status = bq25798_set_charge_enable(&bq_device, enable);
        print_status_result(status, "Set charge enable");
    }
}

void ico_enable_menu() {
    Serial.println("\n--- ICO Enable ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool en_ico;
        bq25798_status_t status = bq25798_get_ico_enable(&bq_device, &en_ico);
        if (status == BQ25798_OK) {
            Serial.print("ICO Enable: ");
            Serial.println(en_ico ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get ICO enable");
        }
    } else if (cmd == "2") {
        bool enable = get_bool_input("Enable ICO?");
        bq25798_status_t status = bq25798_set_ico_enable(&bq_device, enable);
        print_status_result(status, "Set ICO enable");
    }
}

void force_ico_menu() {
    Serial.println("\n--- Force ICO ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool force_ico;
        bq25798_status_t status = bq25798_get_force_ico_enable(&bq_device, &force_ico);
        if (status == BQ25798_OK) {
            Serial.print("Force ICO: ");
            Serial.println(force_ico ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get force ICO");
        }
    } else if (cmd == "2") {
        bool enable = get_bool_input("Force start ICO?");
        bq25798_status_t status = bq25798_set_force_ico_enable(&bq_device, enable);
        print_status_result(status, "Set force ICO");
    }
}

void hiz_mode_menu() {
    Serial.println("\n--- HIZ Mode ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool en_hiz;
        bq25798_status_t status = bq25798_get_hiz_mode(&bq_device, &en_hiz);
        if (status == BQ25798_OK) {
            Serial.print("HIZ Mode: ");
            Serial.println(en_hiz ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get HIZ mode");
        }
    } else if (cmd == "2") {
        bool enable = get_bool_input("Enable HIZ mode?");
        bq25798_status_t status = bq25798_set_hiz_mode(&bq_device, enable);
        print_status_result(status, "Set HIZ mode");
    }
}

void termination_enable_menu() {
    Serial.println("\n--- Termination Enable ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool en_term;
        bq25798_status_t status = bq25798_get_termination_enable(&bq_device, &en_term);
        if (status == BQ25798_OK) {
            Serial.print("Termination Enable: ");
            Serial.println(en_term ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get termination enable");
        }
    } else if (cmd == "2") {
        bool enable = get_bool_input("Enable charge termination?");
        bq25798_status_t status = bq25798_set_termination_enable(&bq_device, enable);
        print_status_result(status, "Set termination enable");
    }
}

void backup_mode_menu() {
    Serial.println("\n--- Backup Mode ---");
    Serial.println("1. Get enable status");
    Serial.println("2. Set enable status");
    Serial.println("3. Get threshold");
    Serial.println("4. Set threshold");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool en_backup;
        bq25798_status_t status = bq25798_get_backup_mode_enable(&bq_device, &en_backup);
        if (status == BQ25798_OK) {
            Serial.print("Backup Mode: ");
            Serial.println(en_backup ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get backup mode enable");
        }
    } else if (cmd == "2") {
        bool enable = get_bool_input("Enable backup mode?");
        bq25798_status_t status = bq25798_set_backup_mode_enable(&bq_device, enable);
        print_status_result(status, "Set backup mode enable");
    } else if (cmd == "3") {
        bq25798_vbus_backup_t threshold;
        bq25798_status_t status = bq25798_get_backup_mode_threshold(&bq_device, &threshold);
        if (status == BQ25798_OK) {
            Serial.print("Backup Mode Threshold: ");
            switch(threshold) {
                case BQ25798_VBUS_BACKUP_40_PERCENT: Serial.println("40% of VINDPM"); break;
                case BQ25798_VBUS_BACKUP_60_PERCENT: Serial.println("60% of VINDPM"); break;
                case BQ25798_VBUS_BACKUP_80_PERCENT: Serial.println("80% of VINDPM"); break;
                case BQ25798_VBUS_BACKUP_100_PERCENT: Serial.println("100% of VINDPM"); break;
            }
        } else {
            print_status_result(status, "Get backup mode threshold");
        }
    } else if (cmd == "4") {
        Serial.println("Select threshold:");
        Serial.println("0: 40% of VINDPM");
        Serial.println("1: 60% of VINDPM");
        Serial.println("2: 80% of VINDPM");
        Serial.println("3: 100% of VINDPM");
        int selection = get_int_input("Enter selection (0-3): ");
        bq25798_vbus_backup_t threshold = (bq25798_vbus_backup_t)selection;
        bq25798_status_t status = bq25798_set_backup_mode_threshold(&bq_device, threshold);
        print_status_result(status, "Set backup mode threshold");
    }
}

void vac_ovp_menu() {
    Serial.println("\n--- VAC OVP Settings ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bq25798_vac_ovp_t threshold;
        bq25798_status_t status = bq25798_get_vac_ovp(&bq_device, &threshold);
        if (status == BQ25798_OK) {
            Serial.print("VAC OVP Threshold: ");
            switch(threshold) {
                case BQ25798_VAC_OVP_26V: Serial.println("26V"); break;
                case BQ25798_VAC_OVP_22V: Serial.println("22V"); break;
                case BQ25798_VAC_OVP_12V: Serial.println("12V"); break;
                case BQ25798_VAC_OVP_7V: Serial.println("7V"); break;
            }
        } else {
            print_status_result(status, "Get VAC OVP");
        }
    } else if (cmd == "2") {
        Serial.println("Select VAC OVP threshold:");
        Serial.println("0: 26V");
        Serial.println("1: 22V");
        Serial.println("2: 12V");
        Serial.println("3: 7V");
        int selection = get_int_input("Enter selection (0-3): ");
        bq25798_vac_ovp_t threshold = (bq25798_vac_ovp_t)selection;
        bq25798_status_t status = bq25798_set_vac_ovp(&bq_device, threshold);
        print_status_result(status, "Set VAC OVP");
    }
}

void watchdog_menu() {
    Serial.println("\n--- Watchdog Timer ---");
    Serial.println("1. Get current setting");
    Serial.println("2. Set new setting");
    Serial.println("3. Reset watchdog");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bq25798_wdt_t timer;
        bq25798_status_t status = bq25798_get_watchdog(&bq_device, &timer);
        if (status == BQ25798_OK) {
            Serial.print("Watchdog Timer: ");
            switch(timer) {
                case BQ25798_WDT_DISABLE: Serial.println("Disabled"); break;
                case BQ25798_WDT_0_5S: Serial.println("0.5s"); break;
                case BQ25798_WDT_1S: Serial.println("1s"); break;
                case BQ25798_WDT_2S: Serial.println("2s"); break;
                case BQ25798_WDT_20S: Serial.println("20s"); break;
                case BQ25798_WDT_40S: Serial.println("40s"); break;
                case BQ25798_WDT_80S: Serial.println("80s"); break;
                case BQ25798_WDT_160S: Serial.println("160s"); break;
            }
        } else {
            print_status_result(status, "Get watchdog timer");
        }
    } else if (cmd == "2") {
        Serial.println("Select watchdog timer:");
        Serial.println("0: Disabled");
        Serial.println("1: 0.5s");
        Serial.println("2: 1s");
        Serial.println("3: 2s");
        Serial.println("4: 20s");
        Serial.println("5: 40s");
        Serial.println("6: 80s");
        Serial.println("7: 160s");
        int selection = get_int_input("Enter selection (0-7): ");
        bq25798_wdt_t timer = (bq25798_wdt_t)selection;
        bq25798_status_t status = bq25798_set_watchdog(&bq_device, timer);
        print_status_result(status, "Set watchdog timer");
    } else if (cmd == "3") {
        bq25798_status_t status = bq25798_reset_watchdog(&bq_device, true);
        print_status_result(status, "Reset watchdog");
    }
}

void hvdcp_menu() {
    Serial.println("\n--- HVDCP Settings ---");
    Serial.println("1. Get HVDCP enable");
    Serial.println("2. Set HVDCP enable");
    Serial.println("3. Get 9V HVDCP enable");
    Serial.println("4. Set 9V HVDCP enable");
    Serial.println("5. Get 12V HVDCP enable");
    Serial.println("6. Set 12V HVDCP enable");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool hvdcp_en;
        bq25798_status_t status = bq25798_get_hvdcp_enable(&bq_device, &hvdcp_en);
        if (status == BQ25798_OK) {
            Serial.print("HVDCP Enable: ");
            Serial.println(hvdcp_en ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get HVDCP enable");
        }
    } else if (cmd == "2") {
        bool enable = get_bool_input("Enable HVDCP?");
        bq25798_status_t status = bq25798_set_hvdcp_enable(&bq_device, enable);
        print_status_result(status, "Set HVDCP enable");
    } else if (cmd == "3") {
        bool en_9v;
        bq25798_status_t status = bq25798_get_hvdcp_9V_enable(&bq_device, &en_9v);
        if (status == BQ25798_OK) {
            Serial.print("9V HVDCP Enable: ");
            Serial.println(en_9v ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get 9V HVDCP enable");
        }
    } else if (cmd == "4") {
        bool enable = get_bool_input("Enable 9V HVDCP?");
        bq25798_status_t status = bq25798_set_hvdcp_9V_enable(&bq_device, enable);
        print_status_result(status, "Set 9V HVDCP enable");
    } else if (cmd == "5") {
        bool en_12v;
        bq25798_status_t status = bq25798_get_hvdcp_12V_enable(&bq_device, &en_12v);
        if (status == BQ25798_OK) {
            Serial.print("12V HVDCP Enable: ");
            Serial.println(en_12v ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get 12V HVDCP enable");
        }
    } else if (cmd == "6") {
        bool enable = get_bool_input("Enable 12V HVDCP?");
        bq25798_status_t status = bq25798_set_hvdcp_12V_enable(&bq_device, enable);
        print_status_result(status, "Set 12V HVDCP enable");
    }
}

void sfet_control_menu() {
    Serial.println("\n--- SFET Control ---");
    Serial.println("1. Get SFET mode");
    Serial.println("2. Set SFET mode");
    Serial.println("3. Get SFET delay");
    Serial.println("4. Set SFET delay");
    Serial.println("5. Get SFET present");
    Serial.println("6. Set SFET present");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bq25798_sdrv_ctrl_t mode;
        bq25798_status_t status = bq25798_get_sfet_mode(&bq_device, &mode);
        if (status == BQ25798_OK) {
            Serial.print("SFET Mode: ");
            switch(mode) {
                case BQ25798_SDRV_IDLE: Serial.println("IDLE"); break;
                case BQ25798_SDRV_SHUTDOWN: Serial.println("Shutdown Mode"); break;
                case BQ25798_SDRV_SHIP: Serial.println("Ship Mode"); break;
                case BQ25798_SDRV_SYSTEM_RESET: Serial.println("System Power Reset"); break;
            }
        } else {
            print_status_result(status, "Get SFET mode");
        }
    } else if (cmd == "2") {
        Serial.println("Select SFET mode:");
        Serial.println("0: IDLE");
        Serial.println("1: Shutdown Mode");
        Serial.println("2: Ship Mode");
        Serial.println("3: System Power Reset");
        int selection = get_int_input("Enter selection (0-3): ");
        bq25798_sdrv_ctrl_t mode = (bq25798_sdrv_ctrl_t)selection;
        bq25798_status_t status = bq25798_set_sfet_mode(&bq_device, mode);
        print_status_result(status, "Set SFET mode");
    } else if (cmd == "3") {
        bool delay;
        bq25798_status_t status = bq25798_get_sfet_delay(&bq_device, &delay);
        if (status == BQ25798_OK) {
            Serial.print("SFET Delay: ");
            Serial.println(delay ? "10s delay" : "No delay");
        } else {
            print_status_result(status, "Get SFET delay");
        }
    } else if (cmd == "4") {
        bool enable = get_bool_input("Enable 10s delay for SFET actions?");
        bq25798_status_t status = bq25798_set_sfet_delay(&bq_device, enable);
        print_status_result(status, "Set SFET delay");
    } else if (cmd == "5") {
        bool present;
        bq25798_status_t status = bq25798_get_sfet_present(&bq_device, &present);
        if (status == BQ25798_OK) {
            Serial.print("SFET Present: ");
            Serial.println(present ? "Yes" : "No");
        } else {
            print_status_result(status, "Get SFET present");
        }
    } else if (cmd == "6") {
        bool present = get_bool_input("Is ship FET populated?");
        bq25798_status_t status = bq25798_set_sfet_present(&bq_device, present);
        print_status_result(status, "Set SFET present");
    }
}

void acdrv_menu() {
    Serial.println("\n--- ACDRV Settings ---");
    Serial.println("1. Get ACDRV disable");
    Serial.println("2. Set ACDRV disable");
    Serial.println("3. Get ACDRV1 enable");
    Serial.println("4. Set ACDRV1 enable");
    Serial.println("5. Get ACDRV2 enable");
    Serial.println("6. Set ACDRV2 enable");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bool disable;
        bq25798_status_t status = bq25798_get_acdrv_disable(&bq_device, &disable);
        if (status == BQ25798_OK) {
            Serial.print("ACDRV Disable: ");
            Serial.println(disable ? "Disabled" : "Enabled");
        } else {
            print_status_result(status, "Get ACDRV disable");
        }
    } else if (cmd == "2") {
        bool disable = get_bool_input("Disable ACDRV1 & ACDRV2?");
        bq25798_status_t status = bq25798_set_acdrv_disable(&bq_device, disable);
        print_status_result(status, "Set ACDRV disable");
    } else if (cmd == "3") {
        bool enable;
        bq25798_status_t status = bq25798_get_acdrv1_enable(&bq_device, &enable);
        if (status == BQ25798_OK) {
            Serial.print("ACDRV1 Enable: ");
            Serial.println(enable ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get ACDRV1 enable");
        }
    } else if (cmd == "4") {
        bool enable = get_bool_input("Enable ACDRV1?");
        bq25798_status_t status = bq25798_set_acdrv1_enable(&bq_device, enable);
        print_status_result(status, "Set ACDRV1 enable");
    } else if (cmd == "5") {
        bool enable;
        bq25798_status_t status = bq25798_get_acdrv2_enable(&bq_device, &enable);
        if (status == BQ25798_OK) {
            Serial.print("ACDRV2 Enable: ");
            Serial.println(enable ? "Enabled" : "Disabled");
        } else {
            print_status_result(status, "Get ACDRV2 enable");
        }
    } else if (cmd == "6") {
        bool enable = get_bool_input("Enable ACDRV2?");
        bq25798_status_t status = bq25798_set_acdrv2_enable(&bq_device, enable);
        print_status_result(status, "Set ACDRV2 enable");
    }
}

void power_management_menu() {
    Serial.println("\n--- Power Management ---");
    Serial.println("1. Get PFM disable");
    Serial.println("2. Set PFM disable");
    Serial.println("3. Get ship wakeup delay");
    Serial.println("4. Set ship wakeup delay");
    Serial.println("5. Get BATFET LDO disable");
    Serial.println("6. Set BATFET LDO disable");
    Serial.println("7. Get OOA disable");
    Serial.println("8. Set OOA disable");
    Serial.println("9. Get PWM frequency");
    Serial.println("10. Set PWM frequency");
    Serial.println("11. Get STAT pin disable");
    Serial.println("12. Set STAT pin disable");
    Serial.println("13. Get VSYS short disable");
    Serial.println("14. Set VSYS short disable");
    Serial.println("15. Get force VINDPM detect");
    Serial.println("16. Set force VINDPM detect");
    Serial.println("17. Get IBUS OCP enable");
    Serial.println("18. Set IBUS OCP enable");
    Serial.println("19. Get IBAT enable");
    Serial.println("20. Set IBAT enable");
    Serial.println("21. Get IINDPM enable");
    Serial.println("22. Set IINDPM enable");
    Serial.println("23. Get external ILIM enable");
    Serial.println("24. Set external ILIM enable");
    Serial.println("25. Get battery OCP enable");
    Serial.println("26. Set battery OCP enable");
    String cmd = wait_for_input();

    // Handle PFM disable
    if (cmd == "1") {
        bool disable;
        bq25798_status_t status = bq25798_get_pfm_disable(&bq_device, &disable);
        if (status == BQ25798_OK) {
            Serial.print("PFM Disable: ");
            Serial.println(disable ? "Disabled" : "Enabled");
        } else {
            print_status_result(status, "Get PFM disable");
        }
    } else if (cmd == "2") {
        bool disable = get_bool_input("Disable PFM in forward mode?");
        bq25798_status_t status = bq25798_set_pfm_disable(&bq_device, disable);
        print_status_result(status, "Set PFM disable");
    }
    // Handle ship wakeup delay
    else if (cmd == "3") {
        bq25798_wkup_dly_t delay;
        bq25798_status_t status = bq25798_get_ship_wakeup_delay(&bq_device, &delay);
        if (status == BQ25798_OK) {
            Serial.print("Ship Wakeup Delay: ");
            Serial.println(delay == BQ25798_WKUP_DLY_1S ? "1 second" : "15ms");
        } else {
            print_status_result(status, "Get ship wakeup delay");
        }
    } else if (cmd == "4") {
        Serial.println("Select wakeup delay:");
        Serial.println("0: 1 second");
        Serial.println("1: 15ms");
        int selection = get_int_input("Enter selection (0-1): ");
        bq25798_wkup_dly_t delay = (bq25798_wkup_dly_t)selection;
        bq25798_status_t status = bq25798_set_ship_wakeup_delay(&bq_device, delay);
        print_status_result(status, "Set ship wakeup delay");
    }
    // ... continue with other power management functions
    // (I'll provide the remaining functions in the next part due to length)
}

// Continue with the remaining functions...
void voc_percentage_menu() {
    Serial.println("\n--- VOC Percentage Setting ---");
    Serial.println("1. Get current value");
    Serial.println("2. Set new value");
    String cmd = wait_for_input();

    if (cmd == "1") {
        bq25798_voc_pct_t voc_pct;
        bq25798_status_t status = bq25798_get_voc_pct(&bq_device, &voc_pct);
        if (status == BQ25798_OK) {
            Serial.print("VOC Percentage: ");
            switch(voc_pct) {
                case BQ25798_VOC_PCT_56_25: Serial.println("56.25%"); break;
                case BQ25798_VOC_PCT_62_5: Serial.println("62.5%"); break;
                case BQ25798_VOC_PCT_68_75: Serial.println("68.75%"); break;
                case BQ25798_VOC_PCT_75: Serial.println("75%"); break;
                case BQ25798_VOC_PCT_81_25: Serial.println("81.25%"); break;
                case BQ25798_VOC_PCT_87_5: Serial.println("87.5%"); break;
                case BQ25798_VOC_PCT_93_75: Serial.println("93.75%"); break;
                case BQ25798_VOC_PCT_100: Serial.println("100%"); break;
            }
        } else {
            print_status_result(status, "Get VOC percentage");
        }
    } else if (cmd == "2") {
        Serial.println("Select VOC percentage:");
        Serial.println("0: 56.25%");
        Serial.println("1: 62.5%");
        Serial.println("2: 68.75%");
        Serial.println("3: 75%");
        Serial.println("4: 81.25%");
        Serial.println("5: 87.5%");
        Serial.println("6: 93.75%");
        Serial.println("7: 100%");
        int selection = get_int_input("Enter selection (0-7): ");
        bq25798_voc_pct_t voc_pct = (bq25798_voc_pct_t)selection;
        bq25798_status_t status = bq25798_set_voc_pct(&bq_device, voc_pct);
        print_status_result(status, "Set VOC percentage");
    }
}

void voc_delay_menu() {
   Serial.println("\n--- VOC Delay Setting ---");
   Serial.println("1. Get current value");
   Serial.println("2. Set new value");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bq25798_voc_dly_t voc_dly;
       bq25798_status_t status = bq25798_get_voc_delay(&bq_device, &voc_dly);
       if (status == BQ25798_OK) {
           Serial.print("VOC Delay: ");
           switch(voc_dly) {
               case BQ25798_VOC_DLY_50MS: Serial.println("50ms"); break;
               case BQ25798_VOC_DLY_300MS: Serial.println("300ms"); break;
               case BQ25798_VOC_DLY_2S: Serial.println("2s"); break;
               case BQ25798_VOC_DLY_5S: Serial.println("5s"); break;
           }
       } else {
           print_status_result(status, "Get VOC delay");
       }
   } else if (cmd == "2") {
       Serial.println("Select VOC delay:");
       Serial.println("0: 50ms");
       Serial.println("1: 300ms");
       Serial.println("2: 2s");
       Serial.println("3: 5s");
       int selection = get_int_input("Enter selection (0-3): ");
       bq25798_voc_dly_t voc_dly = (bq25798_voc_dly_t)selection;
       bq25798_status_t status = bq25798_set_voc_delay(&bq_device, voc_dly);
       print_status_result(status, "Set VOC delay");
   }
}

void voc_rate_menu() {
   Serial.println("\n--- VOC Measurement Rate ---");
   Serial.println("1. Get current value");
   Serial.println("2. Set new value");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bq25798_voc_rate_t voc_rate;
       bq25798_status_t status = bq25798_get_voc_measurement_rate(&bq_device, &voc_rate);
       if (status == BQ25798_OK) {
           Serial.print("VOC Measurement Rate: ");
           switch(voc_rate) {
               case BQ25798_VOC_RATE_30S: Serial.println("30s"); break;
               case BQ25798_VOC_RATE_2MIN: Serial.println("2min"); break;
               case BQ25798_VOC_RATE_10MIN: Serial.println("10min"); break;
               case BQ25798_VOC_RATE_30MIN: Serial.println("30min"); break;
           }
       } else {
           print_status_result(status, "Get VOC measurement rate");
       }
   } else if (cmd == "2") {
       Serial.println("Select VOC measurement rate:");
       Serial.println("0: 30s");
       Serial.println("1: 2min");
       Serial.println("2: 10min");
       Serial.println("3: 30min");
       int selection = get_int_input("Enter selection (0-3): ");
       bq25798_voc_rate_t voc_rate = (bq25798_voc_rate_t)selection;
       bq25798_status_t status = bq25798_set_voc_measurement_rate(&bq_device, voc_rate);
       print_status_result(status, "Set VOC measurement rate");
   }
}

void mppt_enable_menu() {
   Serial.println("\n--- MPPT Enable ---");
   Serial.println("1. Get current setting");
   Serial.println("2. Set new setting");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bool en_mppt;
       bq25798_status_t status = bq25798_get_mppt_enable(&bq_device, &en_mppt);
       if (status == BQ25798_OK) {
           Serial.print("MPPT Enable: ");
           Serial.println(en_mppt ? "Enabled" : "Disabled");
       } else {
           print_status_result(status, "Get MPPT enable");
       }
   } else if (cmd == "2") {
       bool enable = get_bool_input("Enable MPPT?");
       bq25798_status_t status = bq25798_set_mppt_enable(&bq_device, enable);
       print_status_result(status, "Set MPPT enable");
   }
}

void thermal_reg_menu() {
   Serial.println("\n--- Thermal Regulation Threshold ---");
   Serial.println("1. Get current value");
   Serial.println("2. Set new value");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bq25798_treg_t threshold;
       bq25798_status_t status = bq25798_get_thermal_reg_threshold(&bq_device, &threshold);
       if (status == BQ25798_OK) {
           Serial.print("Thermal Regulation Threshold: ");
           switch(threshold) {
               case BQ25798_TREG_60C: Serial.println("60°C"); break;
               case BQ25798_TREG_80C: Serial.println("80°C"); break;
               case BQ25798_TREG_100C: Serial.println("100°C"); break;
               case BQ25798_TREG_120C: Serial.println("120°C"); break;
           }
       } else {
           print_status_result(status, "Get thermal regulation threshold");
       }
   } else if (cmd == "2") {
       Serial.println("Select thermal regulation threshold:");
       Serial.println("0: 60°C");
       Serial.println("1: 80°C");
       Serial.println("2: 100°C");
       Serial.println("3: 120°C");
       int selection = get_int_input("Enter selection (0-3): ");
       bq25798_treg_t threshold = (bq25798_treg_t)selection;
       bq25798_status_t status = bq25798_set_thermal_reg_threshold(&bq_device, threshold);
       print_status_result(status, "Set thermal regulation threshold");
   }
}

void thermal_shutdown_menu() {
   Serial.println("\n--- Thermal Shutdown Threshold ---");
   Serial.println("1. Get current value");
   Serial.println("2. Set new value");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bq25798_tshut_t threshold;
       bq25798_status_t status = bq25798_get_thermal_shutdown_threshold(&bq_device, &threshold);
       if (status == BQ25798_OK) {
           Serial.print("Thermal Shutdown Threshold: ");
           switch(threshold) {
               case BQ25798_TSHUT_150C: Serial.println("150°C"); break;
               case BQ25798_TSHUT_130C: Serial.println("130°C"); break;
               case BQ25798_TSHUT_120C: Serial.println("120°C"); break;
               case BQ25798_TSHUT_85C: Serial.println("85°C"); break;
           }
       } else {
           print_status_result(status, "Get thermal shutdown threshold");
       }
   } else if (cmd == "2") {
       Serial.println("Select thermal shutdown threshold:");
       Serial.println("0: 150°C");
       Serial.println("1: 130°C");
       Serial.println("2: 120°C");
       Serial.println("3: 85°C");
       int selection = get_int_input("Enter selection (0-3): ");
       bq25798_tshut_t threshold = (bq25798_tshut_t)selection;
       bq25798_status_t status = bq25798_set_thermal_shutdown_threshold(&bq_device, threshold);
       print_status_result(status, "Set thermal shutdown threshold");
   }
}

void vbus_pulldown_menu() {
   Serial.println("\n--- VBUS Pulldown ---");
   Serial.println("1. Get current setting");
   Serial.println("2. Set new setting");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bool enable;
       bq25798_status_t status = bq25798_get_vbus_pulldown(&bq_device, &enable);
       if (status == BQ25798_OK) {
           Serial.print("VBUS Pulldown: ");
           Serial.println(enable ? "Enabled (6kΩ)" : "Disabled");
       } else {
           print_status_result(status, "Get VBUS pulldown");
       }
   } else if (cmd == "2") {
       bool enable = get_bool_input("Enable VBUS pulldown resistor (6kΩ)?");
       bq25798_status_t status = bq25798_set_vbus_pulldown(&bq_device, enable);
       print_status_result(status, "Set VBUS pulldown");
   }
}

void vac1_pulldown_menu() {
   Serial.println("\n--- VAC1 Pulldown ---");
   Serial.println("1. Get current setting");
   Serial.println("2. Set new setting");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bool enable;
       bq25798_status_t status = bq25798_get_vac1_pulldown(&bq_device, &enable);
       if (status == BQ25798_OK) {
           Serial.print("VAC1 Pulldown: ");
           Serial.println(enable ? "Enabled" : "Disabled");
       } else {
           print_status_result(status, "Get VAC1 pulldown");
       }
   } else if (cmd == "2") {
       bool enable = get_bool_input("Enable VAC1 pulldown resistor?");
       bq25798_status_t status = bq25798_set_vac1_pulldown(&bq_device, enable);
       print_status_result(status, "Set VAC1 pulldown");
   }
}

void vac2_pulldown_menu() {
   Serial.println("\n--- VAC2 Pulldown ---");
   Serial.println("1. Get current setting");
   Serial.println("2. Set new setting");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bool enable;
       bq25798_status_t status = bq25798_get_vac2_pulldown(&bq_device, &enable);
       if (status == BQ25798_OK) {
           Serial.print("VAC2 Pulldown: ");
           Serial.println(enable ? "Enabled" : "Disabled");
       } else {
           print_status_result(status, "Get VAC2 pulldown");
       }
   } else if (cmd == "2") {
       bool enable = get_bool_input("Enable VAC2 pulldown resistor?");
       bq25798_status_t status = bq25798_set_vac2_pulldown(&bq_device, enable);
       print_status_result(status, "Set VAC2 pulldown");
   }
}

void backup_acfet1_menu() {
   Serial.println("\n--- Backup ACFET1 ---");
   Serial.println("1. Get current setting");
   Serial.println("2. Set new setting");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bool enable;
       bq25798_status_t status = bq25798_get_backup_acfet1_on(&bq_device, &enable);
       if (status == BQ25798_OK) {
           Serial.print("Backup ACFET1: ");
           Serial.println(enable ? "On in backup mode" : "Off in backup mode");
       } else {
           print_status_result(status, "Get backup ACFET1");
       }
   } else if (cmd == "2") {
       bool enable = get_bool_input("Turn on ACFET1 in backup mode?");
       bq25798_status_t status = bq25798_set_backup_acfet1_on(&bq_device, enable);
       print_status_result(status, "Set backup ACFET1");
   }
}

void jeita_high_voltage_menu() {
   Serial.println("\n--- JEITA High Temp Voltage Setting ---");
   Serial.println("1. Get current value");
   Serial.println("2. Set new value");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bq25798_jeita_vset_t setting;
       bq25798_status_t status = bq25798_get_jeita_high_temp_voltage_setting(&bq_device, &setting);
       if (status == BQ25798_OK) {
           Serial.print("JEITA High Temp Voltage: ");
           switch(setting) {
               case BQ25798_JEITA_VSET_SUSP: Serial.println("Charge Suspend"); break;
               case BQ25798_JEITA_VSET_800mV: Serial.println("VREG-800mV"); break;
               case BQ25798_JEITA_VSET_600mV: Serial.println("VREG-600mV"); break;
               case BQ25798_JEITA_VSET_400mV: Serial.println("VREG-400mV"); break;
               case BQ25798_JEITA_VSET_300mV: Serial.println("VREG-300mV"); break;
               case BQ25798_JEITA_VSET_200mV: Serial.println("VREG-200mV"); break;
               case BQ25798_JEITA_VSET_100mV: Serial.println("VREG-100mV"); break;
               case BQ25798_JEITA_VSET_SAME: Serial.println("VREG unchanged"); break;
           }
       } else {
           print_status_result(status, "Get JEITA high temp voltage");
       }
   } else if (cmd == "2") {
       Serial.println("Select JEITA high temp voltage:");
       Serial.println("0: Charge Suspend");
       Serial.println("1: VREG-800mV");
       Serial.println("2: VREG-600mV");
       Serial.println("3: VREG-400mV");
       Serial.println("4: VREG-300mV");
       Serial.println("5: VREG-200mV");
       Serial.println("6: VREG-100mV");
       Serial.println("7: VREG unchanged");
       int selection = get_int_input("Enter selection (0-7): ");
       bq25798_jeita_vset_t setting = (bq25798_jeita_vset_t)selection;
       bq25798_status_t status = bq25798_set_jeita_high_temp_voltage_setting(&bq_device, setting);
       print_status_result(status, "Set JEITA high temp voltage");
   }
}

void jeita_high_current_menu() {
   Serial.println("\n--- JEITA High Temp Current Setting ---");
   Serial.println("1. Get current value");
   Serial.println("2. Set new value");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bq25798_jeita_iseth_t setting;
       bq25798_status_t status = bq25798_get_jeita_high_temp_current_setting(&bq_device, &setting);
       if (status == BQ25798_OK) {
           Serial.print("JEITA High Temp Current: ");
           switch(setting) {
               case BQ25798_JEITA_ISETH_SUSP: Serial.println("Charge Suspend"); break;
               case BQ25798_JEITA_ISETH_20_PERCENT: Serial.println("20% ICHG"); break;
               case BQ25798_JEITA_ISETH_40_PERCENT: Serial.println("40% ICHG"); break;
               case BQ25798_JEITA_ISETH_SAME: Serial.println("ICHG unchanged"); break;
           }
       } else {
           print_status_result(status, "Get JEITA high temp current");
       }
   } else if (cmd == "2") {
       Serial.println("Select JEITA high temp current:");
       Serial.println("0: Charge Suspend");
       Serial.println("1: 20% ICHG");
       Serial.println("2: 40% ICHG");
       Serial.println("3: ICHG unchanged");
       int selection = get_int_input("Enter selection (0-3): ");
       bq25798_jeita_iseth_t setting = (bq25798_jeita_iseth_t)selection;
       bq25798_status_t status = bq25798_set_jeita_high_temp_current_setting(&bq_device, setting);
       print_status_result(status, "Set JEITA high temp current");
   }
}

void jeita_low_current_menu() {
   Serial.println("\n--- JEITA Low Temp Current Setting ---");
   Serial.println("1. Get current value");
   Serial.println("2. Set new value");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bq25798_jeita_isetc_t setting;
       bq25798_status_t status = bq25798_get_jeita_low_temp_current_setting(&bq_device, &setting);
       if (status == BQ25798_OK) {
           Serial.print("JEITA Low Temp Current: ");
           switch(setting) {
               case BQ25798_JEITA_ISETL_SUSP: Serial.println("Charge Suspend"); break;
               case BQ25798_JEITA_ISETL_20_PERCENT: Serial.println("20% ICHG"); break;
               case BQ25798_JEITA_ISETL_40_PERCENT: Serial.println("40% ICHG"); break;
               case BQ25798_JEITA_ISETL_SAME: Serial.println("ICHG unchanged"); break;
           }
       } else {
           print_status_result(status, "Get JEITA low temp current");
       }
   } else if (cmd == "2") {
       Serial.println("Select JEITA low temp current:");
       Serial.println("0: Charge Suspend");
       Serial.println("1: 20% ICHG");
       Serial.println("2: 40% ICHG");
       Serial.println("3: ICHG unchanged");
       int selection = get_int_input("Enter selection (0-3): ");
       bq25798_jeita_isetc_t setting = (bq25798_jeita_isetc_t)selection;
       bq25798_status_t status = bq25798_set_jeita_low_temp_current_setting(&bq_device, setting);
       print_status_result(status, "Set JEITA low temp current");
   }
}

void ts_cool_menu() {
   Serial.println("\n--- TS Cool Setting ---");
   Serial.println("1. Get current value");
   Serial.println("2. Set new value");
   String cmd = wait_for_input();

   if (cmd == "1") {
       bq25798_ts_cool_t setting;
       bq25798_status_t status = bq25798_get_ts_cool_setting(&bq_device, &setting);
       if (status == BQ25798_OK) {
           Serial.print("TS Cool Setting: ");
           switch(setting) {
               case BQ25798_TS_COOL_71_1_PERCENT: Serial.println("71.1% of REGN"); break;
               case BQ25798_TS_COOL_68_4_PERCENT: Serial.println("68.4% of REGN"); break;
               case BQ25798_TS_COOL_65_5_PERCENT: Serial.println("65.5% of REGN"); break;
               case BQ25798_TS_COOL_62_4_PERCENT: Serial.println("62.4% of REGN"); break;
           }
       } else {
           print_status_result(status, "Get TS cool setting");
       }
   } else if (cmd == "2") {
       Serial.println("Select TS cool setting:");
       Serial.println("0: 71.1% of REGN");
       Serial.println("1: 68.4% of REGN");
       Serial.println("2: 65.5% of REGN");
       Serial.println("3: 62.4% of REGN");
       int selection = get_int_input("Enter selection (0-3): ");
       bq25798_ts_cool_t setting = (bq25798_ts_cool_t)selection;
       bq25798_status_t status = bq25798_set_ts_cool_setting(&bq_device, setting);
       print_status_result(status, "Set TS cool setting");
   }
}