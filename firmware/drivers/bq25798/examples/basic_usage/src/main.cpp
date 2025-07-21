/**
 * @file main.cpp
 * @brief Arduino test program for BQ25798 MPPT battery charger driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-21
 *
 * This test program demonstrates comprehensive functionality of the BQ25798 driver
 * including initialization, configuration, status monitoring, and ADC measurements.
 */

#include <Arduino.h>
#include <Wire.h>
#include <bq25798.h>
#include <bq25798_platform.h>

bq25798_t bq;
bq25798_measurements_t measurements;
bq25798_charger_status_t charger_status;
bq25798_fault_status_t fault_status;

// Test configuration values
const int TEST_VSYSMIN = 3000;          // 3.0V minimum system voltage
const int TEST_CHARGE_VOLTAGE = 4200;   // 4.2V charge voltage limit
const int TEST_CHARGE_CURRENT = 1000;   // 1A charge current limit
const int TEST_INPUT_VOLTAGE = 5000;    // 5V input voltage limit
const int TEST_INPUT_CURRENT = 2000;    // 2A input current limit
const int TEST_PRECHARGE_CURRENT = 200; // 200mA precharge current
const int TEST_TERM_CURRENT = 100;      // 100mA termination current

void setup() {
    Serial.begin(115200);
    Wire.begin();

    delay(100);

    while(!Serial) delay(10);

    Serial.println("=== BQ25798 MPPT Battery Charger Test ===");
    Serial.println();

    // Define the IO interface
    bq25798_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .delay_ms = platform_delay_ms
    };

    // Initialize the BQ25798
    Serial.print("Initializing BQ25798... ");
    bq25798_status_t status = bq25798_init(&bq, BQ25798_I2C_ADDR, io);

    if (status != BQ25798_OK) {
        Serial.println("FAILED!");
        Serial.print("Error: ");
        Serial.println(bq25798_stat_error(status));
        while(1) delay(10);
    }
    Serial.println("SUCCESS!");

    // Test basic configuration
    Serial.println("\n--- Testing Basic Configuration ---");

    // Test minimum system voltage
    Serial.print("Setting VSYSMIN to ");
    Serial.print(TEST_VSYSMIN);
    Serial.print("mV... ");
    status = bq25798_set_vsysmin(&bq, TEST_VSYSMIN);
    if (status == BQ25798_OK) {
        int vsysmin;
        bq25798_get_vsysmin(&bq, &vsysmin);
        Serial.print("OK (Read back: ");
        Serial.print(vsysmin);
        Serial.println("mV)");
    } else {
        Serial.println("FAILED");
    }

    // Test charge voltage limit
    Serial.print("Setting charge voltage limit to ");
    Serial.print(TEST_CHARGE_VOLTAGE);
    Serial.print("mV... ");
    status = bq25798_set_charge_limit_v(&bq, TEST_CHARGE_VOLTAGE);
    if (status == BQ25798_OK) {
        int vreg;
        bq25798_get_charge_limit_v(&bq, &vreg);
        Serial.print("OK (Read back: ");
        Serial.print(vreg);
        Serial.println("mV)");
    } else {
        Serial.println("FAILED");
    }

    // Test charge current limit
    Serial.print("Setting charge current limit to ");
    Serial.print(TEST_CHARGE_CURRENT);
    Serial.print("mA... ");
    status = bq25798_set_charge_limit_i(&bq, TEST_CHARGE_CURRENT);
    if (status == BQ25798_OK) {
        int ireg;
        bq25798_get_charge_limit_i(&bq, &ireg);
        Serial.print("OK (Read back: ");
        Serial.print(ireg);
        Serial.println("mA)");
    } else {
        Serial.println("FAILED");
    }

    // Test input voltage limit
    Serial.print("Setting input voltage limit to ");
    Serial.print(TEST_INPUT_VOLTAGE);
    Serial.print("mV... ");
    status = bq25798_set_input_limit_v(&bq, TEST_INPUT_VOLTAGE);
    if (status == BQ25798_OK) {
        int vin;
        bq25798_get_input_limit_v(&bq, &vin);
        Serial.print("OK (Read back: ");
        Serial.print(vin);
        Serial.println("mV)");
    } else {
        Serial.println("FAILED");
    }

    // Test input current limit
    Serial.print("Setting input current limit to ");
    Serial.print(TEST_INPUT_CURRENT);
    Serial.print("mA... ");
    status = bq25798_set_input_limit_i(&bq, TEST_INPUT_CURRENT);
    if (status == BQ25798_OK) {
        int iin;
        bq25798_get_input_limit_i(&bq, &iin);
        Serial.print("OK (Read back: ");
        Serial.print(iin);
        Serial.println("mA)");
    } else {
        Serial.println("FAILED");
    }

    // Test precharge current
    Serial.print("Setting precharge current to ");
    Serial.print(TEST_PRECHARGE_CURRENT);
    Serial.print("mA... ");
    status = bq25798_set_precharge_lim_i(&bq, TEST_PRECHARGE_CURRENT);
    if (status == BQ25798_OK) {
        int ipre;
        bq25798_get_precharge_lim_i(&bq, &ipre);
        Serial.print("OK (Read back: ");
        Serial.print(ipre);
        Serial.println("mA)");
    } else {
        Serial.println("FAILED");
    }

    // Test termination current
    Serial.print("Setting termination current to ");
    Serial.print(TEST_TERM_CURRENT);
    Serial.print("mA... ");
    status = bq25798_set_termination_curr(&bq, TEST_TERM_CURRENT);
    if (status == BQ25798_OK) {
        int iterm;
        bq25798_get_termination_curr(&bq, &iterm);
        Serial.print("OK (Read back: ");
        Serial.print(iterm);
        Serial.println("mA)");
    } else {
        Serial.println("FAILED");
    }

    // Test charging enable
    Serial.print("Enabling charging... ");
    status = bq25798_set_charge_enable(&bq, true);
    if (status == BQ25798_OK) {
        bool en_chg;
        bq25798_get_charge_enable(&bq, &en_chg);
        Serial.print("OK (Enabled: ");
        Serial.print(en_chg ? "YES" : "NO");
        Serial.println(")");
    } else {
        Serial.println("FAILED");
    }

    // Test cell count configuration
    Serial.print("Setting cell count to 1S... ");
    status = bq25798_set_cell_count(&bq, BQ25798_CELL_COUNT_1S);
    if (status == BQ25798_OK) {
        bq25798_cell_count_t cells;
        bq25798_get_cell_count(&bq, &cells);
        Serial.print("OK (Cell count: ");
        Serial.print(cells + 1);
        Serial.println("S)");
    } else {
        Serial.println("FAILED");
    }

    // Enable ADC for measurements
    Serial.print("Enabling ADC... ");
    status = bq25798_set_adc_enable(&bq, true);
    if (status == BQ25798_OK) {
        Serial.println("OK");
    } else {
        Serial.println("FAILED");
    }

    // Test MPPT configuration
    Serial.print("Enabling MPPT... ");
    status = bq25798_set_mppt_enable(&bq, true);
    if (status == BQ25798_OK) {
        Serial.println("OK");
    } else {
        Serial.println("FAILED");
    }

    Serial.print("Setting VOC percentage to 75%... ");
    status = bq25798_set_voc_pct(&bq, BQ25798_VOC_PCT_75);
    if (status == BQ25798_OK) {
        Serial.println("OK");
    } else {
        Serial.println("FAILED");
    }

    Serial.println("\n--- Configuration Complete ---");
    Serial.println("Starting continuous monitoring...\n");
    delay(1000);
}

void loop() {
    static unsigned long lastPrint = 0;
    static unsigned long lastStatusCheck = 0;

    // Print measurements every 2 seconds
    if (millis() - lastPrint >= 2000) {
        lastPrint = millis();

        Serial.println("=== BQ25798 Measurements ===");

        // Read all ADC measurements
        bq25798_status_t status = bq25798_get_adc_mesurements(&bq, &measurements);
        if (status == BQ25798_OK) {
            Serial.print("IBUS: ");
            Serial.print(measurements.ibus);
            Serial.println(" mA");

            Serial.print("IBAT: ");
            Serial.print(measurements.ibat);
            Serial.println(" mA");

            Serial.print("VBUS: ");
            Serial.print(measurements.vbus);
            Serial.println(" mV");

            Serial.print("VBAT: ");
            Serial.print(measurements.vbat);
            Serial.println(" mV");

            Serial.print("VSYS: ");
            Serial.print(measurements.vsys);
            Serial.println(" mV");

            Serial.print("VAC1: ");
            Serial.print(measurements.vac1);
            Serial.println(" mV");

            Serial.print("VAC2: ");
            Serial.print(measurements.vac2);
            Serial.println(" mV");

            Serial.print("TS: ");
            Serial.print(measurements.ts, 2);
            Serial.println(" %");

            Serial.print("TDIE: ");
            Serial.print(measurements.tdie, 1);
            Serial.println(" °C");

            Serial.print("D+: ");
            Serial.print(measurements.dp);
            Serial.println(" mV");

            Serial.print("D-: ");
            Serial.print(measurements.dm);
            Serial.println(" mV");
        } else {
            Serial.print("ADC read failed: ");
            Serial.println(bq25798_stat_error(status));
        }

        Serial.println();
    }

    // Check status every 5 seconds
    if (millis() - lastStatusCheck >= 5000) {
        lastStatusCheck = millis();

        Serial.println("=== BQ25798 Status ===");

        // Read charger status
        bq25798_status_t status = bq25798_check_charger_status(&bq, &charger_status);
        if (status == BQ25798_OK) {
            // Charger state
            Serial.print("Charge Status: ");
            switch(charger_status.chg_stat) {
                case BQ25798_CHG_STAT_NOT_CHARGING:
                    Serial.println("Not Charging");
                    break;
                case BQ25798_CHG_STAT_TRICKLE_CHARGE:
                    Serial.println("Trickle Charge");
                    break;
                case BQ25798_CHG_STAT_PRE_CHARGE:
                    Serial.println("Pre-charge");
                    break;
                case BQ25798_CHG_STAT_FAST_CHARGE:
                    Serial.println("Fast Charge (CC)");
                    break;
                case BQ25798_CHG_STAT_TAPER_CHARGE:
                    Serial.println("Taper Charge (CV)");
                    break;
                case BQ25798_CHG_STAT_TOP_OFF:
                    Serial.println("Top-off Timer Active");
                    break;
                case BQ25798_CHG_STAT_CHARGE_TERM:
                    Serial.println("Charge Termination Done");
                    break;
                default:
                    Serial.println("Unknown");
                    break;
            }

            // VBUS status
            Serial.print("VBUS Status: ");
            switch(charger_status.vbus_stat) {
                case BQ25798_VBUS_STAT_NO_INPUT:
                    Serial.println("No Input");
                    break;
                case BQ25798_VBUS_STAT_USB_SDP:
                    Serial.println("USB SDP (500mA)");
                    break;
                case BQ25798_VBUS_STAT_USB_CDP:
                    Serial.println("USB CDP (1.5A)");
                    break;
                case BQ25798_VBUS_STAT_USB_DCP:
                    Serial.println("USB DCP (3.25A)");
                    break;
                case BQ25798_VBUS_STAT_HVDCP:
                    Serial.println("HVDCP (1.5A)");
                    break;
                case BQ25798_VBUS_STAT_UNKNOWN:
                    Serial.println("Unknown Adapter (3A)");
                    break;
                case BQ25798_VBUS_STAT_NON_STANDARD:
                    Serial.println("Non-standard Adapter");
                    break;
                case BQ25798_VBUS_STAT_BACKUP_MODE:
                    Serial.println("Backup Mode");
                    break;
                default:
                    Serial.println("Other");
                    break;
            }

            // ICO status
            Serial.print("ICO Status: ");
            switch(charger_status.ico_stat) {
                case BQ25798_ICO_STAT_DISABLE:
                    Serial.println("Disabled");
                    break;
                case BQ25798_ICO_STAT_RUNNING:
                    Serial.println("Running");
                    break;
                case BQ25798_ICO_STAT_MAX_CURR:
                    Serial.println("Max Current Detected");
                    break;
                default:
                    Serial.println("Unknown");
                    break;
            }

            // Power status indicators
            Serial.print("Power Good: ");
            Serial.println(charger_status.pg_stat ? "YES" : "NO");

            Serial.print("VBUS Present: ");
            Serial.println(charger_status.vbus_present_stat ? "YES" : "NO");

            Serial.print("Battery Present: ");
            Serial.println(charger_status.vbat_present_stat ? "YES" : "NO");

            Serial.print("VINDPM Active: ");
            Serial.println(charger_status.vindpm_stat ? "YES" : "NO");

            Serial.print("IINDPM Active: ");
            Serial.println(charger_status.iindpm_stat ? "YES" : "NO");

            Serial.print("Thermal Regulation: ");
            Serial.println(charger_status.treg_stat ? "ACTIVE" : "INACTIVE");

            // Temperature status
            if (charger_status.ts_cold_stat) Serial.println("Temperature: COLD");
            else if (charger_status.ts_cool_stat) Serial.println("Temperature: COOL");
            else if (charger_status.ts_warm_stat) Serial.println("Temperature: WARM");
            else if (charger_status.ts_hot_stat) Serial.println("Temperature: HOT");
            else Serial.println("Temperature: NORMAL");

        } else {
            Serial.print("Status read failed: ");
            Serial.println(bq25798_stat_error(status));
        }

        // Check for faults
        status = bq25798_check_fault_status(&bq, &fault_status);
        if (status == BQ25798_OK) {
            bool faultsDetected = false;

            if (fault_status.vbus_ovp_stat) {
                Serial.println("FAULT: VBUS Overvoltage");
                faultsDetected = true;
            }
            if (fault_status.vbat_ovp_stat) {
                Serial.println("FAULT: VBAT Overvoltage");
                faultsDetected = true;
            }
            if (fault_status.ibus_ocp_stat) {
                Serial.println("FAULT: IBUS Overcurrent");
                faultsDetected = true;
            }
            if (fault_status.ibat_ocp_stat) {
                Serial.println("FAULT: IBAT Overcurrent");
                faultsDetected = true;
            }
            if (fault_status.conv_ocp_stat) {
                Serial.println("FAULT: Converter Overcurrent");
                faultsDetected = true;
            }
            if (fault_status.vac1_ovp_stat) {
                Serial.println("FAULT: VAC1 Overvoltage");
                faultsDetected = true;
            }
            if (fault_status.vac2_ovp_stat) {
                Serial.println("FAULT: VAC2 Overvoltage");
                faultsDetected = true;
            }
            if (fault_status.vsys_short_stat) {
                Serial.println("FAULT: VSYS Short Circuit");
                faultsDetected = true;
            }
            if (fault_status.vsys_ovp_stat) {
                Serial.println("FAULT: VSYS Overvoltage");
                faultsDetected = true;
            }
            if (fault_status.otg_ovp_stat) {
                Serial.println("FAULT: OTG Overvoltage");
                faultsDetected = true;
            }
            if (fault_status.otg_uvp_stat) {
                Serial.println("FAULT: OTG Undervoltage");
                faultsDetected = true;
            }
            if (fault_status.tshut_stat) {
                Serial.println("FAULT: Thermal Shutdown");
                faultsDetected = true;
            }

            if (!faultsDetected) {
                Serial.println("No Faults Detected");
            }
        }

        Serial.println("================================");
        Serial.println();
    }

    delay(100); // Small delay to prevent overwhelming the serial output
}