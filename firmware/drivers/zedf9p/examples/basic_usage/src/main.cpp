/*!
 * @file main.cpp
 * @brief Enhanced Arduino example for ZEDF9P GNSS module
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-09-10
 *
 * This example demonstrates the enhanced UBX protocol features including:
 * - Robust message handling with retry logic
 * - Improved I2C communication with bytes available checking
 * - Enhanced CFG-VALGET/VALSET operations
 * - Better error handling and status reporting
 */

#include <Arduino.h>
#include <Wire.h>
#include <zedf9p.h>
#include <zedf9p_platform.h>
//#include <cinttypes>

zedf9p_t gnss_module;
zedf9p_nav_pvt_t pvt_data;        // For PVT data
zedf9p_nav_hpposllh_t hppos_data; // For high precision position
zedf9p_rawx_t rawx_data;          // For raw measurements
zedf9p_mon_ver_t version_info;    // For version information

// Callback function for PVT messages
void pvt_callback(const ubx_message_t *message, void *user_data) {
    Serial.println("PVT message received via callback!");
}

// Generic callback for monitoring all messages
void generic_callback(const ubx_message_t *message, void *user_data) {
    char buffer[128];
    sprintf(buffer, "Message received: Class=0x%02X, ID=0x%02X, Length=%d",
           message->msg_class, message->msg_id, message->length);
    Serial.println(buffer);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    delay(100);

    while(!Serial) delay(10);

    Serial.println("Enhanced ZEDF9P Driver Example");
    Serial.println("===============================");

    // Define the enhanced IO interface
    constexpr zedf9p_interface_t io = {
        .i2c_write = platform_i2c_write,
        .i2c_read = platform_i2c_read,
        .uart_write = platform_uart_write,
        .uart_read = platform_uart_read,
        .delay_ms = platform_delay_ms,
        .get_millis = platform_get_millis
    };

    // Initialize the GNSS module with I2C interface
    zedf9p_status_t status = zedf9p_init(&gnss_module, ZEDF9P_INTERFACE_I2C,
                                        ZEDF9P_I2C_ADDR, io);

    if (status != ZEDF9P_OK) {
        Serial.print("Failed to initialize ZEDF9P: ");
        Serial.println(zedf9p_status_error(status));
        while (true) delay(10);
    }

    Serial.println("ZEDF9P initialized successfully!");

    // Register callbacks for enhanced message handling
    zedf9p_register_pvt_callback(&gnss_module, pvt_callback, NULL);
    zedf9p_register_generic_callback(&gnss_module, generic_callback, NULL);

    // Poll for version information using enhanced polling
    Serial.println("Polling for version information...");
    ubx_message_t ver_response;
    status = zedf9p_poll_ubx_message(&gnss_module, UBX_CLASS_MON, UBX_MON_VER,
                                    5000, &ver_response);

    if (status == ZEDF9P_OK) {
        Serial.println("Version poll successful!");
        // Process the data to get version info
        zedf9p_process_data(&gnss_module);

        if (zedf9p_is_mon_ver_available(&gnss_module)) {
            zedf9p_get_mon_ver(&gnss_module, &version_info);
            Serial.print("Software Version: ");
            Serial.println(version_info.sw_version);
            Serial.print("Hardware Version: ");
            Serial.println(version_info.hw_version);
        }
    } else {
        Serial.print("Version poll failed: ");
        Serial.println(zedf9p_status_error(status));
    }

    // Configure measurement rate using enhanced CFG-VALSET
    Serial.println("Setting measurement rate to 2Hz...");
    status = zedf9p_set_measurement_rate(&gnss_module, UBLOX_CFG_LAYER_RAM, 500, 1); // 500ms = 2Hz
    if (status == ZEDF9P_OK) {
        Serial.println("Measurement rate set successfully!");
    } else {
        Serial.print("Failed to set measurement rate: ");
        Serial.println(zedf9p_status_error(status));
    }

    // Enable PVT messages using enhanced message configuration
    Serial.println("Enabling NAV-PVT messages...");
    status = zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_PVT, 1);
    if (status == ZEDF9P_OK) {
        Serial.println("NAV-PVT messages enabled!");
    } else {
        Serial.print("Failed to enable NAV-PVT: ");
        Serial.println(zedf9p_status_error(status));
    }

    // Test enhanced CFG-VALGET functionality
    Serial.println("Reading current measurement rate...");
    uint64_t current_rate = 0;
    status = zedf9p_config_get_val(&gnss_module, UBLOX_CFG_LAYER_RAM, UBLOX_CFG_RATE_MEAS, &current_rate, 2);
    if (status == ZEDF9P_OK) {
        Serial.print("Current measurement rate: ");
        Serial.print(static_cast<uint16_t>(current_rate));
        Serial.println(" ms");
    } else {
        Serial.print("Failed to read measurement rate: ");
        Serial.println(zedf9p_status_error(status));
    }

    // Configure dynamic model
    Serial.println("Setting dynamic model to stationary...");
    status = zedf9p_set_dynamic_model(&gnss_module, UBLOX_CFG_LAYER_RAM, DYN_MODEL_STATIONARY);
    if (status == ZEDF9P_OK) {
        Serial.println("Dynamic model set to stationary!");
    } else {
        Serial.print("Failed to set dynamic model: ");
        Serial.println(zedf9p_status_error(status));
    }

    Serial.println("Starting enhanced data acquisition...");
    Serial.println("====================================");
}

void loop() {
    // Enhanced data processing with better error handling
    zedf9p_status_t status = zedf9p_process_data(&gnss_module);

    if (status != ZEDF9P_OK && status != ZEDF9P_ERR_NO_DATA) {
        Serial.print("Data processing error: ");
        Serial.println(zedf9p_status_error(status));
    }

    // Check for new PVT data
    if (zedf9p_is_pvt_available(&gnss_module)) {
        status = zedf9p_get_pvt(&gnss_module, &pvt_data);

        if (status == ZEDF9P_OK) {
            // Enhanced data display with more information
            char buffer[256];

            // Convert coordinates to decimal degrees
            const double lat_deg = static_cast<double>(pvt_data.lat) / 1e7;
            const double lon_deg = static_cast<double>(pvt_data.lon) / 1e7;
            const double height_m = static_cast<double>(pvt_data.height) / 1000.0;

            sprintf(buffer, "PVT Data: Lat=%.7f°, Lon=%.7f°, Height=%.3fm, "
                           "Fix=%d, NumSV=%d, HAcc=%lumm, VAcc=%lumm",
                    lat_deg, lon_deg, height_m, pvt_data.fix_type,
                    pvt_data.num_sv, pvt_data.h_acc, pvt_data.v_acc);
            Serial.println(buffer);

            // Display time information
            sprintf(buffer, "Time: %04d-%02d-%02d %02d:%02d:%02d UTC",
                    pvt_data.year, pvt_data.month, pvt_data.day,
                    pvt_data.hour, pvt_data.min, pvt_data.sec);
            Serial.println(buffer);

            // Display velocity information
            sprintf(buffer, "Velocity: N=%.3fm/s, E=%.3fm/s, D=%.3fm/s, Speed=%.3fm/s",
                    static_cast<double>(pvt_data.vel_n) / 1000.0, static_cast<double>(pvt_data.vel_e) / 1000.0,
                    static_cast<double>(pvt_data.vel_d) / 1000.0, static_cast<double>(pvt_data.g_speed) / 1000.0);
            Serial.println(buffer);

            Serial.println("---");
        } else {
            Serial.print("Failed to get PVT data: ");
            Serial.println(zedf9p_status_error(status));
        }
    }

    // Check for high precision position data
    if (zedf9p_is_hpposllh_available(&gnss_module)) {
        status = zedf9p_get_hpposllh(&gnss_module, &hppos_data);

        if (status == ZEDF9P_OK) {
            Serial.println("High precision position data available!");

            // Convert to high precision coordinates
            const double lat_hp = static_cast<double>(hppos_data.lat) / 1e7 + static_cast<double>(hppos_data.lat_hp) / 1e9;
            const double lon_hp = static_cast<double>(hppos_data.lon) / 1e7 + static_cast<double>(hppos_data.lon_hp) / 1e9;
            const double height_hp = static_cast<double>(hppos_data.height) / 1000.0 + static_cast<double>(hppos_data.height_hp) / 10000.0;

            char buffer[256];
            sprintf(buffer, "HP Position: Lat=%.9f°, Lon=%.9f°, Height=%.4fm, "
                           "HAcc=%.1fmm, VAcc=%.1fmm",
                    lat_hp, lon_hp, height_hp,
                    static_cast<double>(hppos_data.h_acc) / 10.0, static_cast<double>(hppos_data.v_acc) / 10.0);
            Serial.println(buffer);
        }
    }

    // Check for RAWX data (raw measurements)
    if (zedf9p_is_rawx_available(&gnss_module)) {
        status = zedf9p_get_rawx(&gnss_module, &rawx_data);

        if (status == ZEDF9P_OK) {
            char buffer[128];
            sprintf(buffer, "RAWX Data: Week=%d, TOW=%.3fs, NumMeas=%d",
                    rawx_data.week, rawx_data.rc_tow, rawx_data.num_meas);
            Serial.println(buffer);

            // Display first few measurements
            for (int i = 0; i < min(3, rawx_data.num_meas); i++) {
                sprintf(buffer, "  Meas[%d]: GNSS=%d, SV=%d, CNO=%ddB",
                        i, rawx_data.meas[i].gnss_id, rawx_data.meas[i].sv_id,
                        rawx_data.meas[i].cno);
                Serial.println(buffer);
            }
        }
    }

    // Demonstrate enhanced error handling with retry
    static uint32_t last_config_test = 0;
    if (millis() - last_config_test > 30000) { // Every 30 seconds
        last_config_test = millis();

        Serial.println("Testing enhanced configuration read...");
        uint64_t nav_rate = 0;
        status = zedf9p_config_get_val(&gnss_module, UBLOX_CFG_LAYER_RAM,UBLOX_CFG_RATE_NAV, &nav_rate, 2);

        if (status == ZEDF9P_OK) {
            Serial.print("Navigation rate: ");
            Serial.println(static_cast<uint16_t>(nav_rate));
        } else {
            Serial.print("Config read failed: ");
            Serial.println(zedf9p_status_error(status));
        }
    }

    delay(100); // Small delay to prevent overwhelming the module
}

// Additional helper functions for enhanced usage

void configure_rtk_base_station() {
    Serial.println("Configuring as RTK base station...");

    // Start survey-in mode
    zedf9p_status_t status = zedf9p_start_survey_in(&gnss_module, UBLOX_CFG_LAYER_RAM,300, 2000); // 5min, 2mm accuracy
    if (status == ZEDF9P_OK) {
        Serial.println("Survey-in started successfully!");
    } else {
        Serial.print("Survey-in start failed: ");
        Serial.println(zedf9p_status_error(status));
        return;
    }

    // Configure RTCM messages for base station
    status = zedf9p_config_rtcm_base_station(&gnss_module, UBLOX_CFG_LAYER_RAM,
                                           true,  // RTCM 1005 (station coordinates)
                                           true,  // RTCM 1077 (GPS MSM7)
                                           true,  // RTCM 1087 (GLONASS MSM7)
                                           true,  // RTCM 1097 (Galileo MSM7)
                                           false, // RTCM 1127 (BeiDou MSM7)
                                           true); // RTCM 1230 (GLONASS code-phase biases)

    if (status == ZEDF9P_OK) {
        Serial.println("RTCM base station configured successfully!");
    } else {
        Serial.print("RTCM configuration failed: ");
        Serial.println(zedf9p_status_error(status));
    }
}

void configure_high_precision_rover() {
    Serial.println("Configuring for high precision rover...");

    // Enable high precision position messages
    zedf9p_status_t status = zedf9p_set_message_rate(&gnss_module, UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, 1);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to enable HPPOSLLH: ");
        Serial.println(zedf9p_status_error(status));
        return;
    }

    // Enable RAWX messages for advanced processing
    status = zedf9p_set_message_rate(&gnss_module, UBX_CLASS_RXM, UBX_RXM_RAWX, 1);
    if (status != ZEDF9P_OK) {
        Serial.print("Failed to enable RAWX: ");
        Serial.println(zedf9p_status_error(status));
        return;
    }

    // Configure GNSS constellation
    constexpr zedf9p_gnss_config_t gnss_config = {
        .gps_enabled = true,
        .gps_l1ca_enabled = true,
        .gps_l2c_enabled = true,
        .sbas_enabled = true,
        .sbas_l1ca_enabled = true,
        .galileo_enabled = true,
        .galileo_e1_enabled = true,
        .galileo_e5b_enabled = true,
        .beidou_enabled = false, // Disable for this example
        .qzss_enabled = false,
        .glonass_enabled = true,
        .glonass_l1_enabled = true,
        .glonass_l2_enabled = true,
    };

    status = zedf9p_config_gnss_signals(&gnss_module, UBLOX_CFG_LAYER_RAM, &gnss_config);
    if (status == ZEDF9P_OK) {
        Serial.println("High precision rover configured successfully!");
    } else {
        Serial.print("GNSS configuration failed: ");
        Serial.println(zedf9p_status_error(status));
    }
}

void demonstrate_power_management() {
    Serial.println("Configuring power management...");

    constexpr zedf9p_power_config_t power_config = {
        .operate_mode = PM_OPERATEMODE_BALANCED,
        .pos_update_period_ms = 1000,
        .acq_period_ms = 2000,
        .grid_offset_ms = 0,
        .on_time_ms = 1000,
        .min_acq_time_ms = 1000,
        .max_acq_time_ms = 10000
    };

    const zedf9p_status_t status = zedf9p_config_power_management(&gnss_module, UBLOX_CFG_LAYER_RAM,&power_config);
    if (status == ZEDF9P_OK) {
        Serial.println("Power management configured successfully!");
    } else {
        Serial.print("Power management configuration failed: ");
        Serial.println(zedf9p_status_error(status));
    }
}

void demonstrate_antenna_configuration() {
    Serial.println("Configuring antenna settings...");

    constexpr zedf9p_antenna_config_t ant_config = {
        .voltage_ctrl = true,
        .short_det = true,
        .short_det_pol = false,
        .open_det = true,
        .open_det_pol = false,
        .power_down = false,
        .power_down_pol = false,
        .recover = true,
        .switch_pin = 0,
        .short_thr = 15,
        .open_thr = 45
    };

    const zedf9p_status_t status = zedf9p_config_antenna(&gnss_module, UBLOX_CFG_LAYER_RAM,&ant_config);
    if (status == ZEDF9P_OK) {
        Serial.println("Antenna configuration successful!");
    } else {
        Serial.print("Antenna configuration failed: ");
        Serial.println(zedf9p_status_error(status));
    }
}