/**
 * @file bq76905.c
 * @brief BQ76905 Battery Monitor and Protector Driver Implementation
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-08-14
 */

#include <stddef.h>
#include <math.h>
#include "bq76905.h"
#include "bq76905_platform.h"

///< @section Registers
///< @subsection Direct Command Registers
#define BQ76905_REG_SAFETY_ALERT_A 0x02     ///< Provide individual alert signals when enabled safety alerts have triggered
#define BQ76905_REG_SAFETY_STATUS_A 0x03    ///< Provide individual fault signals when enabled safety alerts have triggered
#define BQ76905_REG_SAFETY_ALERT_B 0x04     ///< Provide individual alert signals when enabled safety alerts have triggered
#define BQ76905_REG_SAFETY_STATUS_B 0x05    ///< Provide individual fault signals when enabled safety alerts have triggered
#define BQ76905_REG_BATTERY_STATUS 0x12     ///< Provide flags related to battery status
#define BQ76905_REG_CELL_1_VOLTAGE 0x14     ///< 16-bit voltage on cell 1
#define BQ76905_REG_CELL_2_VOLTAGE 0x16     ///< 16-bit voltage on cell 2
#define BQ76905_REG_CELL_3_VOLTAGE 0x18     ///< 16-bit voltage on cell 3
#define BQ76905_REG_CELL_4_VOLTAGE 0x1A     ///< 16-bit voltage on cell 4
#define BQ76905_REG_CELL_5_VOLTAGE 0x1C     ///< 16-bit voltage on cell 5
#define BQ76905_REG_REG18_VOLTAGE 0x22      ///< Internal 1.8V regulator voltage measured using bandgap reference
#define BQ76905_REG_VSS_VOLTAGE 0x24        ///< Measurement of VSS using ADC
#define BQ76905_REG_STACK_VOLTAGE 0x26      ///< 16-bit voltage on top of stack
#define BQ76905_REG_INT_TEMPERATURE 0x28    ///< This is the most recent measured internal die temperature
#define BQ76905_REG_TS_MEASUREMENT 0x2A     ///< ADC measurement of the TS pin
#define BQ76905_REG_RAW_CURRENT 0x36        ///< 32-bit raw current measurement
#define BQ76905_REG_CURRENT 0x3A            ///< 16-bit CC2 current measurement
#define BQ76905_REG_CC1_CURRENT 0x3C        ///< 16-bit CC1 current measurement
#define BQ76905_REG_ALARM_STATUS 0x62       ///< Latched signal used to assert the ALERT pin
#define BQ76905_REG_ALARM_RAW_STATUS 0x64   ///< Unlatched value of flags which can be selected to be latched
#define BQ76905_REG_ALARM_ENABLE 0x66       ///< Mask for Alarm Status()
#define BQ76905_REG_FET_CONTROL 0x68        ///< FET Control: Allows host control of individual FET drivers
#define BQ76905_REG_REGOUT_CONTROL 0x69     ///< REGOUT Control: Changes voltage regulator setting
#define BQ76905_REG_DSG_FET_PWM_CTRL 0x6A   ///< Controls the PWM mode of the DSG FET driver
#define BQ76905_REG_CHG_GET_PWM_CTRL 0x6C   ///< Controls the PWM mode of the CHG FET driver

///< @subsection Command-Only Subcommand Registers
#define BQ76905_REG_RESET_PASSQ 0x0005      ///< Resets the accumulated charge and timer
#define BQ76905_REG_EXIT_DEEPSLEEP 0x000E   ///< Exit DEEPSLEEP mode
#define BQ76905_REG_DEEPSLEEP 0x000F        ///< Enter DEEPSLEEP mode
#define BQ76905_REG_SHUTDOWN 0x0010         ///< Start SHUTDOWN sequence
#define BQ76905_REG_RESET 0x0012            ///< Reset the device
#define BQ76905_REG_FET_ENABLE 0x0022       ///< Toggle the FET_EN bit in Battery Status()
#define BQ76905_REG_SEAL 0x0030             ///< Place the device in SEALED mode
#define BQ76905_REG_SET_CFGUPDATE 0x0090    ///< Place the device in CONFIG_UPDATE mode
#define BQ76905_REG_EXIT_CFGUPDATE 0x0092   ///< Exit CONFIG_UPDATE mode
#define BQ76905_REG_SLEEP_ENABLE 0x0099     ///< Allow device to enter SLEEP mode
#define BQ76905_REG_SLEEP_DISABLE 0x009A    ///< Block device from entering SLEEP mode

///< @subsection Subcommands with Data Registers
#define BQ76905_REG_DEVICE_NUMBER 0x0001    ///< Device number that identifies the product
#define BQ76905_REG_FW_VERSION 0x0002       ///< Returns device number, firmware version, and build number
#define BQ76905_REG_HW_VERSION 0x0003       ///< Returns the device hardware version number
#define BQ76905_REG_PASSQ 0x0004            ///< Returns accumulated charge and time
#define BQ76905_REG_SECURITY_KEYS 0x0035    ///< Security keys to transfer from SEALED to FULLACCESS
#define BQ76905_REG_CB_ACTIVE_CELLS 0x0083  ///< Cell balancing active cells
#define BQ76905_REG_PROG_TIMER 0x0094       ///< Programmable timer for REGOUT LDO disable/waken
#define BQ76905_REG_PROT_RECOVERY 0x009B    ///< Enables the host to allow recovery of selected protection faults

///< @subsection Data Memory Table Addresses
#define BQ76905_REG_CELL_1_GAIN 0x9000                          ///< Cell 1 Gain
#define BQ76905_REG_STACK_GAIN 0x9002                           ///< Battery Stack Gain
#define BQ76905_REG_CELL_2_GAIN 0x9004                          ///< Cell 2 Gain Delta
#define BQ76905_REG_CELL_3_GAIN 0x9005                          ///< Cell 3 Gain Delta
#define BQ76905_REG_CELL_4_GAIN 0x9071                          ///< Cell 4 Gain Delta
#define BQ76905_REG_CELL_5_GAIN 0x9072                          ///< Cell 5 Gain Delta
#define BQ76905_REG_CURR_GAIN 0x9006                            ///< Current Gain
#define BQ76905_REG_CURR_OFFSET 0x9008                          ///< Current Offset
#define BQ76905_REG_CC1_GAIN 0x900A                             ///< CC1 Gain
#define BQ76905_REG_CC1_OFFSET 0x900C                           ///< CC1 Offset
#define BQ76905_REG_TS_OFFSET 0x900E                            ///< TS Offset
#define BQ76905_REG_INT_TEMP_GAIN 0x9010                        ///< Internal Temperature Gain
#define BQ76905_REG_INT_TEMP_OFFSET 0x9012                      ///< Internal Temperature Offset
#define BQ76905_REG_POWER_CONFIG 0x9014                         ///< Power Config
#define BQ76905_REG_REGOUT_CONFIG 0x9015                        ///< REGOUT Config
#define BQ76905_REG_I2C_ADDRESS 0x9016                          ///< I2C Address
#define BQ76905_REG_I2C_CONFIG 0x9017                           ///< I2C Config
#define BQ76905_REG_DA_CONFIG 0x9019                            ///< DA Config
#define BQ76905_REG_VCELL_MODE 0x901B                           ///< Vcell Mode
#define BQ76905_REG_DEFAULT_ALARM_MASK 0x901C                   ///< Default Alarm Mask
#define BQ76905_REG_FET_OPTIONS 0x901E                          ///< FET Options
#define BQ76905_REG_CHARGE_DETECTOR_TIME 0x901F                 ///< Charge Detector Time
#define BQ76905_REG_BALANCING_CONFIG 0x9020                     ///< Balancing Configuration
#define BQ76905_REG_MIN_TEMP_THRESHOLD 0x9021                   ///< Min Temp Threshold
#define BQ76905_REG_MAX_TEMP_THRESHOLD 0x9022                   ///< Max Temp Threshold
#define BQ76905_REG_MAX_INTERNAL_TEMP 0x9023                    ///< Max Internal Temp
#define BQ76905_REG_ENABLED_PROTECTIONS_A 0x9024                ///< Enabled Protections A
#define BQ76905_REG_ENABLED_PROTECTIONS_B 0x9025                ///< Enabled Protections B
#define BQ76905_REG_DSG_FET_PROTECTIONS_A 0x9026                ///< DSG FET Protections A
#define BQ76905_REG_CHG_FET_PROTECTIONS_A 0x9027                ///< CHG FET Protections A
#define BQ76905_REG_BOTH_FET_PROTECTIONS_B 0x9028               ///< Both FET Protections B
#define BQ76905_REG_BODY_DIODE_THRESHOLD 0x9029                 ///< Body Diode Threshold
#define BQ76905_REG_CELL_OPEN_WIRE_NORMAL_TIME 0x902B           ///< Cell Open Wire Normal Check Time
#define BQ76905_REG_CELL_OPEN_WIRE_SLEEP_TIME 0x902C            ///< Cell Open Wire Sleep Check Time
#define BQ76905_REG_HOST_WATCHDOG_TIMEOUT 0x902D                ///< Host Watchdog Timeout
#define BQ76905_REG_CELL_UNDERVOLTAGE_THRESHOLD 0x902E          ///< Cell Undervoltage Protection Threshold
#define BQ76905_REG_CELL_UNDERVOLTAGE_DELAY 0x9030              ///< Cell Undervoltage Protection Delay
#define BQ76905_REG_CELL_UNDERVOLTAGE_RECOVERY 0x9031           ///< Cell Undervoltage Recovery Hysteresis
#define BQ76905_REG_CELL_OVERVOLTAGE_THRESHOLD 0x9032           ///< Cell Overvoltage Protection Threshold
#define BQ76905_REG_CELL_OVERVOLTAGE_DELAY 0x9034               ///< Cell Overvoltage Protection Delay
#define BQ76905_REG_CELL_OVERVOLTAGE_RECOVERY 0x9035            ///< Cell Overvoltage Recovery Hysteresis
#define BQ76905_REG_OVERCURRENT_CHARGE_THRESHOLD 0x9036         ///< Overcurrent in Charge Protection Threshold
#define BQ76905_REG_OVERCURRENT_CHARGE_DELAY 0x9037             ///< Overcurrent in Charge Protection Delay
#define BQ76905_REG_OVERCURRENT_DISCHARGE_1_THRESHOLD 0x9038    ///< Overcurrent in Discharge 1 Protection Threshold
#define BQ76905_REG_OVERCURRENT_DISCHARGE_1_DELAY 0x9039        ///< Overcurrent in Discharge 1 Protection Delay
#define BQ76905_REG_OVERCURRENT_DISCHARGE_2_THRESHOLD 0x903A    ///< Overcurrent in Discharge 2 Protection Threshold
#define BQ76905_REG_OVERCURRENT_DISCHARGE_2_DELAY 0x903B        ///< Overcurrent in Discharge 2 Protection Delay
#define BQ76905_REG_SHORT_CIRCUIT_DISCHARGE_THRESHOLD 0x903C    ///< Short Circuit in Discharge Protection Threshold
#define BQ76905_REG_SHORT_CIRCUIT_DISCHARGE_DELAY 0x903D        ///< Short Circuit in Discharge Protection Delay
#define BQ76905_REG_LATCH_LIMIT 0x903E                          ///< Latch Limit
#define BQ76905_REG_RECOVERY_TIME 0x903F                        ///< Recovery Time
#define BQ76905_REG_OVERTEMP_CHARGE_THRESHOLD 0x9040            ///< Overtemperature in Charge Protection Threshold
#define BQ76905_REG_OVERTEMP_CHARGE_DELAY 0x9041                ///< Overtemperature in Charge Protection Delay
#define BQ76905_REG_OVERTEMP_CHARGE_RECOVERY 0x9042             ///< Overtemperature in Charge Protection Recovery
#define BQ76905_REG_UNDERTEMP_CHARGE_THRESHOLD 0x9043           ///< Undertemperature in Charge Protection Threshold
#define BQ76905_REG_UNDERTEMP_CHARGE_DELAY 0x9044               ///< Undertemperature in Charge Protection Delay
#define BQ76905_REG_UNDERTEMP_CHARGE_RECOVERY 0x9045            ///< Undertemperature in Charge Protection Recovery
#define BQ76905_REG_OVERTEMP_DISCHARGE_THRESHOLD 0x9046         ///< Overtemperature in Discharge Protection Threshold
#define BQ76905_REG_OVERTEMP_DISCHARGE_DELAY 0x9047             ///< Overtemperature in Discharge Protection Delay
#define BQ76905_REG_OVERTEMP_DISCHARGE_RECOVERY 0x9048          ///< Overtemperature in Discharge Protection Recovery
#define BQ76905_REG_UNDERTEMP_DISCHARGE_THRESHOLD 0x9049        ///< Undertemperature in Discharge Protection Threshold
#define BQ76905_REG_UNDERTEMP_DISCHARGE_DELAY 0x904A            ///< Undertemperature in Discharge Protection Delay
#define BQ76905_REG_UNDERTEMP_DISCHARGE_RECOVERY 0x904B         ///< Undertemperature in Discharge Protection Recovery
#define BQ76905_REG_INTERNAL_OVERTEMP_THRESHOLD 0x904C          ///< Internal Overtemperature Protection Threshold
#define BQ76905_REG_INTERNAL_OVERTEMP_DELAY 0x904D              ///< Internal Overtemperature Protection Delay
#define BQ76905_REG_INTERNAL_OVERTEMP_RECOVERY 0x904E           ///< Internal Overtemperature Protection Recovery
#define BQ76905_REG_SLEEP_CURRENT 0x904F                        ///< Sleep Current
#define BQ76905_REG_VOLTAGE_TIME 0x9051                         ///< Voltage Time
#define BQ76905_REG_WAKE_COMPARATOR_CURRENT 0x9052              ///< Wake Comparator Current
#define BQ76905_REG_SHUTDOWN_CELL_VOLTAGE 0x9053                ///< Shutdown Cell Voltage
#define BQ76905_REG_SHUTDOWN_STACK_VOLTAGE 0x9055               ///< Shutdown Stack Voltage
#define BQ76905_REG_SHUTDOWN_TEMPERATURE 0x9057                 ///< Shutdown Temperature
#define BQ76905_REG_AUTO_SHUTDOWN_TIME 0x9058                   ///< Auto Shutdown Time
#define BQ76905_REG_SECURITY_SETTINGS 0x9059                    ///< Security Settings
#define BQ76905_REG_FULL_ACCESS_KEY_STEP1 0x905A                ///< Full Access Key Step 1
#define BQ76905_REG_FULL_ACCESS_KEY_STEP2 0x905C                ///< Full Access Key Step 2

///< @section Register Bit Masks
///< @subsection CB_ACTIVE_CELLS Bit Masks
#define BQ76905_ACTIVE_CELLS_5  (1 << 5)
#define BQ76905_ACTIVE_CELLS_4  (1 << 4)
#define BQ76905_ACTIVE_CELLS_3  (1 << 3)
#define BQ76905_ACTIVE_CELLS_2  (1 << 2)
#define BQ76905_ACTIVE_CELLS_1  (1 << 1)

static uint8_t bq76905_crc(const uint8_t *data, const uint8_t len) {
    if (!data || !len) return 0;
    uint8_t crc = 0x00; // Initialize the CRC variable
    const uint8_t polynomial = 0x07;    // x^8 + x^2 + x + 1
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ polynomial : (crc << 1);
        }
    }

    return crc;
}

/**
 * @brief Helper function to write to a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Value to write to a register
 * @return
 */
static bq76905_status_t bq76905_write_register(const bq76905_t *dev, const uint8_t reg, const uint8_t value) {
    const uint8_t data[2] = {reg, value};
    return dev->io.i2c_write(dev->i2c_address, data, 2) == 0 ? BQ76905_OK : BQ76905_ERR_I2C;
}

/**
 * @brief Helper function to read data from a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param value Pointer to data variable
 * @return
 */
static bq76905_status_t bq76905_read_register(const bq76905_t *dev, const uint8_t reg, uint8_t *value) {
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return BQ76905_ERR_I2C;

    return dev->io.i2c_read(dev->i2c_address, value, 1) == 0 ? BQ76905_OK : BQ76905_ERR_I2C;
}

/**
 * @brief Helper function to write multiple bytes to a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param data Pointer to data array
 * @param length Length of data array
 * @return
 */
static bq76905_status_t bq76905_write_registers(const bq76905_t *dev, const uint8_t reg, const uint8_t *data, const size_t length) {
    uint8_t buffer[length + 1];
    buffer[0] = reg;
    for (size_t i = 0; i < length; i++) {
        buffer[i + 1] = data[i];
    }

    return dev->io.i2c_write(dev->i2c_address, buffer, length + 1) == 0 ? BQ76905_OK : BQ76905_ERR_I2C;
}

/**
 * @brief Helper function to read multiple bytes from a register
 * @param dev Pointer to driver handle
 * @param reg Target register
 * @param data Pointer to data array
 * @param length Length of data array
 * @return
 */
static bq76905_status_t bq76905_read_registers(const bq76905_t *dev, const uint8_t reg, uint8_t *data, const size_t length) {
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return BQ76905_ERR_I2C;

    return dev->io.i2c_read(dev->i2c_address, data, length) == 0 ? BQ76905_OK : BQ76905_ERR_I2C;
}

/**
 * @brief Helper function to insert bits into a register
 * @param reg Pointer to the register array
 * @param reg_len Length of the register array
 * @param bits Bits to insert into the register
 * @param bits_len Length of the bits to insert into the register
 * @param shift Number of bits to shift the data into the register
 * @return bq76905_status_t Status code
 */
static bq76905_status_t bq76905_register_bits(uint8_t *reg, const size_t reg_len, uint32_t bits, const uint32_t bits_len, const uint32_t shift) {
    if (!reg || !reg_len || !bits_len) return BQ76905_ERR_NULL;
    if (shift + bits_len > reg_len * 8) return BQ76905_ERR_INVALID_ARG;

    const uint32_t mask = (1U << bits_len) - 1;
    bits &= mask;

    for (uint32_t i = 0; i < bits_len; ++i) {
        const uint32_t bit_pos = shift + i;
        const uint32_t byte_index = bit_pos / 8;
        const uint8_t bit_index = bit_pos % 8;

        const uint8_t bit = (bits >> i) & 0x1;

        reg[byte_index] &= ~(1 << bit_index);
        reg[byte_index] |= (bit << bit_index);
    }

    return BQ76905_OK;
}

/**
 * @brief Convert status error messages to human-readable strings
 * @param status Status to be converted to a string
 * @return String from status
 */
const char* bq76905_stat_error(const bq76905_status_t status) {
    switch (status) {
        case BQ76905_OK:                return "OK";
        case BQ76905_ERR_I2C:           return "I2C communication failed";
        case BQ76905_ERR_TIMEOUT:       return "Timeout occurred";
        case BQ76905_ERR_NULL:          return "Null pointer";
        case BQ76905_ERR_INVALID_ARG:   return "Invalid argument";
        default:                        return "Unknown error";
    }
}

/**
 * @brief Initialize the BQ76905 driver
 * @param dev Pointer to driver handle
 * @param address I2C address (0 to use default)
 * @param io Interface structure with platform-specific functions
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq25798_init(bq76905_t *dev, const uint8_t address, const bq76905_interface_t io) {
    if (!dev || !io.i2c_write || ! io.i2c_read || !io.delay_ms) return BQ76905_ERR_NULL;

    dev->i2c_address = address ? address : BQ76905_I2C_DEFAULT_ADDRESS;
    dev->io = io;
    dev->initialized = false;

    const uint8_t write[2] = {BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_DEVICE_NUMBER};

    if (dev->io.i2c_write(dev->i2c_address, write, 1) != 0) return BQ76905_ERR_I2C; // Try to write subcommand to the command address

    const uint8_t reg = BQ76905_I2C_DATA_BUFFER_ADDRESS;
    if (dev->io.i2c_write(dev->i2c_address, &reg, 1) != 0) return BQ76905_ERR_I2C; // Try to read device number to check if I2C is working

    uint8_t deviceNumberBytes[2];
    io.i2c_read(dev->i2c_address, deviceNumberBytes, 2);

    const uint16_t deviceNumber = ((uint16_t)deviceNumberBytes[1] << 8) | deviceNumberBytes[0];

    if (deviceNumber != 0x7605) return BQ76905_ERR_I2C;

    dev->initialized = true;

    return BQ76905_OK;
}

/**
 * @brief Get alert signals from A
 * @param dev Pointer to driver handle
 * @param safety_alerts Pointer to safety alerts struct
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_safety_alert_A(bq76905_t *dev, bq76905_safety_alerts_t *safety_alerts) {
    if (!dev || !dev->initialized || !safety_alerts) return BQ76905_ERR_NULL;

    uint8_t data;
    const bq76905_status_t status = bq76905_read_register(dev, BQ76905_REG_SAFETY_ALERT_A, &data);
    if (status != BQ76905_OK) return status;

    safety_alerts->cov = (bool)((data >> 7) & 0x01);
    safety_alerts->cuv = (bool)((data >> 6) & 0x01);
    safety_alerts->scd = (bool)((data >> 5) & 0x01);
    safety_alerts->ocd1 = (bool)((data >> 4) & 0x01);
    safety_alerts->ocd2 = (bool)((data >> 3) & 0x01);
    safety_alerts->occ = (bool)((data >> 2) & 0x01);

    return BQ76905_OK;
}

/**
 * @brief Get fault signals from A
 * @param dev Pointer to driver handle
 * @param safety_faults Pointer to safety fault struct
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_safety_fault_A(bq76905_t *dev, bq76905_safety_faults_t *safety_faults) {
    if (!dev || !dev->initialized || !safety_faults) return BQ76905_ERR_NULL;

    uint8_t data;
    const bq76905_status_t status = bq76905_read_register(dev, BQ76905_REG_SAFETY_STATUS_A, &data);
    if (status != BQ76905_OK) return status;

    safety_faults->cov = (bool)((data >> 7) & 0x01);
    safety_faults->cuv = (bool)((data >> 6) & 0x01);
    safety_faults->scd = (bool)((data >> 5) & 0x01);
    safety_faults->ocd1 = (bool)((data >> 4) & 0x01);
    safety_faults->ocd2 = (bool)((data >> 3) & 0x01);
    safety_faults->occ = (bool)((data >> 2) & 0x01);

    return BQ76905_OK;
}

/**
 * @brief Get alert signals from B
 * @param dev Pointer to driver handle
 * @param safety_alerts Pointer to safety alerts struct
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_safety_alert_B(bq76905_t *dev, bq76905_safety_alerts_t *safety_alerts) {
    if (!dev || !dev->initialized || !safety_alerts) return BQ76905_ERR_NULL;

    uint8_t data;
    const bq76905_status_t status = bq76905_read_register(dev, BQ76905_REG_SAFETY_ALERT_A, &data);
    if (status != BQ76905_OK) return status;

    safety_alerts->otd = (bool)((data >> 7) & 0x01);
    safety_alerts->otc = (bool)((data >> 6) & 0x01);
    safety_alerts->utd = (bool)((data >> 5) & 0x01);
    safety_alerts->utc = (bool)((data >> 4) & 0x01);
    safety_alerts->otint = (bool)((data >> 3) & 0x01);
    safety_alerts->hwd = (bool)((data >> 2) & 0x01);
    safety_alerts->vref = (bool)((data >> 1) & 0x01);
    safety_alerts->vss = (bool)(data & 0x01);

    return BQ76905_OK;
}

/**
 * @brief Get fault signals from B
 * @param dev Pointer to driver handle
 * @param safety_faults Pointer to safety fault struct
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_safety_fault_B(bq76905_t *dev, bq76905_safety_faults_t *safety_faults) {
    if (!dev || !dev->initialized || !safety_faults) return BQ76905_ERR_NULL;

    uint8_t data;
    const bq76905_status_t status = bq76905_read_register(dev, BQ76905_REG_SAFETY_STATUS_B, &data);
    if (status != BQ76905_OK) return status;

    safety_faults->otd = (bool)((data >> 7) & 0x01);
    safety_faults->otc = (bool)((data >> 6) & 0x01);
    safety_faults->utd = (bool)((data >> 5) & 0x01);
    safety_faults->utc = (bool)((data >> 4) & 0x01);
    safety_faults->otint = (bool)((data >> 3) & 0x01);
    safety_faults->hwd = (bool)((data >> 2) & 0x01);
    safety_faults->vref = (bool)((data >> 1) & 0x01);
    safety_faults->vss = (bool)(data & 0x01);

    return BQ76905_OK;
}

/**
 * @brief Get battery status signals
 * @param dev Pointer to device handle
 * @param battery_status Pointer to battery status struct
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_battery_status(bq76905_t *dev, bq76905_battery_status_t *battery_status) {
    if (!dev || !dev->initialized || !battery_status) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_BATTERY_STATUS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    battery_status->sleep = (bool)((data[1] >> 7) & 0x01);
    battery_status->deepsleep = (bool)((data[1] >> 6) & 0x01);
    battery_status->sa = (bool)((data[1] >> 5) & 0x01);
    battery_status->ss = (bool)((data[1] >> 4) & 0x01);
    battery_status->sec = (bq76905_security_state_t)((data[1] >> 2) & 0x03);
    battery_status->fet_en = (bool)(data[1] & 0x01);
    battery_status->por = (bool)((data[0] >> 7) & 0x01);
    battery_status->sleep_en = (bool)((data[0] >> 6) & 0x01);
    battery_status->cfg_update = (bool)((data[0] >> 5) & 0x01);
    battery_status->alert_pin   = (bool)((data[0] >> 4) & 0x01);
    battery_status->chg = (bool)((data[0] >> 3) & 0x01);
    battery_status->dsg = (bool)((data[0] >> 2) & 0x01);
    battery_status->chg = (bool)(data[0] & 0x01);

    return BQ76905_OK;
}

/**
 * @brief Get the voltage measurement for cell 1 in the stack
 * @param dev Pointer to device handle
 * @param cell_1_voltage pointer to cell 1 voltage variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_cell_1_voltage(bq76905_t *dev, int16_t *cell_1_voltage) {
    if (!dev || !dev->initialized || !cell_1_voltage) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_CELL_1_VOLTAGE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *cell_1_voltage = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the voltage measurement for cell 2 in the stack
 * @param dev Pointer to device handle
 * @param cell_2_voltage pointer to cell 2 voltage variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_cell_2_voltage(bq76905_t *dev, int16_t *cell_2_voltage) {
    if (!dev || !dev->initialized || !cell_2_voltage) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_CELL_2_VOLTAGE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *cell_2_voltage = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the voltage measurement for cell 3 in the stack
 * @param dev Pointer to device handle
 * @param cell_3_voltage pointer to cell 3 voltage variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_cell_3_voltage(bq76905_t *dev, int16_t *cell_3_voltage) {
    if (!dev || !dev->initialized || !cell_3_voltage) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_CELL_3_VOLTAGE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *cell_3_voltage = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the voltage measurement for cell 4 in the stack
 * @param dev Pointer to device handle
 * @param cell_4_voltage pointer to cell 4 voltage variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_cell_4_voltage(bq76905_t *dev, int16_t *cell_4_voltage) {
    if (!dev || !dev->initialized || !cell_4_voltage) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_CELL_4_VOLTAGE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *cell_4_voltage = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the voltage measurement for cell 5 in the stack
 * @param dev Pointer to device handle
 * @param cell_5_voltage Pointer to cell 5 voltage variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_cell_5_voltage(bq76905_t *dev, int16_t *cell_5_voltage) {
    if (!dev || !dev->initialized || !cell_5_voltage) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_CELL_5_VOLTAGE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *cell_5_voltage = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the reference voltage for the IC
 * @param dev Pointer to device handle
 * @param reg18_voltage Pointer to the reference voltage variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_reg18_voltage(bq76905_t *dev, int16_t *reg18_voltage) {
    if (!dev || !dev->initialized || !reg18_voltage) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_REG18_VOLTAGE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *reg18_voltage = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the voltage of VSS
 * @param dev Pointer to device handle
 * @param vss_voltage Pointer to the VSS voltage variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_vss_voltage(bq76905_t *dev, int16_t *vss_voltage) {
    if (!dev || !dev->initialized || !vss_voltage) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_VSS_VOLTAGE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *vss_voltage = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the voltage of the battery stack voltage
 * @param dev Pointer to device handle
 * @param stack_voltage Pointer to the stack voltage variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_stack_voltage(bq76905_t *dev, uint16_t *stack_voltage) {
    if (!dev || !dev->initialized || !stack_voltage) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_STACK_VOLTAGE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *stack_voltage = ((uint16_t)data[1] << 8) | data[0];

    return BQ76905_OK;
}

/**
 * @brief Get the internal die temperature of the IC
 * @param dev Pointer to the device handle
 * @param internal_temp Pointer to the die temperature variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_ic_temperature(bq76905_t *dev, float *internal_temp) {
    if (!dev || !dev->initialized || !internal_temp) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_INT_TEMPERATURE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *internal_temp = (float)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the ADC value of the TS
 * @param dev Pointer to the device handle
 * @param ts_temp Pointer to the ts temperature variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_ts_temperature(bq76905_t *dev, int16_t *ts_temp) {
    if (!dev || !dev->initialized || !ts_temp) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_TS_MEASUREMENT, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *ts_temp = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the raw ADC current value
 * @param dev Pointer to the device handle
 * @param raw_current Pointer to the raw current variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_raw_current(bq76905_t *dev, int32_t *raw_current) {
    if (!dev || !dev->initialized || !raw_current) return BQ76905_ERR_NULL;

    uint8_t data[4];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_RAW_CURRENT, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *raw_current = ((int32_t)data[3] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[1] << 8) | ((int32_t)data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the current measurement from CC2
 * @param dev Pointer to the device handle
 * @param current Pointer to the current variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_current(bq76905_t *dev, int16_t *current) {
    if (!dev || !dev->initialized || !current) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_CURRENT, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *current = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Get the current measurement from CC1
 * @param dev Pointer to the device handle
 * @param cc1_current Pointer to the current variable
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_cc1_current(bq76905_t *dev, int16_t *cc1_current) {
    if (!dev || !dev->initialized || !cc1_current) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_CC1_CURRENT, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    *cc1_current = (int16_t)(((int16_t)data[1] << 8) | data[0]);

    return BQ76905_OK;
}

/**
 * @brief Read the alarm status registers
 * @param dev Pointer to the device handle
 * @param alarm_status Pointer to the alarm statuses
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_latched_alarms(bq76905_t *dev, bq76905_alarm_status_t *alarm_status) {
    if (!dev || !dev->initialized || !alarm_status) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_ALARM_STATUS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    alarm_status->ssa = (bool)((data[1] >> 7) & 0x01);
    alarm_status->ssb = (bool)((data[1] >> 6) & 0x01);
    alarm_status->saa = (bool)((data[1] >> 5) & 0x01);
    alarm_status->sab = (bool)((data[1] >> 4) & 0x01);
    alarm_status->xchg = (bool)((data[1] >> 3) & 0x01);
    alarm_status->xdsg = (bool)((data[1] >> 2) & 0x01);
    alarm_status->shutv = (bool)((data[1] >> 1) & 0x01);
    alarm_status->cb = (bool)(data[1] & 0x01);
    alarm_status->fullscan = (bool)((data[0] >> 7) & 0x01);
    alarm_status->adscan = (bool)((data[0] >> 6) & 0x01);
    alarm_status->wake = (bool)((data[0] >> 5) & 0x01);
    alarm_status->sleep = (bool)((data[0] >> 4) & 0x01);
    alarm_status->timer_alarm = (bool)((data[0] >> 3) & 0x01);
    alarm_status->init_comp = (bool)((data[0] >> 2) & 0x01);
    alarm_status->cd_toggle = (bool)((data[0] >> 1) & 0x01);
    alarm_status->por = (bool)(data[0] & 0x01);

    return BQ76905_OK;
}

/**
 * @brief Clear a user-specified alarm
 * @param dev Pointer to the device handle
 * @param alarm Pointer to the alarm statuses
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_clear_alarm(bq76905_t *dev, const bq76905_alarms_t alarm) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    uint8_t data[2];
    bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_ALARM_STATUS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    status = bq76905_register_bits(data, sizeof(data), 1, 1, alarm);
    if (status != BQ76905_OK) return status;

    return BQ76905_OK;
}

/**
 * @brief Check all the alarms
 * @param dev Pointer to the device handle
 * @param alarm_status Pointer to the alarm statuses
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_check_alarms(bq76905_t *dev, bq76905_alarm_status_t *alarm_status) {
    if (!dev || !dev->initialized || !alarm_status) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_ALARM_RAW_STATUS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    alarm_status->ssa = (bool)((data[1] >> 7) & 0x01);
    alarm_status->ssb = (bool)((data[1] >> 6) & 0x01);
    alarm_status->saa = (bool)((data[1] >> 5) & 0x01);
    alarm_status->sab = (bool)((data[1] >> 4) & 0x01);
    alarm_status->xchg = (bool)((data[1] >> 3) & 0x01);
    alarm_status->xdsg = (bool)((data[1] >> 2) & 0x01);
    alarm_status->shutv = (bool)((data[1] >> 1) & 0x01);
    alarm_status->cb = (bool)(data[1] & 0x01);
    alarm_status->fullscan = (bool)((data[0] >> 7) & 0x01);
    alarm_status->adscan = (bool)((data[0] >> 6) & 0x01);
    alarm_status->wake = (bool)((data[0] >> 5) & 0x01);
    alarm_status->sleep = (bool)((data[0] >> 4) & 0x01);
    alarm_status->timer_alarm = (bool)((data[0] >> 3) & 0x01);
    alarm_status->init_comp = (bool)((data[0] >> 2) & 0x01);
    alarm_status->cd_toggle = (bool)((data[0] >> 1) & 0x01);
    alarm_status->por = (bool)(data[0] & 0x01);

    return BQ76905_OK;
}

/**
 * @brief Get the alarms that are enabled to latch
 * @param dev Pointer to the device handle
 * @param alarm Pointer to enabled alarm statuses
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_enabled_alarms(bq76905_t *dev, bq76905_alarm_enable_t *alarm) {
    if (!dev || !dev->initialized || !alarm) return BQ76905_ERR_NULL;

    uint8_t data[2];
    const bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_ALARM_ENABLE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    alarm->ssa = (bool)((data[1] >> 7) & 0x01);
    alarm->ssb = (bool)((data[1] >> 6) & 0x01);
    alarm->saa = (bool)((data[1] >> 5) & 0x01);
    alarm->sab = (bool)((data[1] >> 4) & 0x01);
    alarm->xchg = (bool)((data[1] >> 3) & 0x01);
    alarm->xdsg = (bool)((data[1] >> 2) & 0x01);
    alarm->shutv = (bool)((data[1] >> 1) & 0x01);
    alarm->cb = (bool)(data[1] & 0x01);
    alarm->fullscan = (bool)((data[0] >> 7) & 0x01);
    alarm->adscan = (bool)((data[0] >> 6) & 0x01);
    alarm->wake = (bool)((data[0] >> 5) & 0x01);
    alarm->sleep = (bool)((data[0] >> 4) & 0x01);
    alarm->timer_alarm = (bool)((data[0] >> 3) & 0x01);
    alarm->init_comp = (bool)((data[0] >> 2) & 0x01);
    alarm->cd_toggle = (bool)((data[0] >> 1) & 0x01);
    alarm->por = (bool)(data[0] & 0x01);

    return BQ76905_OK;
}

/**
 * @brief Enable a user-specified alarm
 * @param dev Pointer to the device handle
 * @param alarm Alarm to be enabled
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_enable_alarm(bq76905_t *dev, const bq76905_alarms_t alarm) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    uint8_t data[2];
    bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_ALARM_ENABLE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    status = bq76905_register_bits(data, sizeof(data), 1, 1, alarm);
    if (status != BQ76905_OK) return status;

    return BQ76905_OK;
}

/**
 * @brief Enable a user-specified alarm
 * @param dev Pointer to the device handle
 * @param alarm Alarm to be enabled
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_disable_alarm(bq76905_t *dev, const bq76905_alarms_t alarm) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    uint8_t data[2];
    bq76905_status_t status = bq76905_read_registers(dev, BQ76905_REG_ALARM_ENABLE, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    status = bq76905_register_bits(data, sizeof(data), 0, 1, alarm);
    if (status != BQ76905_OK) return status;

    return BQ76905_OK;
}

///< @section Command-Only Subcommands

/**
 * Reset the accumulated charge and timer
 * @param dev Pointer to driver handle
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_reset_passq(bq76905_t *dev) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    return bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_RESET_PASSQ);
}

/**
 * @brief Exit DEEPSLEEP mode
 * @param dev Pointer to driver handle
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_exit_deepsleep(bq76905_t *dev) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    return bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_EXIT_DEEPSLEEP);
}

/**
 * @brief Enter DEEPSLEEP mode
 * @param dev Pointer to driver handle
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_enter_deepsleep(bq76905_t *dev) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    const bq76905_status_t status = bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_DEEPSLEEP);
    if (status != BQ76905_OK) return status;

    platform_delay_ms(1000);

    return bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_DEEPSLEEP); // Must send command twice within 4s to take effect
}

/**
 * @brief Enter SHUTDOWN mode
 * @param dev Pointer to driver handle
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_shutdown(bq76905_t *dev) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    const bq76905_status_t status = bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_SHUTDOWN);
    if (status != BQ76905_OK) return status;

    platform_delay_ms(1000);

    return bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_SHUTDOWN); // Must send command twice within 4s to take effect
}

/**
 * @brief Reset the device
 * @param dev Pointer to driver handle
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_reset(bq76905_t *dev) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    return bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_RESET);
}

/**
 * @brief Select between manual and autonomous device FET control
 * @details 0 - Manual FET control
 * @details 1 - Autonomous FET control
 * @param dev Pointer to driver handle
 * @param settings Pointer to settings struct
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_fet_enable(bq76905_t *dev, bq76905_settings_t *settings) {
    if (!dev || !dev->initialized || !settings) return BQ76905_ERR_NULL;

    const bq76905_status_t status = bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_FET_ENABLE);
    if (status != BQ76905_OK) return status;

    settings->fet_en = ! settings->fet_en;

    return BQ76905_OK;
}

/**
 * @brief Enter CONFIG_UPDATE mode
 * @param dev Pointer to driver handle
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_enter_cfg_update(bq76905_t *dev) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    return bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_SET_CFGUPDATE);
}

/**
 * @brief Exit CONFIG_UPDATE mode
 * @param dev Pointer to driver handle
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_exit_cfg_update(bq76905_t *dev) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    return bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_EXIT_CFGUPDATE);
}

/**
 * @brief Allow device to enter SLEEP mode
 * @param dev Pointer to driver handle
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_sleep_enable(bq76905_t *dev) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    return bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_SLEEP_ENABLE);
}

/**
 * @brief Block device from entering SLEEP mode
 * @param dev
 * @return
 */
bq76905_status_t bq76905_sleep_disable(bq76905_t *dev) {
    if (!dev || !dev->initialized) return BQ76905_ERR_NULL;

    return bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_SLEEP_DISABLE);
}

///< @section Subcommands with data

/**
 * @brief Get the device number that identifies the product
 * @param dev Pointer to driver handle
 * @param settings Pointer to settings structure
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_device_number(bq76905_t *dev, bq76905_settings_t *settings) {
    if (!dev || !dev->initialized || !settings) return BQ76905_ERR_NULL;

    bq76905_status_t status = bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_DEVICE_NUMBER);
    if (status != BQ76905_OK) return status;

    uint8_t data[3];

    status = bq76905_read_registers(dev, BQ76905_I2C_DATA_BUFFER_ADDRESS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    settings->device_number = (data[1] << 8) | data[2];

    return BQ76905_OK;
}

/**
 * @brief Get the device number and firmware version for the product
 * @param dev Pointer to driver handle
 * @param settings Pointer to settings structure
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_firmware_version(bq76905_t *dev, bq76905_settings_t *settings) {
    if (!dev || !dev->initialized || !settings) return BQ76905_ERR_NULL;

    bq76905_status_t status = bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_FW_VERSION);
    if (status != BQ76905_OK) return status;

    uint8_t data[7];

    status = bq76905_read_registers(dev, BQ76905_I2C_DATA_BUFFER_ADDRESS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    settings->device_number = (data[1] << 8) | data[2];
    settings->firmware_version = (data[3] << 8) | data[4];
    settings->build_number = (data[5] << 8) | data[6];

    return BQ76905_OK;
}

/**
 * @brief Get the device hardware version
 * @param dev Pointer to driver handle
 * @param settings Pointer to settings structure
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_hardware_version(bq76905_t *dev, bq76905_settings_t *settings) {
    if (!dev || !dev->initialized || !settings) return BQ76905_ERR_NULL;

    bq76905_status_t status = bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_HW_VERSION);
    if (status != BQ76905_OK) return status;

    uint8_t data[3];

    status = bq76905_read_registers(dev, BQ76905_I2C_DATA_BUFFER_ADDRESS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    settings->device_number = (data[1] << 8) | data[2];

    return BQ76905_OK;
}

/**
 * @brief Get the accumulated charge and time
 * @param dev Pointer to driver handle
 * @param settings Pointer to settings structure
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_passq(bq76905_t *dev, bq76905_settings_t *settings) {
    if (!dev || !dev->initialized || !settings) return BQ76905_ERR_NULL;

    bq76905_status_t status = bq76905_write_register(dev, BQ76905_I2C_COMMAND_ADDRESS, BQ76905_REG_PASSQ);
    if (status != BQ76905_OK) return status;

    uint8_t data[13];

    status = bq76905_read_registers(dev, BQ76905_I2C_DATA_BUFFER_ADDRESS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    const uint32_t lsb = (data[4] << 24) | (data[3] << 16) | (data[2] << 8) | data[1];
    const int32_t msb = (data[8] << 24) | (data[7] << 16) | (data[6] << 8) | data[5];

    settings->passq = ((int64_t)msb << 32) | lsb;
    settings->passtime = (data[12] << 24) | (data[11] << 16) | (data[10] << 8) | data[9];

    return BQ76905_OK;
}

/**
 * @brief Get how which cells are being actively balanced
 * @param dev Pointer to driver handle
 * @param active_cells Pointer to active_cells struct
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq7605_get_active_cells(bq76905_t *dev, bq76905_active_cells_t *active_cells) {
    if (!dev || !dev->initialized || !active_cells) return BQ76905_ERR_NULL;

    uint8_t data[2];
    data[0] = BQ76905_REG_CB_ACTIVE_CELLS;
    data[1] = 0x00;

    bq76905_status_t status = bq76905_write_registers(dev, BQ76905_I2C_COMMAND_ADDRESS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    uint8_t activeCellsByte;

    status = bq76905_read_register(dev, BQ76905_I2C_DATA_BUFFER_ADDRESS, &activeCellsByte);
    if (status != BQ76905_OK) return status;

    active_cells->cell_5 = (activeCellsByte & BQ76905_ACTIVE_CELLS_5) ? 1 : 0;
    active_cells->cell_4 = (activeCellsByte & BQ76905_ACTIVE_CELLS_4) ? 1 : 0;
    active_cells->cell_3 = (activeCellsByte & BQ76905_ACTIVE_CELLS_3) ? 1 : 0;
    active_cells->cell_2 = (activeCellsByte & BQ76905_ACTIVE_CELLS_2) ? 1 : 0;
    active_cells->cell_1 = (activeCellsByte & BQ76905_ACTIVE_CELLS_1) ? 1 : 0;

    return BQ76905_OK;
}

/**
 * @brief Set which cells should be balanced
 * @param dev Pointer to driver handle
 * @param active_cells Pointer to active_cells structure
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_set_active_cells(bq76905_t *dev, bq76905_active_cells_t *active_cells){
    if (!dev || !dev->initialized || !active_cells) return BQ76905_ERR_NULL;

    uint8_t activeCellsByte = 0;

    if (active_cells->cell_5) activeCellsByte |= BQ76905_ACTIVE_CELLS_5;
    if (active_cells->cell_4) activeCellsByte |= BQ76905_ACTIVE_CELLS_4;
    if (active_cells->cell_3) activeCellsByte |= BQ76905_ACTIVE_CELLS_3;
    if (active_cells->cell_2) activeCellsByte |= BQ76905_ACTIVE_CELLS_2;
    if (active_cells->cell_1) activeCellsByte |= BQ76905_ACTIVE_CELLS_1;

    uint8_t data[3];
    data[0] = BQ76905_REG_CB_ACTIVE_CELLS;
    data[1] = 0x00;
    data[2] = activeCellsByte;

    const bq76905_status_t status = bq76905_write_registers(dev, BQ76905_I2C_COMMAND_ADDRESS, data, sizeof(data));
    if (status != BQ76905_OK) return status;

    uint8_t crc[2];
    crc[0] = bq76905_crc(data, 3);
    crc[1] = 0x05;

    return bq76905_write_registers(dev, 0x60, crc, sizeof(crc));
}

/**
 * @brief Update all measurements in struct
 * @param dev Pointer to driver handle
 * @param measurements Pointer to measurements struct
 * @return bq76905_status_t Status code
 */
bq76905_status_t bq76905_get_measurements(bq76905_t *dev, bq76905_measurements_t *measurements) {
    if (!dev || !dev->initialized || !measurements) return BQ76905_ERR_NULL;

    bq76905_status_t status = bq76905_get_cell_1_voltage(dev, &measurements->cell_1_voltage);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_cell_2_voltage(dev, &measurements->cell_2_voltage);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_cell_3_voltage(dev, &measurements->cell_3_voltage);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_cell_4_voltage(dev, &measurements->cell_4_voltage);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_cell_5_voltage(dev, &measurements->cell_5_voltage);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_reg18_voltage(dev, &measurements->reg18_voltage);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_vss_voltage(dev, &measurements->vss_voltage);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_stack_voltage(dev, &measurements->stack_voltage);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_ic_temperature(dev, &measurements->internal_temp);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_ts_temperature(dev, &measurements->ts_temp);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_raw_current(dev, &measurements->raw_current);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_current(dev, &measurements->current);
    if (status != BQ76905_OK) return status;

    status = bq76905_get_cc1_current(dev, &measurements->cc1_current);
    if (status != BQ76905_OK) return status;

    return BQ76905_OK;
}