/**
 * @file bq25798.h
 * @brief BQ25798 MPPT Battery Driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2025-07-19
 * 
 * This driver supports the BQ25798 IC from Texas Instruments
 */

#ifndef BQ25798_H
#define BQ25798_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define BQ25798_I2C_ADDR 0x6B // Default I2C address

/**
 * @brief BQ25798 IC driver return status codes.
 */
typedef enum {
    BQ25798_OK = 0,
    BQ25798_ERR_I2C = -1,
    BQ25798_ERR_TIMEOUT = -2,
    BQ25798_ERR_NULL = -3,
    BQ25798_ERR_INVALID_ARG = -4,
} bq25798_status_t;

/**
 * @brief Human-readable description of an BQ25798 status code.
 *
 * @param status The status code to translate.
 * @return A string describing the error.
 */
const char* bq25798_stat_error(bq25798_status_t status);

/**
 * @brief Battery voltage threshold for percentage to fast charge transistion
 */
typedef enum {
    BQ25798_VBAT_LOWV_15_PERCENT = 0x00,    ///< 15% of VREG
    BQ25798_VBAT_LOWV_62_2_PERCENT = 0x01,  ///< 62.5% of VREG
    BQ25798_VBAT_LOWV_66_7_PERCENT = 0x02,  ///< 66.7% of VREG
    BQ25798_VBAT_LOWV_71_4_PERCENT = 0x03   ///< 71.4% of VREG
} bq25798_vbat_lowv_t;

/**
 * @brief Battery cell count selection
 */
typedef enum {
    BQ25798_CELL_COUNT_1S = 0x00,   ///< 1 cell
    BQ25798_CELL_COUNT_2S = 0x01,   ///< 2 cells
    BQ25798_CELL_COUNT_3S = 0x02,   ///< 3 cells
    BQ25798_CELL_COUNT_4S = 0x03    ///< 4 cells
} bq25798_cell_count_t;

/**
 * @brief Battery recharge deglitch time
 */
typedef enum {
  BQ25798_TRECHG_64MS = 0x00,   ///< 64ms
  BQ25798_TRECHG_256MS = 0x01,  ///< 256ms
  BQ25798_TRECHG_1024MS = 0x02, ///< 1024ms (default)
  BQ25798_TRECHG_2048MS = 0x03  ///< 2048ms
} bq25798_trechg_time_t;

/**
 * @brief Precharge safety timer setting
 */
typedef enum {
  BQ25798_PRECHG_TMR_2HR = 0x00, ///< 2 hours (default)
  BQ25798_PRECHG_TMR_0_5HR = 0x01 ///< 0.5 hours
} bq25798_prechg_timer_t;

/**
 * @brief Top-off timer control
 */
typedef enum {
  BQ25798_TOPOFF_TMR_DISABLED = 0x00, ///< Disabled (default)
  BQ25798_TOPOFF_TMR_15MIN = 0x01,    ///< 15 minutes
  BQ25798_TOPOFF_TMR_30MIN = 0x02,    ///< 30 minutes
  BQ25798_TOPOFF_TMR_45MIN = 0x03     ///< 45 minutes
} bq25798_topoff_timer_t;

/**
 * @brief Fast charge timer setting
 */
typedef enum {
  BQ25798_CHG_TMR_5HR = 0x00,  ///< 5 hours
  BQ25798_CHG_TMR_8HR = 0x01,  ///< 8 hours
  BQ25798_CHG_TMR_12HR = 0x02, ///< 12 hours (default)
  BQ25798_CHG_TMR_24HR = 0x03  ///< 24 hours
} bq25798_chg_timer_t;

/**
 * @brief Backup mode threshold setting (percentage of VINDPM)
 */
typedef enum {
  BQ25798_VBUS_BACKUP_40_PERCENT = 0x00, ///< 40% of VINDPM
  BQ25798_VBUS_BACKUP_60_PERCENT = 0x01, ///< 60% of VINDPM
  BQ25798_VBUS_BACKUP_80_PERCENT = 0x02, ///< 80% of VINDPM (default)
  BQ25798_VBUS_BACKUP_100_PERCENT = 0x03 ///< 100% of VINDPM
} bq25798_vbus_backup_t;

/**
 * @brief VAC overvoltage protection setting
 */
typedef enum {
  BQ25798_VAC_OVP_26V = 0x00,  ///< 26V
  BQ25798_VAC_OVP_22V = 0x01,  ///< 22V
  BQ25798_VAC_OVP_12V = 0x02,  ///< 12V
  BQ25798_VAC_OVP_7V = 0x03    ///< 7V (default)
} bq25798_vac_ovp_t;

/**
 * @brief Watchdog timer setting
 */
typedef enum {
  BQ25798_WDT_DISABLE = 0x00, ///< Disable watchdog
  BQ25798_WDT_0_5S = 0x01,    ///< 0.5 seconds
  BQ25798_WDT_1S = 0x02,      ///< 1 second
  BQ25798_WDT_2S = 0x03,      ///< 2 seconds
  BQ25798_WDT_20S = 0x04,     ///< 20 seconds
  BQ25798_WDT_40S = 0x05,     ///< 40 seconds (default)
  BQ25798_WDT_80S = 0x06,     ///< 80 seconds
  BQ25798_WDT_160S = 0x07     ///< 160 seconds
} bq25798_wdt_t;

/**
 * @brief Ship FET mode control setting
 */
typedef enum {
  BQ25798_SDRV_IDLE = 0x00,             ///< IDLE (default)
  BQ25798_SDRV_SHUTDOWN = 0x01,         ///< Shutdown Mode
  BQ25798_SDRV_SHIP = 0x02,             ///< Ship Mode
  BQ25798_SDRV_SYSTEM_RESET = 0x03      ///< System Power Reset
} bq25798_sdrv_ctrl_t;

/**
 * @brief Ship mode wakeup delay setting
 */
typedef enum {
  BQ25798_WKUP_DLY_1S = 0x00,   ///< 1 second (default)
  BQ25798_WKUP_DLY_15MS = 0x01  ///< 15ms
} bq25798_wkup_dly_t;

/**
 * @brief PWM switching frequency setting
 */
typedef enum {
  BQ25798_PWM_FREQ_1_5MHZ = 0x00, ///< 1.5 MHz
  BQ25798_PWM_FREQ_750KHZ = 0x01  ///< 750 kHz
} bq25798_pwm_freq_t;

/**
 * @brief Battery discharge current regulation setting
 */
typedef enum {
  BQ25798_IBAT_REG_3A = 0x00,      ///< 3A
  BQ25798_IBAT_REG_4A = 0x01,      ///< 4A
  BQ25798_IBAT_REG_5A = 0x02,      ///< 5A
  BQ25798_IBAT_REG_DISABLE = 0x03  ///< Disable (default)
} bq25798_ibat_reg_t;

/**
 * @brief VINDPM VOC percentage setting
 */
typedef enum {
  BQ25798_VOC_PCT_56_25 = 0x00,    ///< 56.25% (0.5625)
  BQ25798_VOC_PCT_62_5 = 0x01,     ///< 62.5% (0.625)
  BQ25798_VOC_PCT_68_75 = 0x02,    ///< 68.75% (0.6875)
  BQ25798_VOC_PCT_75 = 0x03,       ///< 75% (0.75)
  BQ25798_VOC_PCT_81_25 = 0x04,    ///< 81.25% (0.8125)
  BQ25798_VOC_PCT_87_5 = 0x05,     ///< 87.5% (0.875) (default)
  BQ25798_VOC_PCT_93_75 = 0x06,    ///< 93.75% (0.9375)
  BQ25798_VOC_PCT_100 = 0x07       ///< 100% (1.0)
} bq25798_voc_pct_t;

/**
 * @brief VOC delay time setting
 */
typedef enum {
  BQ25798_VOC_DLY_50MS = 0x00,     ///< 50ms
  BQ25798_VOC_DLY_300MS = 0x01,    ///< 300ms (default)
  BQ25798_VOC_DLY_2S = 0x02,       ///< 2 seconds
  BQ25798_VOC_DLY_5S = 0x03        ///< 5 seconds
} bq25798_voc_dly_t;

/**
 * @brief VOC measurement rate setting
 */
typedef enum {
  BQ25798_VOC_RATE_30S = 0x00,     ///< 30 seconds
  BQ25798_VOC_RATE_2MIN = 0x01,    ///< 2 minutes (default)
  BQ25798_VOC_RATE_10MIN = 0x02,   ///< 10 minutes
  BQ25798_VOC_RATE_30MIN = 0x03    ///< 30 minutes
} bq25798_voc_rate_t;

/**
 * @brief Thermal regulation threshold setting
 */
typedef enum {
  BQ25798_TREG_60C = 0x00,         ///< 60°C
  BQ25798_TREG_80C = 0x01,         ///< 80°C
  BQ25798_TREG_100C = 0x02,        ///< 100°C
  BQ25798_TREG_120C = 0x03         ///< 120°C (default)
} bq25798_treg_t;

/**
 * @brief Thermal shutdown threshold setting
 */
typedef enum {
  BQ25798_TSHUT_150C = 0x00,       ///< 150°C (default)
  BQ25798_TSHUT_130C = 0x01,       ///< 130°C
  BQ25798_TSHUT_120C = 0x02,       ///< 120°C
  BQ25798_TSHUT_85C = 0x03         ///< 85°C
} bq25798_tshut_t;

/**
 * @brief JEITA high temperature range voltage setting
 */
typedef enum {
    BQ25798_JEITA_VSET_SUSP = 0x00,     ///< Charge Suspend
    BQ25798_JEITA_VSET_800mV = 0x01,    ///< Set VREG to VREG-800mV
    BQ25798_JEITA_VSET_600mV = 0x02,    ///< Set VREG to VREG-600mV
    BQ25798_JEITA_VSET_400mV = 0x03,    ///< Set VREG to VREG-400mV
    BQ25798_JEITA_VSET_300mV = 0x04,    ///< Set VREG to VREG-300mV
    BQ25798_JEITA_VSET_200mV = 0x05,    ///< Set VREG to VREG-200mV
    BQ25798_JEITA_VSET_100mV = 0x06,    ///< Set VREG to VREG-100mV
    BQ25798_JEITA_VSET_SAME = 0x07     ///< VREG unchanged
} bq25798_jeita_vset_t;

/**
 * @brief JEITA high temperature range current setting
 */
typedef enum {
    BQ25798_JEITA_ISETH_SUSP = 0x00,        ///< Charge Suspend
    BQ25798_JEITA_ISETH_20_PERCENT = 0x01,  ///< Set ICHG to 20% ICHG
    BQ25798_JEITA_ISETH_40_PERCENT = 0x02,  ///< Set ICHG to 40% ICHG
    BQ25798_JEITA_ISETH_SAME = 0x03         ///< ICHG unchanged
} bq25798_jeita_iseth_t;

/**
 * @brief JEITA low temperature range current setting
 */
typedef enum {
    BQ25798_JEITA_ISETL_SUSP = 0x00,        ///< Charge Suspend
    BQ25798_JEITA_ISETL_20_PERCENT = 0x01,  ///< Set ICHG to 20% ICHG
    BQ25798_JEITA_ISETL_40_PERCENT = 0x02,  ///< Set ICHG to 40% ICHG
    BQ25798_JEITA_ISETL_SAME = 0x03         ///< ICHG unchanged
} bq25798_jeita_isetc_t;

/**
 * @brief JEITA VT2 comparator voltage rising threshold setting
 */
typedef enum {
    BQ25798_TS_COOL_71_1_PERCENT = 0x00,  ///< 71.1% of REGN
    BQ25798_TS_COOL_68_4_PERCENT = 0x01,  ///< 68.4% of REGN
    BQ25798_TS_COOL_65_5_PERCENT = 0x02,  ///< 65.5% of REGN
    BQ25798_TS_COOL_62_4_PERCENT = 0x03  ///< 62.4% of REGN
} bq25798_ts_cool_t;

/**
 * @brief JEITA VT3 comparator voltage falling threshold setting
 */
typedef enum {
    BQ25798_TS_WARM_48_4_PERCENT = 0x00,  ///< 48.4% of REGN
    BQ25798_TS_WARM_44_8_PERCENT = 0x01,  ///< 44.8% of REGN
    BQ25798_TS_WARM_41_2_PERCENT = 0x02,  ///< 41.2% of REGN
    BQ25798_TS_WARM_37_7_PERCENT = 0x03  ///< 37.7% of REGN
} bq25798_ts_warm_t;

/**
 * @brief OTG mode TS HOT temperature threshold setting
 */
typedef enum {
    BQ25798_BHOT_55C = 0x00,    ///< 55°C
    BQ25798_BHOT_60C = 0x01,    ///< 60°C
    BQ25798_BHOT_65C = 0x02,    ///< 65°C
    BQ25798_BHOT_DIS = 0x03    ///< Disable
} bq25798_bhot_t;

/**
 * @brief OTG mode TS COLD temperature threshold setting
 */
typedef enum {
    BQ25798_BCOLD_N10C = 0x00,    ///< -10°C
    BQ25798_BCOLD_N20C = 0x01    ///< -20°C
} bq25798_bcold_t;

/**
 * @brief Charger status setting
 */
typedef enum {
    BQ25798_CHG_STAT_NOT_CHARGING = 0x00,   ///< Not charging
    BQ25798_CHG_STAT_TRICKLE_CHARGE = 0x01, ///< Trickle charge
    BQ25798_CHG_STAT_PRE_CHARGE = 0x02,     ///< Pre-charge
    BQ25798_CHG_STAT_FAST_CHARGE = 0x03,    ///< Fast charge (CC mode)
    BQ25798_CHG_STAT_TAPER_CHARGE = 0x04,   ///< Taper charge (CV mode)
    BQ25798_CHG_STAT_TOP_OFF = 0x06,        ///< Top-off timer active charging
    BQ25798_CHG_STAT_CHARGE_TERM = 0x07     ///< Charge termination done
} bq25798_chg_stat_t;

/**
 * @brief VBUS status setting
 */
typedef enum {
    BQ25798_VBUS_STAT_NO_INPUT = 0x00,      ///< No input or BHOT or BCOLD in OTG mode
    BQ25798_VBUS_STAT_USB_SDP = 0x01,       ///< USB SDP (500mA)
    BQ25798_VBUS_STAT_USB_CDP = 0x02,       ///< USB CDP (1.5A)
    BQ25798_VBUS_STAT_USB_DCP = 0x03,       ///< USB DCP (3.25A)
    BQ25798_VBUS_STAT_HVDCP = 0x04,         ///< Adjustable high voltage DCP (HVDCP) (1.5A)
    BQ25798_VBUS_STAT_UNKNOWN = 0x05,       ///< Unknown adapter (3A)
    BQ25798_VBUS_STAT_NON_STANDARD = 0x06,  ///< Non-standard adapter (1A/2A/2.1A/2.4A)
    BQ25798_VBUS_STAT_OTG_MODE = 0x07,      ///< In OTG mode
    BQ25798_VBUS_STAT_NON_QUAL = 0x08,      ///< Not qualified adapter
    BQ25798_VBUS_STAT_DIRECT_VBUS = 0x0B,   ///< Device directly powered from VBUS
    BQ25798_VBUS_STAT_BACKUP_MODE = 0x0C    ///< Backup mode
} bq25798_vbus_stat_t;

/**
 * @brief Input current optimizer (ICO) status setting
 */
typedef enum {
    BQ25798_ICO_STAT_DISABLE = 0x00,    ///< ICO disabled
    BQ25798_ICO_STAT_RUNNING = 0x01,    ///< ICO optimization in progress
    BQ25798_ICO_STAT_MAX_CURR = 0x02    ///< Maximum input current detected
} bq25798_ico_stat_t;

/**
 * @breif ADC sample speed settings
 */
typedef enum {
    BQ25798_ADC_15BIT = 0x00,   ///< ADC 15 bit effective resolution
    BQ25798_ADC_14BIT = 0x01,   ///< ADC 14 bit effective resolution
    BQ25798_ADC_13BIT = 0x02,   ///< ADC 13 bit effective resolution
    BQ25798_ADC_12BIT = 0x03,   ///< ADC 12 bit effective resolution
} bq25798_adc_speed_t;

/**
 * @brief Platform interface abstraction for BQ25798 driver.
 *
 * The user must provide these functions to enable hardware-specific I2C and timing.
 */
typedef struct {
    int (*i2c_write)(uint8_t dev_addr, const uint8_t *data, uint16_t len);
    int (*i2c_read)(uint8_t dev_addr, uint8_t *data, uint16_t len);
    void (*delay_ms)(uint32_t ms);
} bq25798_interface_t;

/**
 * @brief BQ25798 driver instance.
 */
typedef struct {
    uint8_t i2c_address;
    bq25798_interface_t io;
    bool initialized;

} bq25798_t;

/**
 * @brief BQ25798 general settings.
 */
typedef struct {
    int vsysmin;                        ///< Minimum system voltage (mV)
    int vreg_lim;                       ///< Charger voltage limit (mV)
    int ireg_lim;                       ///< Charger current limit (mA)
    int vin_lim;                        ///< Input voltage limit (mV)
    int iin_lim;                        ///< Input current limit (mA)
    bq25798_vbat_lowv_t lowv_threshold; ///< Lower threshold for battery fast charging
    int precharge_lim;                  ///< Precharge current limit (mA)
    bool stopOnWDT;                     ///< Watchdog timer expiration config
    int term_curr;                      ///< Termination current (mA)
    bq25798_cell_count_t cellCount;     ///< Battery cell count
    bq25798_trechg_time_t deglitchTime; ///< Battery recharge deglitch time
    int thresholdOffset;                ///< Battery recharge threshold offset (below VREG) (mV)
    int ico_ilim;                       ///< Input current limit from ICO/ILIM_HZ (mA)
} bq25798_settings_t;

/**
 * @brief BQ25798 charge control settings.
 */
typedef struct {
    bool auto_ibatdis;                      ///< Enable auto battery discharging durring battery OVP fault
    bool force_ibatdis;                     ///< Force a battery discharge current
    bool en_chg;                            ///< Enable charging
    bool en_ico;                            ///< Enable input current optimizer (ICO)
    bool force_ico;                         ///< Force start input current optimizer (ICO)
    bool en_hiz;                            ///< Enable high-impedance (HIZ) mode
    bool en_term;                           ///< Enable termination
    bool en_backup;                         ///< Enable backup mode
    bq25798_vbus_backup_t vbckup_threshold; ///< The threshold to trigger backup mode
    bq25798_vac_ovp_t vac_threshold;        ///< VAC_OVP threshold
    bool wdt_rst;                           ///< Reset watchdog timer
    bq25798_wdt_t wdt_time;                 ///< Watchdog timer setting
    bool en_12v;                            ///< Enable 12V HVDC
    bool en_9v;                             ///< Enable 9V HVDC
    bool hvdcp_en;                          ///< Enable high voltage DCP
    bq25798_sdrv_ctrl_t sfet_mode;          ///< SFET model control
    bool sdrv_dly;                          ///< Add 10s delay for SFET actions
    bool dis_acdrv;                         ///< Disconnect ACDRV1 & ACDRV2
    bool pfm_fwd_dis;                       ///< Disable PFM in forward mode
    bq25798_wkup_dly_t wkup_dly;            ///< Time to pull low the QON pin
    bool dis_ldo;                           ///< Disable BATFET LDO in pre-charge stage
    bool dis_fwd_ooa;                       ///< Disable OOA in forward mode
    bool en_acdrv2;                         ///< Enable ACDRV2
    bool en_acdrv1;                         ///< Enable ACDRV1
    bq25798_pwm_freq_t pwm_freq;            ///< Select switching frequency
    bool dis_stat;                          ///< Disable the STAT pin output
    bool dis_vsys_short;                    ///< Disable forward mode VSYS short hiccup protection
    bool force_vindpm_det;                  ///< Force VINDPM detection
    bool en_ibus_ocp;                       ///< Enable IBUS_OCP in forward mode
    bool sfet_present;                      ///< Define if ship FET is populated or not
    bool en_ibat;                           ///< IBAT discharge current sening
    bool en_iindpm;                         ///< Enable the internal IINDPM input current regulation
    bool en_extilim;                        ///< Enable the external ILIM_HIZ pin input current regulation
    bool en_batoc;                          ///< Enable the battery discharging current OCP
} bq25798_chg_ctrl_t;

/**
 * @brief BQ25798 MPPT control settings.
 */
typedef struct {
    bq25798_voc_pct_t voc_pct;      ///< Set VINDPM as a pertcentage of the VBUS open circuit voltage when VOC measurement is done
    bq25798_voc_dly_t voc_dly;      ///< Delay after converter stops switching before VOC is measured
    bq25798_voc_rate_t voc_rate;    ///< Time interval for VBUS open circuit voltage measurements
    bool en_mppt;                   ///< Enable the MPPT to measure the VBUS open circuit voltage
} bq25798_mppt_ctrl_t;

/**
 * @brief BQ25798 temperature control settings.
 */
typedef struct {
    bq25798_treg_t treg_threshold;      ///< Thermal regulation thresholds
    bq25798_tshut_t tshut_threshold;    ///< Thermal shutdown thresholds
    bool vbus_pd_en;                    ///< Enable VBUS pull down resistor (6k Ohm)
    bool vac1_pd_en;                    ///< Enable VAC1 pull down resistor
    bool vac2_pd_en;                    ///< Enable VAC2 pull down resistor
    bool bkup_acfet1_on;                ///< Turn on ACFET1 in backup mode
} bq25798_temp_ctrl_t;

/**
 * @brief BQ25798 NTC control settings.
 */
typedef struct {
    bq25798_jeita_vset_t jeita_vset;    ///< JEITA high temperature range voltage setting
    bq25798_jeita_iseth_t jeita_iseth;  ///< JEITA high temperature range current setting
    bq25798_jeita_isetc_t jeita_isetc;  ///< JEITA low temperature range current setting
    bq25798_ts_cool_t ts_cool;          ///< JEITA VT2 rising thershold
    bq25798_ts_warm_t ts_warm;          ///< JEITA VT3 falling threshold
    bq25798_bhot_t bhot;                ///< OTG mode TS HOT temperature threshold
    bq25798_bcold_t bcold;              ///< OTG mode TS COLD temperature threshold
    bool ts_ignore;                     ///< Ignore TS feedback
} bq25798_ntc_ctrl_t;

/**
 * @brief BQ25798 charger status registers.
 */
typedef struct {
    bool iindpm_stat;               ///< IINDPM status (forward mode) or IOTG status (OTG mode)
    bool vindpm_stat;               /**< VINDPM status (forward mode) or VOTG status (OTG mode) */
    bool watchdog_stat;             ///< I2C watchdog timer status
    bool pg_stat;                   ///< Power good status
    bool ac2_present_stat;          ///< VAC2 insert status
    bool ac1_present_stat;          ///< VAC1 insert status
    bool vbus_present_stat;         ///< VBUS present status
    bq25798_chg_stat_t chg_stat;    ///< Charger status bits
    bq25798_vbus_stat_t vbus_stat;  ///< VBUS status bits
    bool bc_1_2_done_stat;          ///< BC1.2 status bit
    bq25798_ico_stat_t ico_stat;    ///< Input current optimizer (ICO) status
    bool treg_stat;                 ///< IC thermal regulation status
    bool dpdm_stat;                 ///< D+/D- detection status bits
    bool vbat_present_stat;         ///< Battery present status (Vbat > Vbat_uvloz)
    bool acrb2_stat;                ///< ACFET2-RBFET2 status
    bool acrb1_stat;                ///< ACFET1-RBFET1 status
    bool adc_done_stat;             ///< ADC conversion status (in one-shot mode only)
    bool vsys_stat;                 ///< VSYS regulation status (forward mode)
    bool chg_tmr_stat;              ///< Fast charge timer status
    bool trichg_tmr_stat;           ///< Trickle charge timer status
    bool prechg_tmr_stat;           ///< Pre-charge timer status
    bool vbat_otg_low_stat;         ///< Battery voltage is too low to enable OTG mode
    bool ts_cold_stat;              ///< TS temperature is in the cold range, lower than T1
    bool ts_cool_stat;              ///< TS temperature is in the cool range, between T1 and T2
    bool ts_warm_stat;              ///< TS temperature is in the warm range, between T3 and T5
    bool ts_hot_stat;               ///< TS temperature is in the hot range, higher than T5
} bq25798_charger_status_t;

/**
 * @brief BQ25798 fault status registers.
 */
typedef struct {
    bool ibat_reg_stat;     ///< IBAT regulation status
    bool vbus_ovp_stat;     ///< VBUS over-voltage status
    bool vbat_ovp_stat;     ///< VBAT over-voltage status
    bool ibus_ocp_stat;     ///< IBUS over-current status
    bool ibat_ocp_stat;     ///< IBAT over-current status
    bool conv_ocp_stat;     ///< Converter over-current status
    bool vac2_ovp_stat;     ///< VAC2 over-voltage status
    bool vac1_ovp_stat;     ///< VAC1 over-voltage status
    bool vsys_short_stat;   ///< VSYS short circuit status
    bool vsys_ovp_stat;     ///< VSYS over-voltage status
    bool otg_ovp_stat;      ///< OTG over-voltage status
    bool otg_uvp_stat;      ///< OTG under-voltage status
    bool tshut_stat;        ///< IC temperature shutdown status
} bq25798_fault_status_t;

/**
 * @brief BQ25798 charger flags.
 */
typedef struct {
    bool iindpm_flag;       ///< IINDPM / IOTG flag
    bool vindpm_flag;       ///< VINDPM / VOTG flag
    bool watchdog_flag;     ///< I2C watchdog timer flag
    bool poor_src_flag;     ///< Poor source detection flag
    bool pg_flag;           ///< Power good flag
    bool ac2_present_flag;  ///< VAC2 present flag
    bool ac1_present_flag;  ///< VAC1 present flag
    bool vbus_present_flag; ///< VBUS present flag
    bool charge_flag;       ///< Charge status flag
    bool ico_flag;          ///< ICO status flag
    bool vbus_flag;         ///< VBUS status flag
    bool treg_flag;         ///< IC thermal regulation flag
    bool vbat_present_flag; ///< VBAT present flag
    bool bc_1_2_done_flag;  ///< BC1.2 status flag
    bool dpdm_done_flag;    ///< D+/D- detection is done flag
    bool adc_done_flag;     ///< ADC conversion flag (only in one-shot mode)
    bool vsys_flag;         ///< VSYSMIN regulation flag
    bool chg_tmr_flag;      ///< Fast charge timer flag
    bool trichg_tmr_flag;   ///< Trickle charge timer flag
    bool prechg_tmr_flag;   ///< Pre-charge timer flag
    bool topoff_tmr_flag;   ///< Top off timer flag
    bool vbat_otg_low_flag; ///< VBAT too low to enable OTG flag
    bool ts_cold_flag;      ///< TS cold temperature flag
    bool ts_cool_flag; ///< TS cool temperature flag
    bool ts_warm_flag; ///< TS warm temperature flag
    bool ts_hot_flag;  ///< TS hot temperature flag
} bq25798_charger_flags_t;

/**
 * @brief BQ25798 fault flags
 */
typedef struct {
    bool ibat_reg_flag;     ///< IBAT regulation flag
    bool vbus_ovp_flag;     ///< VBUS over-voltage flag
    bool vbat_ovp_flag;     ///< VBAT over-voltage flag
    bool ibus_ocp_flag;     ///< IBUS over-current flag
    bool ibat_ocp_flag;     ///< IBAT over-current flag
    bool conv_ocp_flag;     ///< Converter over-current flag
    bool vac2_ovp_flag;     ///< VAC2 over-voltage flag
    bool vac1_ovp_flag;     ///< VAC1 over-voltage flag
    bool vsys_short_flag;   ///< VSYS short circuit flag
    bool vsys_ovp_flag;     ///< VSYS over-voltage flag
    bool otg_ovp_flag;      ///< OTG over-voltage flag
    bool otg_uvp_flag;      ///< OTG under-voltage flag
    bool tshut_flag;        ///< IC thermal shutdown flag

} bq25798_fault_flags_t;

/**
 * @brief BQ25798 flags
 */
typedef struct {
    uint32_t charger_flags;
    uint16_t fault_flags;
} bq25798_flags_t;

/**
 * @breif BQ25798 ADC control settings
 */
typedef struct {
    bool enable;                    ///< ADC Control
    bool rate;                      ///< ADC conversion rate control
    bq25798_adc_speed_t resolution; ///< ADC sample speed
    bool avg;                       ///< ADC average control
    bool avg_init;                  ///< ADC average initial value control
    bool ibus_dis;                  ///< IBUS ADC control
    bool ibat_dis;                  ///< IBAT ADC control
    bool vbus_dis;                  ///< VBUS ADC control
    bool vbat_dis;                  ///< VBAT ADC control
    bool vsys_dis;                  ///< VSYS ADC control
    bool ts_dis;                    ///< TS ADC control
    bool tdie_dis;                  ///< TDIE ADC control
    bool dp_dis;                    ///< D+ ADC control
    bool dm_dis;                    ///< D- ADC control
    bool vac2_dis;                  ///< VAC2 ADC control
    bool vac1_dis;                  ///< VAC1 ADC control
} bq25798_adc_ctrl_t;

/**
 * @brief BQ25798 measurements
 */
typedef struct {
    int ibus;       ///< IBUS ADC reading (mA)
    int ibat;       ///< IBAT ADC reading (mA)
    int vbus;       ///< VBUS ADC reading (mV)
    int vac1;       ///< VAC1 ADC reading (mV)
    int vac2;       ///< VAC2 ADC reading (mV)
    int vbat;       ///< VBAT ADC reading (mV)
    int vsys;       ///< VSYS ADC reading (mV)
    float ts;       ///< TS ADC reading (%)
    float tdie;     ///< TDIE ADC reading (°C)
    int dp;         ///< D+ ADC reading (mV)
    int dm;         ///< D- ADC reading (mV)
} bq25798_measurements_t;

/* Public function prototypes */

bq25798_status_t bq25798_init(bq25798_t *dev, uint8_t address, bq25798_interface_t io);

bq25798_status_t bq25798_get_vsysmin(bq25798_t *dev, int *vsysmin);
bq25798_status_t bq25798_set_vsysmin(bq25798_t *dev, int vsysmin);

bq25798_status_t bq25798_get_charge_limit_v(bq25798_t *dev, int *vreg_lim);
bq25798_status_t bq25798_set_charge_limit_v(bq25798_t *dev, int vreg_lim);

bq25798_status_t bq25798_get_charge_limit_i(bq25798_t *dev, int *ireg_lim);
bq25798_status_t bq25798_set_charge_limit_i(bq25798_t *dev, int ireg_lim);

bq25798_status_t bq25798_get_input_limit_v(bq25798_t *dev, int *vin_lim);
bq25798_status_t bq25798_set_input_limit_v(bq25798_t *dev, int vin_lim);

bq25798_status_t bq25798_get_input_limit_i(bq25798_t *dev, int *iin_lim);
bq25798_status_t bq25798_set_input_limit_i(bq25798_t *dev, int iin_lim);

bq25798_status_t bq25798_get_vbat_lowv(bq25798_t *dev, bq25798_vbat_lowv_t *threshold);
bq25798_status_t bq25798_set_vbat_lowv(bq25798_t *dev, bq25798_vbat_lowv_t threshold);

bq25798_status_t bq25798_get_precharge_lim_i(bq25798_t *dev, int *precharge_lim);
bq25798_status_t bq25798_set_precharge_lim_i(bq25798_t *dev, int precharge_lim);

bq25798_status_t bq25798_reset_settings(bq25798_t *dev);

bq25798_status_t bq25798_get_wd_chg(bq25798_t *dev, bool *stopOnWDT);
bq25798_status_t bq25798_set_wd_chg(bq25798_t *dev, bool stopOnWDT);

bq25798_status_t bq25798_get_termination_curr(bq25798_t *dev, int *term_curr);
bq25798_status_t bq25798_set_termination_curr(bq25798_t *dev, int term_curr);

bq25798_status_t bq25798_get_cell_count(bq25798_t *dev, bq25798_cell_count_t *cellCount);
bq25798_status_t bq25798_set_cell_count(bq25798_t *dev, bq25798_cell_count_t cellCount);

bq25798_status_t bq25798_get_deglitch_time(bq25798_t *dev, bq25798_trechg_time_t *deglitchTime);
bq25798_status_t bq25798_set_deglitch_time(bq25798_t *dev, bq25798_trechg_time_t deglitchTime);

bq25798_status_t bq25798_get_recharge_threshold_offset(bq25798_t *dev, int *thresholdOffset);
bq25798_status_t bq25798_set_recharge_threshold_offset(bq25798_t *dev, int thresholdOffset);

bq25798_status_t bq25798_get_auto_ovp_batt_discharge(bq25798_t *dev, bool *auto_ibatdis);
bq25798_status_t bq25798_set_auto_ovp_batt_discharge(bq25798_t *dev, bool auto_ibatdis);

bq25798_status_t bq25798_get_force_ovp_batt_discharge(bq25798_t *dev, bool *force_ibatdis);
bq25798_status_t bq25798_set_force_ovp_batt_discharge(bq25798_t *dev, bool force_ibatdis);

bq25798_status_t bq25798_get_charge_enable(bq25798_t *dev, bool *en_chg);
bq25798_status_t bq25798_set_charge_enable(bq25798_t *dev, bool en_chg);

bq25798_status_t bq25798_get_ico_enable(bq25798_t *dev, bool *en_ico);
bq25798_status_t bq25798_set_ico_enable(bq25798_t *dev, bool en_ico);

bq25798_status_t bq25798_get_force_ico_enable(bq25798_t *dev, bool *force_ico);
bq25798_status_t bq25798_set_force_ico_enable(bq25798_t *dev, bool force_ico);

bq25798_status_t bq25798_get_hiz_mode(bq25798_t *dev, bool *en_hiz);
bq25798_status_t bq25798_set_hiz_mode(bq25798_t *dev, bool en_hiz);

bq25798_status_t bq25798_get_termination_enable(bq25798_t *dev, bool *en_term);
bq25798_status_t bq25798_set_termination_enable(bq25798_t *dev, bool en_term);

bq25798_status_t bq25798_get_backup_mode_enable(bq25798_t *dev, bool *en_backup);
bq25798_status_t bq25798_set_backup_mode_enable(bq25798_t *dev, bool en_backup);

bq25798_status_t bq25798_get_backup_mode_threshold(bq25798_t *dev, bq25798_vbus_backup_t *vbckup_threshold);
bq25798_status_t bq25798_set_backup_mode_threshold(bq25798_t *dev, bq25798_vbus_backup_t vbckup_threshold);

bq25798_status_t bq25798_get_vac_ovp(bq25798_t *dev, bq25798_vac_ovp_t *vac_threshold);
bq25798_status_t bq25798_set_vac_ovp(bq25798_t *dev, bq25798_vac_ovp_t vac_threshold);

bq25798_status_t bq25798_reset_watchdog(bq25798_t *dev, bool wdt_rst);

bq25798_status_t bq25798_get_watchdog(bq25798_t *dev, bq25798_wdt_t *timer);
bq25798_status_t bq25798_set_watchdog(bq25798_t *dev, bq25798_wdt_t timer);

bq25798_status_t bq25798_get_hvdcp_12V_enable(bq25798_t *dev, bool *en_12v);
bq25798_status_t bq25798_set_hvdcp_12V_enable(bq25798_t *dev, bool en_12v);

bq25798_status_t bq25798_get_hvdcp_9V_enable(bq25798_t *dev, bool *en_9v);
bq25798_status_t bq25798_set_hvdcp_9V_enable(bq25798_t *dev, bool en_9v);

bq25798_status_t bq25798_get_hvdcp_enable(bq25798_t *dev, bool *hvdcp_en);
bq25798_status_t bq25798_set_hvdcp_enable(bq25798_t *dev, bool hvdcp_en);

bq25798_status_t bq25798_get_sfet_mode(bq25798_t *dev, bq25798_sdrv_ctrl_t *sfet_mode);
bq25798_status_t bq25798_set_sfet_mode(bq25798_t *dev, bq25798_sdrv_ctrl_t sfet_mode);

bq25798_status_t bq25798_get_sfet_delay(bq25798_t *dev, bool *sdrv_dly);
bq25798_status_t bq25798_set_sfet_delay(bq25798_t *dev, bool sdrv_dly);

bq25798_status_t bq25798_get_acdrv_disable(bq25798_t *dev, bool *dis_acdrv);
bq25798_status_t bq25798_set_acdrv_disable(bq25798_t *dev, bool dis_acdrv);

bq25798_status_t bq25798_get_pfm_disable(bq25798_t *dev, bool *pfm_fwd_dis);
bq25798_status_t bq25798_set_pfm_disable(bq25798_t *dev, bool pfm_fwd_dis);

bq25798_status_t bq25798_get_ship_wakeup_delay(bq25798_t *dev, bq25798_wkup_dly_t *wkup_dly);
bq25798_status_t bq25798_set_ship_wakeup_delay(bq25798_t *dev, bq25798_wkup_dly_t wkup_dly);

bq25798_status_t bq25798_get_batfet_ldo_disable(bq25798_t *dev, bool *dis_ldo);
bq25798_status_t bq25798_set_batfet_ldo_disable(bq25798_t *dev, bool dis_ldo);

bq25798_status_t bq25798_get_ooa_disable(bq25798_t *dev, bool *dis_fwd_ooa);
bq25798_status_t bq25798_set_ooa_disable(bq25798_t *dev, bool dis_fwd_ooa);

bq25798_status_t bq25798_get_acdrv2_enable(bq25798_t *dev, bool *en_acdrv2);
bq25798_status_t bq25798_set_acdrv2_enable(bq25798_t *dev, bool en_acdrv2);

bq25798_status_t bq25798_get_acdrv1_enable(bq25798_t *dev, bool *en_acdrv1);
bq25798_status_t bq25798_set_acdrv1_enable(bq25798_t *dev, bool en_acdrv1);

bq25798_status_t bq25798_get_pwm_freq(bq25798_t *dev, bq25798_pwm_freq_t *pwm_freq);
bq25798_status_t bq25798_set_pwm_freq(bq25798_t *dev, bq25798_pwm_freq_t pwm_freq);

bq25798_status_t bq25798_get_stat_pin_disable(bq25798_t *dev, bool *dis_stat);
bq25798_status_t bq25798_set_stat_pin_disable(bq25798_t *dev, bool dis_stat);

bq25798_status_t bq25798_get_vsys_short_disable(bq25798_t *dev, bool *dis_vsys_short);
bq25798_status_t bq25798_set_vsys_short_disable(bq25798_t *dev, bool dis_vsys_short);

bq25798_status_t bq25798_get_force_vindpm_detect(bq25798_t *dev, bool *force_vindpm_det);
bq25798_status_t bq25798_set_force_vindpm_detect(bq25798_t *dev, bool force_vindpm_det);

bq25798_status_t bq25798_get_ibus_ocp_enable(bq25798_t *dev, bool *en_ibus_ocp);
bq25798_status_t bq25798_set_ibus_ocp_enable(bq25798_t *dev, bool en_ibus_ocp);

bq25798_status_t bq25798_get_sfet_present(bq25798_t *dev, bool *sfet_present);
bq25798_status_t bq25798_set_sfet_present(bq25798_t *dev, bool sfet_present);

bq25798_status_t bq25798_get_ibat_enable(bq25798_t *dev, bool *en_ibat);
bq25798_status_t bq25798_set_ibat_enable(bq25798_t *dev, bool en_ibat);

bq25798_status_t bq25798_get_iindpm_enable(bq25798_t *dev, bool *en_iindpm);
bq25798_status_t bq25798_set_iindpm_enable(bq25798_t *dev, bool en_iindpm);

bq25798_status_t bq25798_get_ext_ilim_enable(bq25798_t *dev, bool *en_extilim);
bq25798_status_t bq25798_set_ext_ilim_enable(bq25798_t *dev, bool en_extilim);

bq25798_status_t bq25798_get_bat_ocp_enable(bq25798_t *dev, bool *en_batoc);
bq25798_status_t bq25798_set_bat_ocp_enable(bq25798_t *dev, bool en_batoc);

bq25798_status_t bq25798_get_voc_pct(bq25798_t *dev, bq25798_voc_pct_t *voc_pct);
bq25798_status_t bq25798_set_voc_pct(bq25798_t *dev, bq25798_voc_pct_t voc_pct);

bq25798_status_t bq25798_get_voc_delay(bq25798_t *dev, bq25798_voc_dly_t *voc_dly);
bq25798_status_t bq25798_set_voc_delay(bq25798_t *dev, bq25798_voc_dly_t voc_dly);

bq25798_status_t bq25798_get_voc_measurement_rate(bq25798_t *dev, bq25798_voc_rate_t *voc_rate);
bq25798_status_t bq25798_set_voc_measurement_rate(bq25798_t *dev, bq25798_voc_rate_t voc_rate);

bq25798_status_t bq25798_get_mppt_enable(bq25798_t *dev, bool *en_mppt);
bq25798_status_t bq25798_set_mppt_enable(bq25798_t *dev, bool en_mppt);

bq25798_status_t bq25798_get_thermal_reg_threshold(bq25798_t *dev, bq25798_treg_t *treg_threshold);
bq25798_status_t bq25798_set_thermal_reg_threshold(bq25798_t *dev, bq25798_treg_t treg_threshold);

bq25798_status_t bq25798_get_thermal_shutdown_threshold(bq25798_t *dev, bq25798_tshut_t *tshut_threshold);
bq25798_status_t bq25798_set_thermal_shutdown_threshold(bq25798_t *dev, bq25798_tshut_t tshut_threshold);

bq25798_status_t bq25798_get_vbus_pulldown(bq25798_t *dev, bool *vbus_pd_en);
bq25798_status_t bq25798_set_vbus_pulldown(bq25798_t *dev, bool vbus_pd_en);

bq25798_status_t bq25798_get_vac1_pulldown(bq25798_t *dev, bool *vac1_pd_en);
bq25798_status_t bq25798_set_vac1_pulldown(bq25798_t *dev, bool vac1_pd_en);

bq25798_status_t bq25798_get_vac2_pulldown(bq25798_t *dev, bool *vac2_pd_en);
bq25798_status_t bq25798_set_vac2_pulldown(bq25798_t *dev, bool vac2_pd_en);

bq25798_status_t bq25798_get_backup_acfet1_on(bq25798_t *dev, bool *bkup_acfet1_on);
bq25798_status_t bq25798_set_backup_acfet1_on(bq25798_t *dev, bool bkup_acfet1_on);

bq25798_status_t bq25798_get_jeita_high_temp_voltage_setting(bq25798_t *dev, bq25798_jeita_vset_t *jeita_vset);
bq25798_status_t bq25798_set_jeita_high_temp_voltage_setting(bq25798_t *dev, bq25798_jeita_vset_t jeita_vset);

bq25798_status_t bq25798_get_jeita_high_temp_current_setting(bq25798_t *dev, bq25798_jeita_iseth_t *jeita_iseth);
bq25798_status_t bq25798_set_jeita_high_temp_current_setting(bq25798_t *dev, bq25798_jeita_iseth_t jeita_iseth);

bq25798_status_t bq25798_get_jeita_low_temp_current_setting(bq25798_t *dev, bq25798_jeita_isetc_t *jeita_isetc);
bq25798_status_t bq25798_set_jeita_low_temp_current_setting(bq25798_t *dev, bq25798_jeita_isetc_t jeita_isetc);

bq25798_status_t bq25798_get_ts_cool_setting(bq25798_t *dev, bq25798_ts_cool_t *ts_cool);
bq25798_status_t bq25798_set_ts_cool_setting(bq25798_t *dev, bq25798_ts_cool_t ts_cool);

bq25798_status_t bq25798_get_ts_warm_setting(bq25798_t *dev, bq25798_ts_warm_t *ts_warm);
bq25798_status_t bq25798_set_ts_warm_setting(bq25798_t *dev, bq25798_ts_warm_t ts_warm);

bq25798_status_t bq25798_get_ts_hot_setting(bq25798_t *dev, bq25798_bhot_t *bhot);
bq25798_status_t bq25798_set_ts_hot_setting(bq25798_t *dev, bq25798_bhot_t bhot);

bq25798_status_t bq25798_get_ts_cold_setting(bq25798_t *dev, bq25798_bcold_t *bcold);
bq25798_status_t bq25798_set_ts_cold_setting(bq25798_t *dev, bq25798_bcold_t bcold);

bq25798_status_t bq25798_get_ts_ignore(bq25798_t *dev, bool *ts_ignore);
bq25798_status_t bq25798_set_ts_ignore(bq25798_t *dev, bool ts_ignore);

bq25798_status_t bq25798_get_iindpm_status(bq25798_t *dev, bool *iindpm_stat);

bq25798_status_t bq25798_get_vindpm_status(bq25798_t *dev, bool *vindpm_stat);

bq25798_status_t bq25798_get_watchdog_status(bq25798_t *dev, bool *watchdog_stat);

bq25798_status_t bq25798_get_power_good_status(bq25798_t *dev, bool *pg_stat);

bq25798_status_t bq25798_get_ac2_present_status(bq25798_t *dev, bool *ac2_present_stat);

bq25798_status_t bq25798_get_ac1_present_status(bq25798_t *dev, bool *ac1_present_stat);

bq25798_status_t bq25798_get_vbus_present_status(bq25798_t *dev, bool *vbus_present_stat);

bq25798_status_t bq25798_get_charger_status(bq25798_t *dev, bq25798_chg_stat_t *chg_stat);

bq25798_status_t bq25798_get_vbus_status(bq25798_t *dev, bq25798_vbus_stat_t *vbus_stat);

bq25798_status_t bq25798_get_bc_1_2_done_status(bq25798_t *dev, bool *bc_1_2_done_stat);

bq25798_status_t bq25798_get_ico_status(bq25798_t *dev, bq25798_ico_stat_t *ico_stat);

bq25798_status_t bq25798_get_treg_status(bq25798_t *dev, bool *treg_stat);

bq25798_status_t bq25798_get_dpdm_status(bq25798_t *dev, bool *dpdm_stat);

bq25798_status_t bq25798_get_vbat_present_status(bq25798_t *dev, bool *vbat_present_stat);

bq25798_status_t bq25798_get_acrb2_status(bq25798_t *dev, bool *acrb2_stat);

bq25798_status_t bq25798_get_acrb1_status(bq25798_t *dev, bool *acrb1_stat);

bq25798_status_t bq25798_get_adc_done_status(bq25798_t *dev, bool *adc_done_stat);

bq25798_status_t bq25798_get_vsys_status(bq25798_t *dev, bool *vsys_stat);

bq25798_status_t bq25798_get_charge_timer_status(bq25798_t *dev, bool *chg_tmr_stat);

bq25798_status_t bq25798_get_trickle_timer_status(bq25798_t *dev, bool *trichg_tmr_stat);

bq25798_status_t bq25798_get_precharge_timer_status(bq25798_t *dev, bool *prechg_tmr_stat);

bq25798_status_t bq25798_get_vbat_otg_low_status(bq25798_t *dev, bool *vbat_otg_low_stat);

bq25798_status_t bq25798_get_ts_cold_status(bq25798_t *dev, bool *ts_cold_stat);

bq25798_status_t bq25798_get_ts_cool_status(bq25798_t *dev, bool *ts_cool_stat);

bq25798_status_t bq25798_get_ts_warm_status(bq25798_t *dev, bool *ts_warm_stat);

bq25798_status_t bq25798_get_ts_hot_status(bq25798_t *dev, bool *ts_hot_stat);

bq25798_status_t bq25798_get_ibat_regulator_status(bq25798_t *dev, bool *ibat_reg_stat);

bq25798_status_t bq25798_get_vbus_ovp_status(bq25798_t *dev, bool *vbus_ovp_stat);

bq25798_status_t bq25798_get_vbat_ovp_status(bq25798_t *dev, bool *vbat_ovp_stat);

bq25798_status_t bq25798_get_ibus_ocp_status(bq25798_t *dev, bool *ibus_ocp_stat);

bq25798_status_t bq25798_get_ibat_ocp_status(bq25798_t *dev, bool *ibat_ocp_stat);

bq25798_status_t bq25798_get_converter_ocp_status(bq25798_t *dev, bool *conv_ocp_stat);

bq25798_status_t bq25798_get_vac2_ovp_status(bq25798_t *dev, bool *vac2_ovp_stat);

bq25798_status_t bq25798_get_vac1_ovp_status(bq25798_t *dev, bool *vac1_ovp_stat);

bq25798_status_t bq25798_get_vsys_short_status(bq25798_t *dev, bool *vsys_short_stat);

bq25798_status_t bq25798_get_vsys_ovp_status(bq25798_t *dev, bool *vsys_ovp_stat);

bq25798_status_t bq25798_get_otg_ovp_status(bq25798_t *dev, bool *otg_ovp_stat);

bq25798_status_t bq25798_get_otg_uvp_status(bq25798_t *dev, bool *otg_uvp_stat);

bq25798_status_t bq25798_get_thermal_shutdown_status(bq25798_t *dev, bool *tshut_stat);

bq25798_status_t bq25798_check_charger_status(bq25798_t *dev, bq25798_charger_status_t *charger_status);

bq25798_status_t bq25798_check_fault_status(bq25798_t *dev, bq25798_fault_status_t *fault_status);

bq25798_status_t bq25798_get_iindpm_flag(bq25798_t *dev, bool *iindpm_flag);

bq25798_status_t bq25798_get_vindpm_flag(bq25798_t *dev, bool *vindpm_flag);

bq25798_status_t bq25798_get_watchdog_flag(bq25798_t *dev, bool *watchdog_flag);

bq25798_status_t bq25798_get_poor_src_flag(bq25798_t *dev, bool *poor_src_flag);

bq25798_status_t bq25798_get_power_good_flag(bq25798_t *dev, bool *pg_flag);

bq25798_status_t bq25798_get_ac2_present_flag(bq25798_t *dev, bool *ac2_present_flag);

bq25798_status_t bq25798_get_ac1_present_flag(bq25798_t *dev, bool *ac1_present_flag);

bq25798_status_t bq25798_get_vbus_present_flag(bq25798_t *dev, bool *vbus_present_flag);

bq25798_status_t bq25798_get_charge_flag(bq25798_t *dev, bool *charge_flag);

bq25798_status_t bq25798_get_ico_flag(bq25798_t *dev, bool *ico_flag);

bq25798_status_t bq25798_get_vbus_flag(bq25798_t *dev, bool *vbus_flag);

bq25798_status_t bq25798_get_regulator_temp_flag(bq25798_t *dev, bool *treg_flag);

bq25798_status_t bq25798_get_vbat_present_flag(bq25798_t *dev, bool *vbat_present_flag);

bq25798_status_t bq25798_get_bc_1_2_done_flag(bq25798_t *dev, bool *bc_1_2_done_flag);

bq25798_status_t bq25798_get_dpdm_done_flag(bq25798_t *dev, bool *dpdm_done_flag);

bq25798_status_t bq25798_get_adc_done_flag(bq25798_t *dev, bool *adc_done_flag);

bq25798_status_t bq25798_get_vsys_flag(bq25798_t *dev, bool *vsys_flag);

bq25798_status_t bq25798_get_charge_timer_flag(bq25798_t *dev, bool *chg_tmr_flag);

bq25798_status_t bq25798_get_trickle_timer_flag(bq25798_t *dev, bool *trichg_tmr_flag);

bq25798_status_t bq25798_get_precharge_timer_flag(bq25798_t *dev, bool *prechg_tmr_flag);

bq25798_status_t bq25798_get_topoff_timer_flag(bq25798_t *dev, bool *topoff_tmr_flag);

bq25798_status_t bq25798_get_vbat_otg_low_flag(bq25798_t *dev, bool *vbat_otg_low_flag);

bq25798_status_t bq25798_get_ts_cold_flag(bq25798_t *dev, bool *ts_cold_flag);

bq25798_status_t bq25798_get_ts_cool_flag(bq25798_t *dev, bool *ts_cool_flag);

bq25798_status_t bq25798_get_ts_warm_flag(bq25798_t *dev, bool *ts_warm_flag);

bq25798_status_t bq25798_get_ts_hot_flag(bq25798_t *dev, bool *ts_hot_flag);

bq25798_status_t bq25798_get_ibat_reg_flag(bq25798_t *dev, bool *ibat_reg_flag);

bq25798_status_t bq25798_get_vbus_ovp_flag(bq25798_t *dev, bool *vbus_ovp_flag);

bq25798_status_t bq25798_get_vbat_ovp_flag(bq25798_t *dev, bool *vbat_ovp_flag);

bq25798_status_t bq25798_get_ibus_ocp_flag(bq25798_t *dev, bool *ibus_ocp_flag);

bq25798_status_t bq25798_get_ibat_ocp_flag(bq25798_t *dev, bool *ibat_ocp_flag);

bq25798_status_t bq25798_get_converter_ocp_flag(bq25798_t *dev, bool *conv_ocp_flag);

bq25798_status_t bq25798_get_vac2_ovp_flag(bq25798_t *dev, bool *vac2_ovp_flag);

bq25798_status_t bq25798_get_vac1_ovp_flag(bq25798_t *dev, bool *vac1_ovp_flag);

bq25798_status_t bq25798_get_vsys_short_flag(bq25798_t *dev, bool *vsys_short_flag);

bq25798_status_t bq25798_get_vsys_ovp_flag(bq25798_t *dev, bool *vsys_ovp_flag);

bq25798_status_t bq25798_get_otg_ovp_flag(bq25798_t *dev, bool *otg_ovp_flag);

bq25798_status_t bq25798_get_otg_uvp_flag(bq25798_t *dev, bool *otg_uvp_flag);

bq25798_status_t bq25798_get_thermal_shutdown_flag(bq25798_t *dev, bool *tshut_flag);

bq25798_status_t bq25798_check_charger_flags(bq25798_t *dev, uint32_t *charger_flags);

bq25798_status_t bq25798_check_fault_flags(bq25798_t *dev, uint16_t *fault_flags);

bq25798_status_t bq25798_get_adc_enable(bq25798_t *dev, bool *adc_enable);
bq25798_status_t bq25798_set_adc_enable(bq25798_t *dev, bool adc_enable);

bq25798_status_t bq25798_get_adc_rate(bq25798_t *dev, bool *adc_rate);
bq25798_status_t bq25798_set_adc_rate(bq25798_t *dev, bool adc_rate);

bq25798_status_t bq25798_get_adc_resolution(bq25798_t *dev, bq25798_adc_speed_t *resolution);
bq25798_status_t bq25798_set_adc_resolution(bq25798_t *dev, bq25798_adc_speed_t resolution);

bq25798_status_t bq25798_get_adc_averageing_ctrl(bq25798_t *dev, bool *avg);
bq25798_status_t bq25798_set_adc_averageing_ctrl(bq25798_t *dev, bool avg);

bq25798_status_t bq25798_get_adc_avg_init_val(bq25798_t *dev, bool *avg_init);
bq25798_status_t bq25798_set_adc_avg_init_val(bq25798_t *dev, bool avg_init);

bq25798_status_t bq25798_get_ibus_adc_disable(bq25798_t *dev, bool *ibus_dis);
bq25798_status_t bq25798_set_ibus_adc_disable(bq25798_t *dev, bool ibus_dis);

bq25798_status_t bq25798_get_ibat_adc_disable(bq25798_t *dev, bool *ibat_dis);
bq25798_status_t bq25798_set_ibat_adc_disable(bq25798_t *dev, bool ibat_dis);

bq25798_status_t bq25798_get_vbus_adc_disable(bq25798_t *dev, bool *vbus_dis);
bq25798_status_t bq25798_set_vbus_adc_disable(bq25798_t *dev, bool vbus_dis);

bq25798_status_t bq25798_get_vbat_adc_disable(bq25798_t *dev, bool *vbat_dis);
bq25798_status_t bq25798_set_vbat_adc_disable(bq25798_t *dev, bool vbat_dis);

bq25798_status_t bq25798_get_vsys_adc_disable(bq25798_t *dev, bool *vsys_dis);
bq25798_status_t bq25798_set_vsys_adc_disable(bq25798_t *dev, bool vsys_dis);

bq25798_status_t bq25798_get_ts_adc_disable(bq25798_t *dev, bool *ts_dis);
bq25798_status_t bq25798_set_ts_adc_disable(bq25798_t *dev, bool ts_dis);

bq25798_status_t bq25798_get_tdie_adc_disable(bq25798_t *dev, bool *tdie_dis);
bq25798_status_t bq25798_set_tdie_adc_disable(bq25798_t *dev, bool tdie_dis);

bq25798_status_t bq25798_get_dp_adc_disable(bq25798_t *dev, bool *dp_dis);
bq25798_status_t bq25798_set_dp_adc_disable(bq25798_t *dev, bool dp_dis);

bq25798_status_t bq25798_get_dm_adc_disable(bq25798_t *dev, bool *dm_dis);
bq25798_status_t bq25798_set_dm_adc_disable(bq25798_t *dev, bool dm_dis);

bq25798_status_t bq25798_get_vac2_adc_disable(bq25798_t *dev, bool *vac2_dis);
bq25798_status_t bq25798_set_vac2_adc_disable(bq25798_t *dev, bool vac2_dis);

bq25798_status_t bq25798_get_vac1_adc_disable(bq25798_t *dev, bool *vac1_dis);
bq25798_status_t bq25798_set_vac1_adc_disable(bq25798_t *dev, bool vac1_dis);

bq25798_status_t bq25798_get_ibus_measurement(bq25798_t *dev, int *ibus);

bq25798_status_t bq25798_get_ibat_measurement(bq25798_t *dev, int *ibat);

bq25798_status_t bq25798_get_vbus_measurement(bq25798_t *dev, int *vbus);

bq25798_status_t bq25798_get_vac1_measurement(bq25798_t *dev, int *vac1);

bq25798_status_t bq25798_get_vac2_measurement(bq25798_t *dev, int *vac2);

bq25798_status_t bq25798_get_vbat_measurement(bq25798_t *dev, int *vbat);

bq25798_status_t bq25798_get_vsys_measurement(bq25798_t *dev, int *vsys);

bq25798_status_t bq25798_get_ts_measurement(bq25798_t *dev, float *ts);

bq25798_status_t bq25798_get_tdie_measurement(bq25798_t *dev, float *tdie);

bq25798_status_t bq25798_get_dp_measurement(bq25798_t *dev, int *dp);

bq25798_status_t bq25798_get_dm_measurement(bq25798_t *dev, int *dm);

bq25798_status_t bq25798_get_adc_mesurements(bq25798_t *dev, bq25798_measurements_t *measurements);

#ifdef __cplusplus
}
#endif

#endif /* BQ25798_H */