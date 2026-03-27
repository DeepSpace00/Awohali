/**
 * @file bq25798_zephyr.c
 * @brief Zephyr charger subsystem wrapper for the BQ25798 MPPT battery charger
 * @author Madison Gleydura (DeepSpace00)
 * @date 2026-03-03
 *
 * Adapts the platform-agnostic BQ25798 driver to Zephyr's charger driver API
 * (include/zephyr/drivers/charger.h). The wrapper is intentionally thin:
 * all register-level logic stays in bq25798.c; this file only handles
 * Zephyr device model plumbing and unit conversion (mA/mV <-> µA/µV).
 *
 * Devicetree binding expected: "ti,bq25798"
 * Required DT properties:
 *   reg                                   - I2C address (usually 0x6b)
 *   constant-charge-current-max-microamp  - Default ICHG limit (µA)
 *   constant-charge-voltage-max-microvolt - Default VREG limit (µV)
 *
 * Optional DT properties:
 *   ti,cell-count    (1-4, default 1)
 *   ti,en-mppt       (boolean)
 *   ti,input-voltage-limit-millivolt
 *   ti,input-current-limit-milliamp
 *   ti,precharge-current-milliamp
 *   ti,termination-current-milliamp
 */

#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bq25798.h"
#include "bq25798_platform_zephyr.h"

LOG_MODULE_REGISTER(bq25798, CONFIG_CHARGER_LOG_LEVEL);

/* ---------------------------------------------------------------------------
 * Driver config & data structures
 * --------------------------------------------------------------------------*/

struct bq25798_zephyr_config {
	struct i2c_dt_spec i2c;

	/* DT-sourced defaults applied during init (all in SI units used by
	 * the Zephyr charger API: µA and µV) */
	uint32_t charge_current_max_ua;
	uint32_t charge_voltage_max_uv;
	uint32_t input_voltage_limit_mv;  /* mV – hardware register resolution */
	uint32_t input_current_limit_ma;  /* mA – hardware register resolution */
	uint32_t precharge_current_ma;    /* mA */
	uint32_t termination_current_ma;  /* mA */
	uint8_t  cell_count;              /* 1-4  */
	bool     en_mppt;
};

struct bq25798_zephyr_data {
	bq25798_t dev; /* Platform-agnostic driver instance */
};

/* ---------------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------------*/

/** Convert a bq25798_status_t to a Zephyr errno. */
static int bq_to_errno(bq25798_status_t s)
{
	switch (s) {
	case BQ25798_OK:             return 0;
	case BQ25798_ERR_I2C:        return -EIO;
	case BQ25798_ERR_TIMEOUT:    return -ETIMEDOUT;
	case BQ25798_ERR_NULL:       return -EINVAL;
	case BQ25798_ERR_INVALID_ARG:return -EINVAL;
	default:                     return -EIO;
	}
}

/** Map BQ25798 charger status bits to Zephyr charger status. */
static enum charger_status chg_stat_to_zephyr(bq25798_chg_stat_t s)
{
	switch (s) {
	case BQ25798_CHG_STAT_NOT_CHARGING:
		return CHARGER_STATUS_DISCHARGING;
	case BQ25798_CHG_STAT_TRICKLE_CHARGE:
	case BQ25798_CHG_STAT_PRE_CHARGE:
	case BQ25798_CHG_STAT_FAST_CHARGE:
	case BQ25798_CHG_STAT_TAPER_CHARGE:
	case BQ25798_CHG_STAT_TOP_OFF:
		return CHARGER_STATUS_CHARGING;
	case BQ25798_CHG_STAT_CHARGE_TERM:
		return CHARGER_STATUS_FULL;
	default:
		return CHARGER_STATUS_UNKNOWN;
	}
}

/** Map BQ25798 charger status bits to Zephyr charge type. */
static enum charger_charge_type chg_stat_to_charge_type(bq25798_chg_stat_t s)
{
	switch (s) {
	case BQ25798_CHG_STAT_TRICKLE_CHARGE:
		return CHARGER_CHARGE_TYPE_TRICKLE;
	case BQ25798_CHG_STAT_PRE_CHARGE:
		return CHARGER_CHARGE_TYPE_TRICKLE; /* closest Zephyr type */
	case BQ25798_CHG_STAT_FAST_CHARGE:
		return CHARGER_CHARGE_TYPE_FAST;
	case BQ25798_CHG_STAT_TAPER_CHARGE:
	case BQ25798_CHG_STAT_TOP_OFF:
	case BQ25798_CHG_STAT_CHARGE_TERM:
		return CHARGER_CHARGE_TYPE_STANDARD;
	default:
		return CHARGER_CHARGE_TYPE_NONE;
	}
}

/* ---------------------------------------------------------------------------
 * charger_driver_api – get_property
 * --------------------------------------------------------------------------*/

static int bq25798_zephyr_get_prop(const struct device *dev,
				   charger_prop_t prop,
				   union charger_propval *val)
{
	struct bq25798_zephyr_data *data = dev->data;
	bq25798_t *bq = &data->dev;
	bq25798_status_t ret;

	switch (prop) {

	case CHARGER_PROP_ONLINE: {
		bool pg;
		ret = bq25798_get_power_good_status(bq, &pg);
		if (ret != BQ25798_OK) {
			return bq_to_errno(ret);
		}
		val->online = pg ? CHARGER_ONLINE_FIXED : CHARGER_ONLINE_OFFLINE;
		return 0;
	}

	case CHARGER_PROP_STATUS: {
		bq25798_chg_stat_t chg_stat;
		ret = bq25798_get_charger_status(bq, &chg_stat);
		if (ret != BQ25798_OK) {
			return bq_to_errno(ret);
		}
		val->status = chg_stat_to_zephyr(chg_stat);
		return 0;
	}

	case CHARGER_PROP_CHARGE_TYPE: {
		bq25798_chg_stat_t chg_stat;
		ret = bq25798_get_charger_status(bq, &chg_stat);
		if (ret != BQ25798_OK) {
			return bq_to_errno(ret);
		}
		val->charge_type = chg_stat_to_charge_type(chg_stat);
		return 0;
	}

	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA: {
		int ireg_ma;
		ret = bq25798_get_charge_limit_i(bq, &ireg_ma);
		if (ret != BQ25798_OK) {
			return bq_to_errno(ret);
		}
		val->const_charge_current_ua = (uint32_t)ireg_ma * 1000U;
		return 0;
	}

	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV: {
		int vreg_mv;
		ret = bq25798_get_charge_limit_v(bq, &vreg_mv);
		if (ret != BQ25798_OK) {
			return bq_to_errno(ret);
		}
		val->const_charge_voltage_uv = (uint32_t)vreg_mv * 1000U;
		return 0;
	}

	case CHARGER_PROP_PRECHARGE_CURRENT_UA: {
		int ipre_ma;
		ret = bq25798_get_precharge_lim_i(bq, &ipre_ma);
		if (ret != BQ25798_OK) {
			return bq_to_errno(ret);
		}
		val->precharge_current_ua = (uint32_t)ipre_ma * 1000U;
		return 0;
	}

	case CHARGER_PROP_CHARGE_TERM_CURRENT_UA: {
		int iterm_ma;
		ret = bq25798_get_termination_curr(bq, &iterm_ma);
		if (ret != BQ25798_OK) {
			return bq_to_errno(ret);
		}
		val->charge_term_current_ua = (uint32_t)iterm_ma * 1000U;
		return 0;
	}

	case CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV: {
		int vindpm_mv;
		ret = bq25798_get_input_limit_v(bq, &vindpm_mv);
		if (ret != BQ25798_OK) {
			return bq_to_errno(ret);
		}
		val->input_voltage_regulation_voltage_uv = (uint32_t)vindpm_mv * 1000U;
		return 0;
	}

	case CHARGER_PROP_INPUT_REGULATION_CURRENT_UA: {
		int iin_ma;
		ret = bq25798_get_input_limit_i(bq, &iin_ma);
		if (ret != BQ25798_OK) {
			return bq_to_errno(ret);
		}
		val->input_current_regulation_current_ua = (uint32_t)iin_ma * 1000U;
		return 0;
	}

	default:
		return -ENOTSUP;
	}
}

/* ---------------------------------------------------------------------------
 * charger_driver_api – set_property
 * --------------------------------------------------------------------------*/

static int bq25798_zephyr_set_prop(const struct device *dev,
				   charger_prop_t prop,
				   const union charger_propval *val)
{
	struct bq25798_zephyr_data *data = dev->data;
	bq25798_t *bq = &data->dev;
	bq25798_status_t ret;

	switch (prop) {

	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA: {
		/* µA -> mA, rounding down */
		int ma = (int)(val->const_charge_current_ua / 1000U);
		ret = bq25798_set_charge_limit_i(bq, ma);
		return bq_to_errno(ret);
	}

	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV: {
		int mv = (int)(val->const_charge_voltage_uv / 1000U);
		ret = bq25798_set_charge_limit_v(bq, mv);
		return bq_to_errno(ret);
	}

	case CHARGER_PROP_PRECHARGE_CURRENT_UA: {
		int ma = (int)(val->precharge_current_ua / 1000U);
		ret = bq25798_set_precharge_lim_i(bq, ma);
		return bq_to_errno(ret);
	}

	case CHARGER_PROP_CHARGE_TERM_CURRENT_UA: {
		int ma = (int)(val->charge_term_current_ua / 1000U);
		ret = bq25798_set_termination_curr(bq, ma);
		return bq_to_errno(ret);
	}

	case CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV: {
		int mv = (int)(val->input_voltage_regulation_voltage_uv / 1000U);
		ret = bq25798_set_input_limit_v(bq, mv);
		return bq_to_errno(ret);
	}

	case CHARGER_PROP_INPUT_REGULATION_CURRENT_UA: {
		int ma = (int)(val->input_current_regulation_current_ua / 1000U);
		ret = bq25798_set_input_limit_i(bq, ma);
		return bq_to_errno(ret);
	}

	default:
		return -ENOTSUP;
	}
}

/* ---------------------------------------------------------------------------
 * charger_driver_api – charge_enable
 * --------------------------------------------------------------------------*/

static int bq25798_zephyr_charge_enable(const struct device *dev,
					const bool enable)
{
	struct bq25798_zephyr_data *data = dev->data;
	return bq_to_errno(bq25798_set_charge_enable(&data->dev, enable));
}

/* ---------------------------------------------------------------------------
 * Driver init
 * --------------------------------------------------------------------------*/

static int bq25798_zephyr_init(const struct device *dev)
{
	const struct bq25798_zephyr_config *cfg = dev->config;
	struct bq25798_zephyr_data *data = dev->data;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	/* Build the platform interface using the Zephyr I2C implementation */
	bq25798_interface_t io = bq25798_zephyr_make_io(&cfg->i2c);

	/* Initialise the platform-agnostic driver (verifies part number) */
	bq25798_status_t ret = bq25798_init(&data->dev,
					    (uint8_t)cfg->i2c.addr,
					    io);
	if (ret != BQ25798_OK) {
		LOG_ERR("bq25798_init failed: %s", bq25798_stat_error(ret));
		return bq_to_errno(ret);
	}

	/* ---- Apply DT-sourced defaults ---- */

	/* Cell count */
	bq25798_cell_count_t cell_enum;
	switch (cfg->cell_count) {
	case 2:  cell_enum = BQ25798_CELL_COUNT_2S; break;
	case 3:  cell_enum = BQ25798_CELL_COUNT_3S; break;
	case 4:  cell_enum = BQ25798_CELL_COUNT_4S; break;
	default: cell_enum = BQ25798_CELL_COUNT_1S; break;
	}
	ret = bq25798_set_cell_count(&data->dev, cell_enum);
	if (ret != BQ25798_OK) {
		LOG_WRN("Failed to set cell count: %s", bq25798_stat_error(ret));
	}

	/* Charge voltage limit (µV -> mV) */
	if (cfg->charge_voltage_max_uv > 0) {
		ret = bq25798_set_charge_limit_v(&data->dev,
			(int)(cfg->charge_voltage_max_uv / 1000U));
		if (ret != BQ25798_OK) {
			LOG_WRN("Failed to set VREG: %s",
				bq25798_stat_error(ret));
		}
	}

	/* Charge current limit (µA -> mA) */
	if (cfg->charge_current_max_ua > 0) {
		ret = bq25798_set_charge_limit_i(&data->dev,
			(int)(cfg->charge_current_max_ua / 1000U));
		if (ret != BQ25798_OK) {
			LOG_WRN("Failed to set ICHG: %s",
				bq25798_stat_error(ret));
		}
	}

	/* Input voltage limit */
	if (cfg->input_voltage_limit_mv > 0) {
		ret = bq25798_set_input_limit_v(&data->dev,
			(int)cfg->input_voltage_limit_mv);
		if (ret != BQ25798_OK) {
			LOG_WRN("Failed to set VINDPM: %s",
				bq25798_stat_error(ret));
		}
	}

	/* Input current limit */
	if (cfg->input_current_limit_ma > 0) {
		ret = bq25798_set_input_limit_i(&data->dev,
			(int)cfg->input_current_limit_ma);
		if (ret != BQ25798_OK) {
			LOG_WRN("Failed to set IIN_LIM: %s",
				bq25798_stat_error(ret));
		}
	}

	/* Precharge current */
	if (cfg->precharge_current_ma > 0) {
		ret = bq25798_set_precharge_lim_i(&data->dev,
			(int)cfg->precharge_current_ma);
		if (ret != BQ25798_OK) {
			LOG_WRN("Failed to set IPRECHG: %s",
				bq25798_stat_error(ret));
		}
	}

	/* Termination current */
	if (cfg->termination_current_ma > 0) {
		ret = bq25798_set_termination_curr(&data->dev,
			(int)cfg->termination_current_ma);
		if (ret != BQ25798_OK) {
			LOG_WRN("Failed to set ITERM: %s",
				bq25798_stat_error(ret));
		}
	}

	/* MPPT */
	if (cfg->en_mppt) {
		ret = bq25798_set_mppt_enable(&data->dev, true);
		if (ret != BQ25798_OK) {
			LOG_WRN("Failed to enable MPPT: %s",
				bq25798_stat_error(ret));
		}
	}

	/* Enable the ADC so measurements are available immediately */
	ret = bq25798_set_adc_enable(&data->dev, true);
	if (ret != BQ25798_OK) {
		LOG_WRN("Failed to enable ADC: %s", bq25798_stat_error(ret));
	}

	/* Enable charging */
	ret = bq25798_set_charge_enable(&data->dev, true);
	if (ret != BQ25798_OK) {
		LOG_ERR("Failed to enable charging: %s",
			bq25798_stat_error(ret));
		return bq_to_errno(ret);
	}

	LOG_INF("BQ25798 initialised on %s (addr 0x%02x)",
		cfg->i2c.bus->name, cfg->i2c.addr);
	return 0;
}

/* ---------------------------------------------------------------------------
 * API table & device instantiation macro
 * --------------------------------------------------------------------------*/

static const struct charger_driver_api bq25798_charger_api = {
	.get_property  = bq25798_zephyr_get_prop,
	.set_property  = bq25798_zephyr_set_prop,
	.charge_enable = bq25798_zephyr_charge_enable,
};

/*
 * DT_INST_PROP_OR() returns a default of 0 when the optional property is
 * absent, which the init function treats as "not specified / skip".
 */
#define BQ25798_INIT(inst)						       \
	static struct bq25798_zephyr_data bq25798_data_##inst;		       \
									       \
	static const struct bq25798_zephyr_config bq25798_cfg_##inst = {       \
		.i2c = I2C_DT_SPEC_INST_GET(inst),			       \
		.charge_current_max_ua = DT_INST_PROP(inst,		       \
			constant_charge_current_max_microamp),		       \
		.charge_voltage_max_uv = DT_INST_PROP(inst,		       \
			constant_charge_voltage_max_microvolt),		       \
		.input_voltage_limit_mv  = DT_INST_PROP_OR(inst,	       \
			ti_input_voltage_limit_millivolt, 0),		       \
		.input_current_limit_ma  = DT_INST_PROP_OR(inst,	       \
			ti_input_current_limit_milliamp, 0),		       \
		.precharge_current_ma    = DT_INST_PROP_OR(inst,	       \
			ti_precharge_current_milliamp, 0),		       \
		.termination_current_ma  = DT_INST_PROP_OR(inst,	       \
			ti_termination_current_milliamp, 0),		       \
		.cell_count = DT_INST_PROP_OR(inst, ti_cell_count, 1),	       \
		.en_mppt    = DT_INST_PROP(inst, ti_en_mppt),		       \
	};								       \
									       \
	DEVICE_DT_INST_DEFINE(inst,					       \
			      bq25798_zephyr_init,			       \
			      NULL,					       \
			      &bq25798_data_##inst,			       \
			      &bq25798_cfg_##inst,			       \
			      POST_KERNEL,				       \
			      CONFIG_CHARGER_INIT_PRIORITY,		       \
			      &bq25798_charger_api);

DT_INST_FOREACH_STATUS_OKAY(BQ25798_INIT)