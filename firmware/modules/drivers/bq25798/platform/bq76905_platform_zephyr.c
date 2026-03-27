/**
 * @file bq25798_platform_zephyr.c
 * @brief Zephyr platform implementation for BQ25798 I2C driver
 * @author Madison Gleydura (DeepSpace00)
 * @date 2026-03-03
 *
 * Implements the three functions declared in bq25798_platform.h using
 * Zephyr's I2C and kernel APIs. The I2C bus and device address are
 * supplied via a context pointer (const struct i2c_dt_spec *) so that
 * multiple BQ25798 instances on different buses are supported without
 * global state.
 *
 * Call bq25798_zephyr_make_io() to build the bq25798_interface_t that
 * ties these implementations to a specific i2c_dt_spec.
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bq25798_platform.h"
#include "bq25798_platform_zephyr.h"

LOG_MODULE_REGISTER(bq25798_platform, CONFIG_CHARGER_LOG_LEVEL);

/* ---------------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------------*/

/**
 * Recover a const struct i2c_dt_spec * from the opaque context pointer.
 * Returns NULL and logs an error if ctx is NULL.
 */
static inline const struct i2c_dt_spec *get_spec(void *ctx)
{
	if (!ctx) {
		LOG_ERR("bq25798 platform: NULL context");
		return NULL;
	}
	return (const struct i2c_dt_spec *)ctx;
}

/* ---------------------------------------------------------------------------
 * Platform function implementations
 * --------------------------------------------------------------------------*/

/**
 * @brief Write bytes to the BQ25798 over I2C.
 *
 * @param dev_addr  7-bit I2C address (ignored – address comes from spec).
 * @param data      Buffer to transmit; data[0] must be the register address.
 * @param len       Total number of bytes to send (register byte + payload).
 * @param ctx       Pointer to the owning struct i2c_dt_spec.
 * @return 0 on success, negative errno on failure.
 */
int platform_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len,
		       void *ctx)
{
	ARG_UNUSED(dev_addr); /* Address comes from the i2c_dt_spec */

	const struct i2c_dt_spec *spec = get_spec(ctx);
	if (!spec) {
		return -EINVAL;
	}

	int ret = i2c_write_dt(spec, data, len);
	if (ret < 0) {
		LOG_ERR("I2C write failed (reg 0x%02x): %d", data[0], ret);
	}
	return ret;
}

/**
 * @brief Read bytes from the BQ25798 over I2C.
 *
 * Assumes the register address was already written by a preceding call to
 * platform_i2c_write(). Uses i2c_read_dt() to clock in the response bytes.
 *
 * @param dev_addr  7-bit I2C address (ignored – address comes from spec).
 * @param data      Buffer to receive into.
 * @param len       Number of bytes to read.
 * @param ctx       Pointer to the owning struct i2c_dt_spec.
 * @return 0 on success, negative errno on failure.
 */
int platform_i2c_read(uint8_t dev_addr, uint8_t *data, uint16_t len,
		      void *ctx)
{
	ARG_UNUSED(dev_addr);

	const struct i2c_dt_spec *spec = get_spec(ctx);
	if (!spec) {
		return -EINVAL;
	}

	int ret = i2c_read_dt(spec, data, len);
	if (ret < 0) {
		LOG_ERR("I2C read failed: %d", ret);
	}
	return ret;
}

/**
 * @brief Blocking millisecond delay.
 *
 * @param ms  Number of milliseconds to sleep.
 */
void platform_delay_ms(uint32_t ms)
{
	k_msleep((int32_t)ms);
}

/* ---------------------------------------------------------------------------
 * Convenience factory
 * --------------------------------------------------------------------------*/

/**
 * @brief Build a bq25798_interface_t pre-wired for Zephyr.
 *
 * @param spec  Pointer to the i2c_dt_spec for this device instance. Must
 *              remain valid for the lifetime of the returned interface.
 * @return Populated bq25798_interface_t ready to pass to bq25798_init().
 */
bq25798_interface_t bq25798_zephyr_make_io(const struct i2c_dt_spec *spec)
{
	return (bq25798_interface_t){
		.i2c_write = platform_i2c_write,
		.i2c_read  = platform_i2c_read,
		.delay_ms  = platform_delay_ms,
		.ctx       = (void *)spec,
	};
}