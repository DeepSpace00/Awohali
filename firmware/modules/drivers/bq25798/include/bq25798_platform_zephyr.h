/**
 * @file bq25798_platform_zephyr.h
 * @brief Zephyr-specific platform helpers for BQ25798
 * @author Madison Gleydura (DeepSpace00)
 * @date 2026-03-03
 */

#ifndef BQ25798_PLATFORM_ZEPHYR_H
#define BQ25798_PLATFORM_ZEPHYR_H

#include "bq25798.h"
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Build a bq25798_interface_t wired to a Zephyr i2c_dt_spec.
 *
 * @param spec  Pointer to the i2c_dt_spec for this device. Must remain
 *              valid for the lifetime of the bq25798_t instance.
 * @return Populated bq25798_interface_t ready to pass to bq25798_init().
 */
bq25798_interface_t bq25798_zephyr_make_io(const struct i2c_dt_spec *spec);

#ifdef __cplusplus
}
#endif

#endif /* BQ25798_PLATFORM_ZEPHYR_H */