/*
 * bq25798_platform.c
 *
 *  Created on: Jun 28, 2025
 *      Author: deepspace
 */

#include "main.h"
#include "bq25798_platform.h"

extern I2C_HandleTypeDef hi2c1;

BQ25798_Status bq25798_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len)
{
	if (HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) != HAL_OK)
		return BQ25798_ERROR;
	return BQ25798_OK;
}

BQ25798_Status bq25798_i2c_write(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len)
{
	if (HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, HAL_MAX_DELAY) != HAL_OK)
		return BQ25798_ERROR;
	return BQ25798_OK;
}