/*
 * MAX30101_driver.c
 *
 *  Created on: Apr 1, 2025
 *  Author: Joseph Fortino
 */

#include "watch_config.h"
#ifdef BIOSIGNALS

#include "MAX30101_driver.h"

static I2C_HandleTypeDef* hi2c;

ESPO2Status SPO2_Init(I2C_HandleTypeDef* I2C_handle)
{
	if (HAL_I2C_GetState(I2C_handle) != HAL_I2C_STATE_READY)
	{
		return SPO2_ERROR;
	}

	hi2c = I2C_handle;

	// Set all of the sensor config registers
}


ESPO2Status SPO2_MeasureHeartRate()
{
	// Add implementation
}


ESPO2Status SPO2_MeasurePulseOx()
{
	// Add implementation
}

#endif
