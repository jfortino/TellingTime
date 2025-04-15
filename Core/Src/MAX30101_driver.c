/*
 *  MAX30101_driver.c
 *
 *  Created on: Apr 1, 2025
 *  Author: Joseph Fortino
 *  Consultant: Tara Black
 */

#include "watch_config.h"
#ifdef BIOSIGNALS

#include "MAX30101_driver.h"
#include <string.h>

static I2C_HandleTypeDef* hi2c;
static uint8_t measurement_data[192];
static uint8_t num_active_leds = 0;
static uint8_t adc_bit_resolution = 0;


static ESPO2Status SPO2_ReadRegister(uint16_t reg_addr, uint8_t* reg_data, uint16_t data_size)
{
	if (HAL_I2C_Mem_Read(hi2c, DEVICE_ADDR, reg_addr, REG_ADDR_SIZE, reg_data, data_size, SPO2_TIMEOUT) != HAL_OK)
	{
		return SPO2_ERROR;
	}

	return SPO2_OK;
}


static ESPO2Status SPO2_WriteRegister(uint16_t reg_addr, uint8_t reg_mask, uint8_t reg_data)
{
	uint8_t old_reg_data;
	uint8_t new_reg_data = reg_data;

	// We only need to read the current contents of the register if we are writing to part of it
	if (reg_mask != ENTIRE_REG_MASK)
	{
		// Reads the current data in the register
		if (SPO2_ReadRegister(reg_addr, &old_reg_data, REG_DATA_SIZE) != SPO2_OK)
		{
			return SPO2_ERROR;
		}

		// Generates the new 8-bit register value while preserving the data we don't want to overwrite
		new_reg_data = (old_reg_data & ~reg_mask) | (reg_data & reg_mask);
	}

	// Writes the new register data to the register
	if (HAL_I2C_Mem_Write(hi2c, DEVICE_ADDR, reg_addr, REG_ADDR_SIZE, &new_reg_data, REG_DATA_SIZE, SPO2_TIMEOUT) != HAL_OK)
	{
		return SPO2_ERROR;
	}

	return SPO2_OK;
}


ESPO2Status SPO2_Init(I2C_HandleTypeDef* I2C_handle)
{
	if (HAL_I2C_GetState(I2C_handle) != HAL_I2C_STATE_READY)
	{
		return SPO2_ERROR;
	}

	hi2c = I2C_handle;

	ESPO2Status spo2_init_status = SPO2_OK;

	spo2_init_status |= SPO2_WriteRegister(INT_EN_1_REG, A_FULL_MASK, SET_BITS);
	//spo2_init_status |= SPO2_WriteRegister(INT_EN_2_REG, DIE_TEMP_RDY_MASK, SET_BITS);

	spo2_init_status |= SPO2_WriteRegister(FIFO_CONFIG_REG, FIFO_A_FULL_MASK, FIFO_A_FULL(15));
	spo2_init_status |= SPO2_WriteRegister(FIFO_CONFIG_REG, SMP_AVE_MASK, CLEAR_BITS);
	spo2_init_status |= SPO2_WriteRegister(SPO2_CONFIG_REG, LED_PW_MASK, LED_PW_118);
	spo2_init_status |= SPO2_WriteRegister(SPO2_CONFIG_REG, SPO2_ADC_RGE_MASK, ADC_RGE_16384);

	// We always want the first time slot to be IR LED since it is used for both HR and SPO2 measurements
	//spo2_init_status |= SPO2_WriteRegister(SLOT_1_2_REG, SLOTx_MASK(1), SLOTx_IR(1));
	spo2_init_status |= SPO2_WriteRegister(SLOT_1_2_REG, SLOTx_MASK(1), SLOTx_RED(1));

	spo2_init_status |= SPO2_WriteRegister(LEDx_PA_REG(1), ENTIRE_REG_MASK, LED_PA(6000));
	spo2_init_status |= SPO2_WriteRegister(LEDx_PA_REG(2), ENTIRE_REG_MASK, LED_PA(12000));
	spo2_init_status |= SPO2_WriteRegister(LEDx_PA_REG(3), ENTIRE_REG_MASK, LED_PA(24000));
	spo2_init_status |= SPO2_WriteRegister(LEDx_PA_REG(4), ENTIRE_REG_MASK, LED_PA(48000));

	// Enables shutdown mode
	spo2_init_status |= SPO2_WriteRegister(MODE_CONFIG_REG, SHDN_MASK, SET_BITS);

	// Sets the sensor mode - we always want to use multi-LED mode, so we don't have to deal with red LED temperature compensation
	spo2_init_status |= SPO2_WriteRegister(MODE_CONFIG_REG, MODE_MASK, MODE_MULTI_LED);

	//spo2_init_status |= SPO2_WriteRegister(SLOT_1_2, SLOTx_MASK(1) | SLOTx_MASK(2), SLOTx_IR(1) | SLOTx_IR(2));
	//spo2_init_status |= SPO2_WriteRegister(SLOT_3_4, SLOTx_MASK(3) | SLOTx_MASK(4), SLOTx_RED(3) | SLOTx_GREEN(4));

	uint8_t dummy;
	spo2_init_status |=  SPO2_ReadRegister(INT_STATUS_1_REG, &dummy, REG_DATA_SIZE);		// Clears the PWR_RDY interrupt

	if (spo2_init_status != SPO2_OK)
	{
		return SPO2_ERROR;
	}

	uint8_t led_pw;
	spo2_init_status |=  SPO2_ReadRegister(SPO2_CONFIG_REG, &led_pw, REG_DATA_SIZE);
	adc_bit_resolution = 15 + (led_pw & LED_PW_MASK);				// Calculates the number of bits of ADC resolution

	return SPO2_OK;
}


ESPO2Status SPO2_StartMeasurement(ESPO2MeasurementType measurement_type)
{
	ESPO2Status spo2_status = SPO2_OK;

	// Resets the FIFO read and write pointers as well as the sample overflow counter
	spo2_status |= SPO2_WriteRegister(FIFO_RD_PTR_REG, ENTIRE_REG_MASK, CLEAR_BITS);
	spo2_status |= SPO2_WriteRegister(FIFO_WR_PTR_REG, ENTIRE_REG_MASK, CLEAR_BITS);
	spo2_status |= SPO2_WriteRegister(OVF_COUNTER_REG, ENTIRE_REG_MASK, CLEAR_BITS);

	// Allows us to set different sample rates for HR and SPO2 Measurements
	spo2_status |= SPO2_WriteRegister(SPO2_CONFIG_REG, SPO2_SR_MASK, (measurement_type == MEASURE_HR) ? SR_100 : SR_100);
	// Enables Green LED time slot if we are taking an SPO2 measurement, else it disables the time slot
	spo2_status |= SPO2_WriteRegister(SLOT_1_2_REG, SLOTx_MASK(2), (measurement_type == MEASURE_SPO2) ? SLOTx_GREEN(2) : CLEAR_BITS);

	num_active_leds = (measurement_type == MEASURE_HR ? 1 : 2);

	// Starts the measurement
	spo2_status |= SPO2_WriteRegister(MODE_CONFIG_REG, SHDN_MASK, CLEAR_BITS);

	return spo2_status;
}


ESPO2Status SPO2_StopMeasurement()
{
	if (SPO2_WriteRegister(MODE_CONFIG_REG, SHDN_MASK, SET_BITS) != SPO2_OK)
	{
		return SPO2_ERROR;
	}

	return SPO2_OK;
}


ESPO2Status SPO2_ReadNewFIFOData()
{
	uint8_t fifo_rd_ptr, fifo_wr_ptr;
	uint8_t samples_to_read;
	uint8_t sample_width = num_active_leds * 3;

	// Gets the current FIFO read and write pointers
	if (SPO2_ReadRegister(FIFO_RD_PTR_REG, &fifo_rd_ptr, REG_DATA_SIZE) != SPO2_OK)
	{
		return SPO2_ERROR;
	}

	if (SPO2_ReadRegister(FIFO_WR_PTR_REG, &fifo_wr_ptr, REG_DATA_SIZE) != SPO2_OK)
	{
		return SPO2_ERROR;
	}

	// Calculates the number of samples we need to read
	samples_to_read = (fifo_wr_ptr - fifo_rd_ptr) % 32;

	// Loops through each sample
	for (int i = 0; i < samples_to_read; i++)
	{
		if (SPO2_ReadRegister(FIFO_DATA_REG, &(measurement_data[i * sample_width]), sample_width) != SPO2_OK)
		{
			return SPO2_ERROR;
		}
	}

	return SPO2_OK;
}

/*
ESPO2Status SPO2_ShutDown()
{
	return SPO2_WriteRegister(MODE_CONFIG_REG, SHDN_MASK, SET_BITS);
}


ESPO2Status SPO2_ReleaseShutDown()
{
	return SPO2_WriteRegister(MODE_CONFIG_REG, SHDN_MASK, CLEAR_BITS);
}
*/


ESPO2Status SPO2_Reset()
{
	return SPO2_WriteRegister(MODE_CONFIG_REG, RESET_MASK, SET_BITS);
}


ESPO2Status SPO2_ReadInterrupts(uint8_t interrupts[5], uint8_t* num_interrupts)
{
	uint8_t int_1_reg, int_2_reg;

	memset(interrupts, 0, 5);
	*num_interrupts = 0;

	if (SPO2_ReadRegister(INT_STATUS_1_REG, &int_1_reg, REG_DATA_SIZE) != SPO2_OK)
	{
		return SPO2_ERROR;
	}

	if (SPO2_ReadRegister(INT_STATUS_2_REG, &int_2_reg, REG_DATA_SIZE) != SPO2_OK)
	{
		return SPO2_ERROR;
	}

	if (int_1_reg & PWR_RDY_MASK)
	{
		interrupts[(*num_interrupts)++] = PWR_RDY_MASK;
	}

	if (int_1_reg & ALC_OVF_MASK)
	{
		interrupts[(*num_interrupts)++] = ALC_OVF_MASK;
	}

	if (int_1_reg & PPG_RDY_MASK)
	{
		interrupts[(*num_interrupts)++] = PPG_RDY_MASK;
	}

	if (int_1_reg & A_FULL_MASK)
	{
		interrupts[(*num_interrupts)++] = A_FULL_MASK;
	}

	if (int_2_reg & DIE_TEMP_RDY_MASK)
	{
		interrupts[(*num_interrupts)++] = DIE_TEMP_RDY_MASK;
	}

	return SPO2_OK;
}


uint32_t SPO2_GetSample(uint8_t led_num, uint8_t sample_num)
{
	uint32_t sample = 0;
	uint8_t sample_width = num_active_leds * 3;

	for (int i = 0; i < 3; i++)
	{
		sample |= measurement_data[sample_num * sample_width + (led_num-1) * 3 + i] << ((2-i) * 8);
	}

	sample = sample >> (18 - adc_bit_resolution);	// We may have to shift everything back because MSB is at fixed position, and we may not be using all 18 ADC bits

	return sample;
}

#endif
