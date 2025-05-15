/*
 *  MAX30101_driver.h
 *
 *  Created on: Mar 20, 2025
 *  Author: Joseph Fortino
 *  Contributor: Tara Black
 *  Adapted from: W25Q128JV_driver.h
 */

#ifndef INC_MAX30101_DRIVER_H_
#define INC_MAX30101_DRIVER_H_

#include "i2c.h"
#include <stdint.h>

#define DEVICE_ADDR (0xAE)
#define REG_ADDR_SIZE (1)
#define REG_DATA_SIZE (1)

//====================================================Registers====================================================
// STATUS REGISTERS
#define	INT_STATUS_1_REG (0x00)
#define	INT_STATUS_2_REG (0x01)
#define	INT_EN_1_REG (0x02)
#define	INT_EN_2_REG (0x03)

// FIFO REGISTERS
#define	FIFO_WR_PTR_REG (0x04)
#define	OVF_COUNTER_REG (0x05)
#define	FIFO_RD_PTR_REG (0x06)
#define	FIFO_DATA_REG (0x07)

// CONFIG REGISTERS
#define	FIFO_CONFIG_REG (0x08)
#define	MODE_CONFIG_REG (0x09)
#define	SPO2_CONFIG_REG (0x0A)
// Reserved registers: (0x0B) POR_STATE 0x00 R/W

// LED PULSE AMPLITUDE REGISTERS
#define LEDx_PA_REG(led_num) (0x0C + (led_num - 1) % 4)

// MULTI-LED MODE CONTROL REGISTERS
#define SLOT_1_2_REG (0x11)
#define SLOT_3_4_REG (0x12)
// Reserved registers: 0x13-0x17 POR_STATE 0xFF R/W
// Reserved registers: 0x18-0x1e POR_STATE 0x00 R

// DIE TEMPERATURE REGISTERS
#define DIE_TEMP_INTGR_REG (0x1F)
#define DIE_TEMP_FRAC_REG (0x20)
#define DIE_TEMP_CONFIG_REG (0x21)
// Reserved registers: 0x22-0x2F POR_STATE 0x00 R/W

// PART ID REGISTERS
#define REV_ID_REG (0xFE)
#define PART_ID_REG (0xFF) //POR_STATE 0x15 R


//====================================================Bit Masks====================================================
// INT_STATUS_1 Bit Masks
#define PWR_RDY_MASK (0x01)
#define ALC_OVF_MASK (0x20)
#define PPG_RDY_MASK (0x40)
#define A_FULL_MASK (0x80)

// INT_STATUS_2 Bit Masks
#define DIE_TEMP_RDY_MASK (0x02)

// CONFIG Bit Masks
// FIFO
#define FIFO_A_FULL_MASK (0x0F)
#define FIFO_ROLL_OVER_EN_MASK (0x10)
#define SMP_AVE_MASK (0xE0)
// Mode
#define MODE_MASK (0x07)
#define RESET_MASK (0x40)
#define SHDN_MASK (0x80)
// SP02
#define LED_PW_MASK (0x03)
#define SPO2_SR_MASK (0x1C)
#define SPO2_ADC_RGE_MASK (0x60)

// MULTI-LED MODE Bit Masks
#define SLOTx_MASK(slot_num) (0x07 << ((slot_num % 2) ? 0 : 4))

// DIE TEMPERATURE Bit Masks
#define TFRAC_MASK (0x0F)
#define TEMP_EN_MASK (0x01)

// ENTIRE REGISTER Bit Mask
#define ENTIRE_REG_MASK (0xFF)


//============================================Generic Bit Set and Reset============================================
#define SET_BITS (0xFF)
#define CLEAR_BITS (0x00)

//==============================================FIFO Config Reg Params=============================================
#define SMP_AVE_1 (0x00)
#define SMP_AVE_2 (0x20)
#define SMP_AVE_4 (0x40)
#define SMP_AVE_8 (0x60)
#define SMP_AVE_16 (0x80)
#define SMP_AVE_32 (0xA0)

#define FIFO_A_FULL(empty_samples) (empty_samples * 0x01)


//==============================================Mode Config Reg Params=============================================
#define MODE_HR (0x02)
#define MODE_SPO2 (0x03)
#define MODE_MULTI_LED (0x07)


//==============================================SPO2 Config Reg Params=============================================
#define ADC_RGE_2048 (0x00)
#define ADC_RGE_4096 (0x20)
#define ADC_RGE_8192 (0x40)
#define ADC_RGE_16384 (0x60)

#define SR_50 (0x00)
#define SR_100 (0x04)
#define SR_200 (0x08)
#define SR_400 (0x0C)
#define SR_800 (0x10)
#define SR_1000 (0x14)
#define SR_1600 (0x18)
#define SR_3200 (0x1C)

#define LED_PW_69 (0x00)
#define LED_PW_118 (0x01)
#define LED_PW_215 (0x02)
#define LED_PW_411 (0x03)


//============================================LED Pulse Amplitude Params===========================================
#define LED_PA(current_uA) (current_uA / 200 * 0x01)


//=============================================Multi-LED Control Params============================================
#define SLOTx_NONE (0x00)
#define SLOTx_RED(slot_num) (0x01 << ((slot_num % 2) ? 0 : 4))
#define SLOTx_IR(slot_num) (0x02 << ((slot_num % 2) ? 0 : 4))
#define SLOTx_GREEN(slot_num) (0x03 << ((slot_num % 2) ? 0 : 4))


//===============================================Types and Functions===============================================
typedef enum
{
	SPO2_OK = 0x00,
	SPO2_ERROR = 0x01
} ESPO2Status;

typedef enum
{
	MEASURE_HR,
	MEASURE_SPO2
} ESPO2MeasurementType;


ESPO2Status SPO2_Init(I2C_HandleTypeDef* I2C_handle);
ESPO2Status SPO2_StartMeasurement(ESPO2MeasurementType measurement_type);
ESPO2Status SPO2_StopMeasurement();
ESPO2Status SPO2_ReadNewFIFOData();
ESPO2Status SPO2_ShutDown();
ESPO2Status SPO2_ReleaseShutDown();
ESPO2Status SPO2_ResetDevice();
ESPO2Status SPO2_ReadInterrupts(uint8_t interrupts[5], uint8_t* num_interrupts);
uint32_t SPO2_GetSample(uint8_t led_num, uint8_t sample_num);

#endif /* INC_MAX30101_DRIVER_H_ */
