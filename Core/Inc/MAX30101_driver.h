/*
 *  MAX30101_driver.h
 *
 *  Created on: Mar 20, 2025
 *  Author: Tara Black
 *  Reviewer: Joseph Fortino
 *  Adapted from: W25Q128JV_driver.h
 */

#ifndef INC_MAX30101_DRIVER_H_
#define INC_MAX30101_DRIVER_H_

#define INSTR_SIZE (1) //copied from flash diver
#define ADDR_SIZE (3) // copied from flash driver

// STATUS REGISTERS
#define	INT_STATUS_1 (0x00)
#define	INT_STATUS_2 (0x01)
#define	INT_EN_1 (0x02)
#define	INT_EN_2 (0x03)

// FIFO REGISTERS
#define	FIFO_WR_PTR (0x04)
#define	OVF_CTR (0x05)
#define	FIFO_RD_PTR (0x06)
#define	FIFO_DATA_REG (0x07)

// CONFIG REGISTERS
#define	FIFO_CONFIG (0x08)
#define	MODE_CONFIG (0x09)
#define	SPO2_CONFIG (0x0A)
// Reserved registers: (0x0B) POR_STATE 0x00 R/W

// LED PULSE AMPLITUDE REGISTERS
#define	LED1_PA (0x0C)
#define	LED2_PA (0x0D)
#define LED3_PA (0x0E)
#define LED4_PA (0x0F)

// MULTI-LED MODE CONTROL REGISTERS
#define SLOT_2_1 (0x11)
#define SLOT_4_3 (0x12)
// Reserved registers: 0x13-0x17 POR_STATE 0xFF R/W
// Reserved registers: 0x18-0x1e POR_STATE 0x00 R

// DIE TEMPERATURE REGISTERS
#define DIE_TEMP_INTEGER (0x1F)
#define DIE_TEMP_FRACTION (0x20)
#define DIE_TEMP_CONFIG (0x21)
// Reserved registers: 0x22-0x2F POR_STATE 0x00 R/W

// PART ID REGISTERS
#define REV_ID (0xFE)
#define PART_ID (0xFF) //POR_STATE 0x15 R



// INT_STATUS_1 Bit Masks
#define PWR_RDY_MASK (0x01)
#define ALC_OVF_MASK (0x20)
#define PPG_RDY_MASK (0x40)
#define A_FULL_MASK (0x80)

// INT_STATUS_2 Bit Masks
#define DIE_TEMP_RDY_MASK (0x02)

// FIFO Bit Masks
#define FIFO_WR_PTR_MASK (0x1F)
#define OVF_CTR_MASK (0x1F)
#define FIFO_RD_PTR_MASK (0x1F)

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
#define SP02_SR_MASK (0x1C)
#define SP02_ADC_RGE_MASK (0x60)

// MULTI-LED MODE Bit Masks
#define SLOT_1_MASK (0x07)
#define SLOT_2_MASK (0x70)
#define SLOT_3_MASK (0x07)
#define SLOT_4_MASK (0x70)

// DIE TEMPERATURE Bit Masks
#define TFRAC_MASK (0x0F)
#define TEMP_EN_MASK (0x01)


typedef enum
{
  SP02_OK,
  SP02_ERROR
} ESP02Status;

EFlashStatus SP02_Init(I2C_HandleTypeDef* hi2c);

#endif /* INC_MAX30101_DRIVER_H_ */
