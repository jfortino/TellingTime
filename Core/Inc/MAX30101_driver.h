/*
 *  MAX30101_driver.h
 *
 *  Created on: Mar 20, 2025
 *  Author: Joseph Fortino
 *  GruntWorker: tara black
 *  Adapted from: W25Q128JV_driver.h
 */


#ifndef INC_MAX30101_DRIVER_H_
#define INC_MAX30101_DRIVER_H_
#include <stdint.h>
#include "spi.h"

#define INSTR_SIZE (1) //copied from flash diver
#define ADDR_SIZE (3) // copied from flash driver

// Register Addresses
#define	INTR_STATUS_1 (0x00)
#define	INTR_STATUS_2 (0x01)
#define	INTR_EN_1 (0x02)
#define	INTR_EN_2 (0x03)
#define	FIFO_WR_PNTR (0x04)
#define	OVERFLOW_COUNTER (0x05)
#define	FIFO_RD_PNTR (0x06)
#define	FIFO_DATA_REG (0x07)
#define	FIFO_CONFIG (0x08)
#define	MODE_CONFIG (0x09)
#define	SPO2_CONFIG (0x0A)

// Reserved registers: (0x0B) POR_STATE 0x00 R/W
//LED PULSE AMPLITUDE
#define	LED1_PA (0x0C)//i feel dubious about these
#define	LED2_PA (0x0D)
#define LED3_PA (0x0E)
#define LED4_PA (0x0F)

//MULTI-LED MODE CONTROL REGISTERS
#define SLOT2_1 (0x11)//i feel dubious about these
#define SLOT4_3 (0x12)

//Reserved registers: 0x13-0x17 POR_STATE 0xFF R/W
//Reserved registers: 0x18-0x1e POR_STATE 0x00 R
#define DIE_TEMP_INTEGER (0x1F)
#define DIE_TEMP_FRACTION (0x20)
#define DIE_TEMP_CONFIG (0x21)
//Reserved registers: 0x22-0x2F POR_STATE 0x00 R/W
#define PART_ID (0xFF) //POR_STATE 0x15 R



// SR1 Bit Masks
#define BUSY_MASK (0x01)
#define WEL_MASK (0x02)
#define BP_MASK (0x1C)
#define TB_MASK (0x20)
#define SEC_MASK (0x40)
#define SRP_MASK (0x80)

// SR2 Bit Masks
#define SRL_MASK (0x01)
#define QE_MASK (0x02)
#define R_MASK (0x04)
#define LB_MASK (0x38)
#define CMP_MASK (0x40)
#define SUS_MASK (0x80)

// SR3 Bit Masks
#define WPS_MASK (0x04)
#define DVR_MASK (0x60)
#define HOLD_RST_MASK (0x80)


typedef enum
{
  FLASH_OK       = 0x00,
  FLASH_ERROR    = 0x01,
  FLASH_BUSY     = 0x02
  //FLASH_TIMEOUT  = 0x03
} EFlashStatus;

typedef enum
{
	SECTOR_4KB,
	BLOCK_64KB,
	BLOCK_32KB,
	CHIP
} EFlashEraseType;

EFlashStatus Flash_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
EFlashStatus Flash_IsBusy();
//EFlashStatus Flash_WriteEnable();
//EFlashStatus Flash_VolatileSRWriteEnable();
//EFlashStatus Flash_WriteDisable();
EFlashStatus Flash_ReleasePowerDown();
//EFlashStatus Flash_DeviceID();
EFlashStatus Flash_ReadData(uint32_t addr, uint8_t* data, uint16_t size);
//EFlashStatus Flash_FastRead();
EFlashStatus Flash_PageProgram(uint32_t addr, uint8_t* data, uint16_t size);
EFlashStatus Flash_Erase(uint32_t addr, EFlashEraseType erase_type);
//EFlashStatus Flash_ReadSR(uint8_t sr_num);
//EFlashStatus Flash_WriteSR(uint8_t sr_num);
//EFlashStatus Flash_ReadSFDPRegister();
//EFlashStatus Flash_EraseSecurityRegister();
//EFlashStatus Flash_ProgramSecurityRegister();
//EFlashStatus Flash_ReadSecurityRegister();
//EFlashStatus Flash_GlobalBlockLock();
//EFlashStatus Flash_GlobalBlockUnlock();
//EFlashStatus Flash_ReadBlockLock();
//EFlashStatus Flash_IndividualBlockLock();
//EFlashStatus Flash_IndividualBlockUnlock();
//EFlashStatus Flash_EraseProgramSuspend();
//EFlashStatus Flash_EraseProgramResume();
EFlashStatus Flash_PowerDown();
//EFlashStatus Flash_EnableReset();
EFlashStatus Flash_ResetDevice();


#endif /* INC_MAX30101_DRIVER_H_ */
