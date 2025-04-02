/*
 * W25Q128JV_driver.h
 *
 *  Created on: Feb 12, 2025
 *  Author: Joseph Fortino
 */


#ifndef INC_W25Q128JV_DRIVER_H_
#define INC_W25Q128JV_DRIVER_H_

#include <stdint.h>
#include "spi.h"

#define INSTR_SIZE (1)			// The instruction size in bytes
#define ADDR_SIZE (3)			// The address size in bytes
#define PAGE_SIZE (256)			// The page size in bytes
#define SECTOR_SIZE (4096)		// The sector size in bytes
#define PAGES_PER_SECTOR (SECTOR_SIZE / PAGE_SIZE)
#define SUB_BLK_SIZE (32768)	// The sub block size in bytes
#define BLK_SIZE (65536)		// The block size in bytes

// Instruction Codes
#define	WRITE_EN (0x06)
#define	VOLATILE_SR_WRITE_EN (0x50)
#define	WRITE_DISABLE (0x04)
#define	RELEASE_POWER_DOWN (0xAB)
#define	MANUFACTURER_AND_DEVICE_ID (0x90)
#define	JEDEC_ID (0x9F)
#define	READ_UNIQUE_ID (0x4B)
#define	READ_DATA (0x03)
#define	FAST_READ (0x0B)
#define	PAGE_PROGRAM (0x02)
#define	SECTOR_ERASE_4KB (0x20)
#define	BLOCK_ERASE_32KB (0x52)
#define	BLOCK_ERASE_64KB (0xD8)
#define	CHIP_ERASE (0xC7)
#define READ_SR_1 (0x05)
#define WRITE_SR_1 (0x01)
#define READ_SR_2 (0x35)
#define WRITE_SR_2 (0x31)
#define READ_SR_3 (0x15)
#define WRITE_SR_3 (0x11)
#define READ_SFDP_REGISTER (0x5A)
#define ERASE_SECURITY_REGISTER (0x44)
#define PROGRAM_SECURITY_REGISTER (0x42)
#define READ_SECURITY_REGISTER (0x48)
#define GLOBAL_BLOCK_LOCK (0x7E)
#define GLOBAL_BLOCK_UNLOCK (0x98)
#define READ_BLOCK_LOCK (0x3D)
#define INDIVIDUAL_BLOCK_LOCK (0x36)
#define INDIVIDUAL_BLOCK_UNLOCK (0x39)
#define ERASE_OR_PROGRAM_SUSPEND (0x75)
#define ERASE_OR_PROGRAM_RESUME (0x7A)
#define POWER_DOWN (0xB9)
#define RESET_EN (0x66)
#define RESET_DEVICE (0x99)

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
  FLASH_OK,
  FLASH_ERROR,
  FLASH_BUSY
} EFlashStatus;

typedef enum
{
	SECTOR_4KB,
	BLOCK_64KB,
	BLOCK_32KB
} EFlashEraseType;

EFlashStatus Flash_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
EFlashStatus Flash_ReadData(uint32_t addr, uint8_t* data, uint16_t size);
EFlashStatus Flash_ReleasePowerDown();
EFlashStatus Flash_IsBusy();
EFlashStatus Flash_WaitUntilAvailable();
//EFlashStatus Flash_WriteEnable();
//EFlashStatus Flash_VolatileSRWriteEnable();
//EFlashStatus Flash_WriteDisable();
//EFlashStatus Flash_DeviceID();
//EFlashStatus Flash_FastRead();
EFlashStatus Flash_WriteData(uint32_t addr, uint8_t* data, uint16_t size);
EFlashStatus Flash_EraseSection(uint32_t addr, EFlashEraseType erase_type);
EFlashStatus Flash_EraseChip();
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


#endif /* INC_W25Q128JV_DRIVER_H_ */
