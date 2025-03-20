/*
 * W25Q128JV_driver.c
 *
 *  Created on: Feb 12, 2025
 *  Author: Joseph Fortino
 */

#include <string.h>
#include "W25Q128JV_driver.h"
#include "watch_config.h"

static SPI_HandleTypeDef* hspi;
static GPIO_TypeDef* cs_pin_port;
static uint16_t cs_pin_num;

static void format_instr(uint8_t instr_code, uint32_t addr, uint8_t* instr)
{
	instr[0] = instr_code;

	for (int i = 0; i < ADDR_SIZE; i++)
	{
		instr[ADDR_SIZE - i] = ((uint8_t*) &addr)[i];
	}
}


EFlashStatus Flash_Init(SPI_HandleTypeDef* SPI, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	if (HAL_SPI_GetState(SPI) == HAL_SPI_STATE_READY)
	{
		hspi = SPI;
		cs_pin_port = GPIOx;
		cs_pin_num = GPIO_Pin;

		return FLASH_OK;
	}

	return FLASH_ERROR;
}



EFlashStatus Flash_ReadData(uint32_t addr, uint8_t* data, uint16_t size)
{
	HAL_StatusTypeDef hal_status;
	uint8_t instr[4];

	// Checks for argument errors
	if (addr + size - 1 > 0x00FFFFFF)
	{
		return FLASH_ERROR;
	}

	// Concatenates and orders instruction code and address
	format_instr(READ_DATA, addr, instr);

	#ifdef USB_DRIVE
	// Waits for any previously running operations to complete
	if (Flash_WaitUntilAvailable() != FLASH_OK)
	{
		return FLASH_ERROR;
	}
	#endif

	// Sets CS pin low
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);

	// Transmits read instruction
	hal_status = HAL_SPI_Transmit(hspi, instr, INSTR_SIZE + ADDR_SIZE, SPI_FLASH_TIMEOUT);
	if (hal_status != HAL_OK)
	{
		HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);
		return FLASH_ERROR;
	}

	// Reads the specified number of bytes of data
	hal_status = HAL_SPI_Receive(hspi, data, size, SPI_FLASH_TIMEOUT);
	// Sets CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}

	return FLASH_OK;
}


EFlashStatus Flash_ReleasePowerDown()
{
	uint8_t instr_code = RELEASE_POWER_DOWN;
	HAL_StatusTypeDef hal_status;

	// Sets the CS pin low
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);
	// Transmits the release power down instruction
	hal_status = HAL_SPI_Transmit(hspi, &instr_code, INSTR_SIZE, SPI_FLASH_TIMEOUT);
	// Sets the CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}

	return FLASH_OK;
}




/*
static EFlashStatus Flash_VolatileSRWriteEnable()
{
	return FLASH_OK;
}


static EFlashStatus Flash_WriteDisable()
{
	return FLASH_OK;
}
*/



/*
EFlashStatus Flash_DeviceID()
{
	return FLASH_OK;
}
*/

#ifdef USB_DRIVE
EFlashStatus Flash_IsBusy()
{
	uint8_t sr1;
	uint8_t instr_code = READ_SR_1;
	HAL_StatusTypeDef hal_status;

	// Sets the CS pin low
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);

	// Transmits the read status register 1 instruction
	hal_status = HAL_SPI_Transmit(hspi, &instr_code, INSTR_SIZE, SPI_FLASH_TIMEOUT);
	if (hal_status != HAL_OK)
	{
		HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);
		return FLASH_ERROR;
	}

	// Reads status register 1
	hal_status = HAL_SPI_Receive(hspi, &sr1, 1, SPI_FLASH_TIMEOUT);
	// Sets the CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}

	// Checks if the busy bit in status register 1 is set
	if (sr1 & BUSY_MASK)
	{
		return FLASH_BUSY;
	}

	return FLASH_OK;
}


EFlashStatus Flash_WaitUntilAvailable()
{
	EFlashStatus flash_status;

	do
	{
		flash_status = Flash_IsBusy();
		if (flash_status == FLASH_ERROR)
		{
			return FLASH_ERROR;
		}

	} while(flash_status == FLASH_BUSY);

	return FLASH_OK;
}


static EFlashStatus Flash_WriteEnable()
{
	uint8_t instr_code = WRITE_EN;
	HAL_StatusTypeDef hal_status;

	// Waits for any previously running operations to complete
	if (Flash_WaitUntilAvailable() != FLASH_OK)
	{
		return FLASH_ERROR;
	}

	// Sets the CS pin low
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);
	// Transmits the write enable instruction
	hal_status = HAL_SPI_Transmit(hspi, &instr_code, INSTR_SIZE, SPI_FLASH_TIMEOUT);
	// Sets the CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}

	return FLASH_OK;
}


EFlashStatus Flash_EraseSection(uint32_t addr, EFlashEraseType erase_type)
{
	HAL_StatusTypeDef hal_status;
	uint8_t instr_code;
	uint8_t instr[4];
	uint32_t section_addr;

	// Sets the instruction code based on the specified type of erase
	switch (erase_type)
	{
		case SECTOR_4KB:
			instr_code = SECTOR_ERASE_4KB;
			section_addr = addr & 0xFFF000;
			break;
		case BLOCK_32KB:
			instr_code = BLOCK_ERASE_32KB;
			section_addr = addr & 0xFF8000;
			break;
		case BLOCK_64KB:
			instr_code = BLOCK_ERASE_64KB;
			section_addr = addr & 0xFF0000;
			break;

		default:
			return FLASH_ERROR;
	}

	// Concatenates and orders instruction code and address (not used for chip erase)
	format_instr(instr_code, section_addr, instr);

	// Executes write enable instruction before erasing
	if (Flash_WriteEnable() != FLASH_OK)
	{
		return FLASH_ERROR;
	}

	// Sets CS pin low
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);

	// Erases the contents of a section
	hal_status = HAL_SPI_Transmit(hspi, (uint8_t*) &instr, INSTR_SIZE + ADDR_SIZE, SPI_FLASH_TIMEOUT);

	// Sets the CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}

	return FLASH_OK;
}


EFlashStatus Flash_EraseChip()
{
	HAL_StatusTypeDef hal_status;
	uint8_t instr_code = CHIP_ERASE;

	// Executes write enable instruction before erasing
	if (Flash_WriteEnable() != FLASH_OK)
	{
		return FLASH_ERROR;
	}

	// Sets CS pin low
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);

	// Erases the contents of the entire chip
	hal_status = HAL_SPI_Transmit(hspi, &instr_code, INSTR_SIZE, SPI_FLASH_TIMEOUT);

	// Sets the CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}

	return FLASH_OK;
}


static EFlashStatus Flash_ProgramPage(uint32_t addr, uint8_t* data, uint16_t size)
{
	HAL_StatusTypeDef hal_status;
	uint8_t instr[4];

	// Concatenates and orders instruction code and address for the page program operation
	format_instr(PAGE_PROGRAM, addr, instr);

	// Executes write enable instruction before programming
	if (Flash_WriteEnable() != FLASH_OK)
	{
		return FLASH_ERROR;
	}

	// Sets CS pin low
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);

	// Transmits page program instruction
	hal_status = HAL_SPI_Transmit(hspi, instr, INSTR_SIZE + ADDR_SIZE, SPI_FLASH_TIMEOUT);
	if (hal_status != HAL_OK)
	{
		HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);
		return FLASH_ERROR;
	}

	// Programs the page
	hal_status = HAL_SPI_Transmit(hspi, data, size, SPI_FLASH_TIMEOUT);

	// Sets CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);
	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}

	return FLASH_OK;
}


EFlashStatus Flash_WriteData(uint32_t start_addr, uint8_t* data, uint16_t size)
{
	uint32_t end_addr = start_addr + size - 1;
	uint32_t start_sector_addr = start_addr & 0xFFF000;
	uint32_t end_sector_addr = end_addr & 0xFFF000;
	uint32_t current_sector_addr;
	uint16_t num_sectors = (end_sector_addr - start_sector_addr) / SECTOR_SIZE + 1;
	uint16_t sector_start_byte, sector_end_byte;
	uint16_t sector_data_size;

	uint8_t sector_buf[SECTOR_SIZE];
	uint8_t* new_data = data;
	uint8_t erase_sector_flag;


	// Checks for argument errors
	if (end_addr > 0x00FFFFFF || size == 0)
	{
		return FLASH_ERROR;
	}


	for (int i = 0; i < num_sectors; i++)
	{
		// Calculates the starting address of each sector that needs programming
		current_sector_addr = (0x001000 * i) + start_sector_addr;
		// Reset the erase sector flag
		erase_sector_flag = 0;
		// If we are not overwriting the whole sector contents, we calculate the start and end page and byte numbers within the sector
		// This really only applies to the first and last sector that are written to (which can be the same sector)
		sector_start_byte = (i == 0) ? (start_addr & 0x000FFF) : 0;
		sector_end_byte = (i == num_sectors - 1) ? (end_addr & 0x000FFF) : (SECTOR_SIZE - 1);
		// Calculates the number of bytes from the passed-in data buffer which we will write to this sector
		sector_data_size = sector_end_byte - sector_start_byte + 1;

		// Stores the entire contents of the current sector into the sector buffer
		if (Flash_ReadData(current_sector_addr, sector_buf, SECTOR_SIZE) != FLASH_OK)
		{
			return FLASH_ERROR;
		}

		// Determines if any bytes being written to the sector flip a 0 -> 1. If so, then the sector must be erased
		for (int j = 0; j < sector_data_size; j++)
		{
			// We XOR the current flash data with the new data we want to write
			// This results in a bit mask of all of the bits that we are going to change
			// We then AND this mask with the new data. Any resulting 1s are bits that flipped 0 -> 1
			if ((sector_buf[sector_start_byte + j] ^ new_data[j]) & new_data[j])
			{
				erase_sector_flag = 1;
				break;	// Jumps out of FOR loop as soon as a 0 -> 1 flip is found
			}
		}

		// Copies passed-in data into the sector buffer while retaining any existing data that should not be overwritten
		memcpy(sector_buf + sector_start_byte, new_data, sector_data_size);
		new_data += sector_data_size;

		// We only erase the sector if we absolutely have to
		if (erase_sector_flag)
		{
			// Erases the current flash sector
			if (Flash_EraseSection(current_sector_addr, SECTOR_4KB) != FLASH_OK)
			{
				return FLASH_ERROR;
			}
		}

		// Programs each page in the current sector
		for (int j = 0; j < PAGES_PER_SECTOR; j++)
		{
			// Programs a single page in the current sector
			if (Flash_ProgramPage(current_sector_addr + (0x000100 * j), sector_buf + (0x000100 * j), PAGE_SIZE) != FLASH_OK)
			{
				return FLASH_ERROR;
			}
		}
	}

	return FLASH_OK;
}

#endif
/*
EFlashStatus Flash_ReadSR(uint8_t sr_num)
{
	return FLASH_OK;
}


EFlashStatus Flash_WriteSR(uint8_t sr_num)
{
	return FLASH_OK;
}


EFlashStatus Flash_ReadSFDPRegister()
{
	return FLASH_OK;
}


EFlashStatus Flash_EraseSecurityRegister()
{
	return FLASH_OK;
}


EFlashStatus Flash_ProgramSecurityRegister()
{
	return FLASH_OK;
}


EFlashStatus Flash_ReadSecurityRegister()
{
	return FLASH_OK;
}


EFlashStatus Flash_GlobalBlockLock()
{
	return FLASH_OK;
}


EFlashStatus Flash_GlobalBlockUnlock()
{
	return FLASH_OK;
}


EFlashStatus Flash_ReadBlockLock()
{
	return FLASH_OK;
}


EFlashStatus Flash_IndividualBlockLock()
{
	return FLASH_OK;
}


EFlashStatus Flash_IndividualBlockUnlock()
{
	return FLASH_OK;
}


EFlashStatus Flash_EraseProgramSuspend()
{
	return FLASH_OK;
}


EFlashStatus Flash_EraseProgramResume()
{
	return FLASH_OK;
}
*/

EFlashStatus Flash_PowerDown()
{
	uint8_t instr_code = POWER_DOWN;
	HAL_StatusTypeDef hal_status;

	// Sets the CS pin low
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);

	// Transmits the power down instruction
	hal_status = HAL_SPI_Transmit(hspi, &instr_code, INSTR_SIZE, SPI_FLASH_TIMEOUT);
	// Sets the CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}

	return FLASH_OK;
}


EFlashStatus Flash_ResetDevice()
{
	uint8_t instr_code = RESET_EN;
	HAL_StatusTypeDef hal_status;

	// Sets the CS pin low
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);

	// Transmits the reset enable instruction
	hal_status = HAL_SPI_Transmit(hspi, &instr_code, INSTR_SIZE, SPI_FLASH_TIMEOUT);
	// Sets the CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}


	// Changes the instruction code and sets the CS pin low again
	instr_code = RESET_DEVICE;
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_RESET);

	// Transmits the reset instruction
	hal_status = HAL_SPI_Transmit(hspi, &instr_code, INSTR_SIZE, SPI_FLASH_TIMEOUT);
	// Sets the CS pin high
	HAL_GPIO_WritePin(cs_pin_port, cs_pin_num, GPIO_PIN_SET);

	if (hal_status != HAL_OK)
	{
		return FLASH_ERROR;
	}

	return FLASH_OK;
}
