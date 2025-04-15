/*
 * feature_config.h
 *
 *  Created on: Feb 10, 2025
 *  Author: Joseph Fortino
 */

#ifndef INC_WATCH_CONFIG_H_
#define INC_WATCH_CONFIG_H_

//====================================================Misc====================================================
//#define LOW_POWER_ENABLED
#define DEBUG_TERMINAL

#define SPI_FLASH
#define SPI_FLASH_TIMEOUT 100

// USB MSC can't work without external flash
#ifndef SPI_FLASH
#undef USB_DRIVE
#endif
// USB Eats a lot of Flash so we have 2 different software builds, one for USB drive (initial loading of data on device) and one for normal operation
// This symbol is defined by the build configuration
#ifndef USB_DRIVE

//================================================User Feedback===============================================
#define AUDIO
#define AUDIO_BUFFER_SAMPLES 512		// The number of samples per audio buffer for 1 audio channel
#define NUM_AUDIO_BUFFERS 2				// The number of audio buffers (1 = single buffer, 2 = double buffering). Value should not exceed 2.
										// NOTE: If NUM_AUDIO_BUFFERS == 1, the DMA must be set to "Normal" mode

#define VIBRATION
#define DOT_LENGTH 2		// The number of timer overflow lengths in a dot
#define DASH_LENGTH 5		// The number of timer overflow lengths in a dash
#define SPACE_LENGTH 5		// The number of timer overflow lengths between numbers
#define PAUSE_LENGTH 1		// The number of timer overflow lengths between consecutive pulses


//=================================================Biosignals=================================================
#define BIOSIGNALS
#define SPO2_TIMEOUT 100
#define HR_BUFFER_NUM 1
#define SPO2_BUFFER_NUM 1

//================================================Time Keeping================================================
#define DATE_TIME
#ifdef DATE_TIME
	typedef enum
	{
		MDY,
		DMY
	} EDateFormat;

	typedef enum
	{
		TWELVE,
		TWENTYFOUR
	} ETimeFormat;
#endif

#define ALARMS


//===============================================User Interface===============================================
#define EXTI_SIGNALS
typedef enum	// The value for each button must match the EXTI line number that it is connected to
{
	TIME_BTN = 3,
	DATE_BTN = 0,
	HR_BTN = 4,
	SPO2_BTN = 1,
	VOL_UP_BTN = 2,
	VOL_DOWN_BTN = 5,
	MENU_UP_BTN = 6,
	MENU_DOWN_BTN = 7,
	MENU_SELECT_BTN = 8,
	SPO2_INT = 9,
	CHARGE_STAT_INT = 10,
	LOW_BATTERY_INT = 11
} EUserButton;

#endif /* USB_DRIVE */
#endif /* INC_WATCH_CONFIG_H_ */
