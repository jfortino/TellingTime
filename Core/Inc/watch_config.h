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
#define AUDIO_BUFFER_SAMPLES 512		// The number of samples per audio buffer

#define VIBRATION
#define DOT_LENGTH 2		// The number of timer overflow lengths in a dot
#define DASH_LENGTH 5		// The number of timer overflow lengths in a dash
#define SPACE_LENGTH 5		// The number of timer overflow lengths between numbers
#define PAUSE_LENGTH 1		// The number of timer overflow lengths between consecutive pulses


//=================================================Biosignals=================================================
#define BIOSIGNALS
#define SPO2_TIMEOUT 100

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

#endif /* USB_DRIVE */
#endif /* INC_WATCH_CONFIG_H_ */
