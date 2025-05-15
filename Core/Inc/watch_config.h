/*
 * feature_config.h
 *
 *  Created on: Feb 10, 2025
 *  Author: Joseph Fortino
 */

#ifndef INC_WATCH_CONFIG_H_
#define INC_WATCH_CONFIG_H_


//====================================================Misc====================================================
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
#define AUDIO_BUFFER_SAMPLES 1024		// The number of samples per audio buffer for 1 audio channel
#define NUM_AUDIO_BUFFERS 1				// The number of audio buffers (1 = single buffer, 2 = double buffering). Value should not exceed 2.
										// NOTE: If NUM_AUDIO_BUFFERS == 1, the DMA must be set to "Normal" mode
#define MAX_AUDIO_LIST_SIZE 20

#define VIBRATION
#define MAX_MORSE_BUFFER_SIZE 15
#define DOT_LENGTH 2		// The number of timer overflow lengths in a dot
#define DASH_LENGTH 5		// The number of timer overflow lengths in a dash
#define SPACE_LENGTH 5		// The number of timer overflow lengths between numbers
#define PAUSE_LENGTH 1		// The number of timer overflow lengths between consecutive pulses


//=================================================Biosignals=================================================
//#define HEARTRATE
//#define PULSE_OX
#define SPO2_TIMEOUT 100
#define HR_SAMPLE_NUM 2000
#define SPO2_SAMPLE_NUM 5
#define FIR_FILTER_TAPS 66
#define IIR_FILTER_STAGES 4


//================================================Time Keeping================================================
#define DATE_TIME
#ifdef DATE_TIME
typedef enum
{
	DATE_MDY,
	DATE_DMY
} EDateFormat;

typedef enum
{
	TIME_TWELVE_HOUR,
	TIME_TWENTYFOUR_HOUR
} ETimeFormat;
#endif

#define ALARMS


//===============================================User Interface===============================================
#define EXTI_SIGNALS
typedef enum	// The value for each button must match the EXTI line number that it is connected to
{
	TIME_BTN = 0,
	DATE_BTN = 15,
	HR_BTN = 11,
	PO_BTN = 4,
	VOL_UP_BTN = 14,
	VOL_DOWN_BTN = 8,
	MENU_UP_BTN = 13,
	MENU_DOWN_BTN = 3,
	MENU_SELECT_BTN = 1,
	SPO2_INT = 9,
	MCP_STAT_INT = 2,
} EUserButton;

typedef enum
{
	AUDIO_MODE,
	VIBRATION_MODE
} EFeedbackMode;


//==============================================Battery Management============================================
//#define MEASURE_BATT
#define BATT_MEASURE_SAMPLES 6

#define CHARGING_MONITOR

//#define LOW_POWER_MODE

#endif /* USB_DRIVE */
#endif /* INC_WATCH_CONFIG_H_ */
