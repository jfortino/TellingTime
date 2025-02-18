/*
 * feature_config.h
 *
 *  Created on: Feb 10, 2025
 *  Author: Joseph Fortino
 */

#ifndef INC_WATCH_CONFIG_H_
#define INC_WATCH_CONFIG_H_

//================================================User Feedback===============================================
#define SPEAKER

#define VIBRATION
#ifdef VIBRATION
	#define DOT_LENGTH 2		// The number of timer overflow lengths in a dot
	#define DASH_LENGTH 5		// The number of timer overflow lengths in a dash
	#define SPACE_LENGTH 5		// The number of timer overflow lengths between numbers
	#define PAUSE_LENGTH 1		// The number of timer overflow lengths between consecutive pulses
#endif

//=================================================Biosignals=================================================
#define HEART_RATE_MEASURE
#define PULSE_OX_MEASURE

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

//====================================================Misc====================================================
//#define LOW_POWER_ENABLED
#define DEBUG_TERMINAL


#endif /* INC_WATCH_CONFIG_H_ */
