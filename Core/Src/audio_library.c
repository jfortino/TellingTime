/*
 * audio_library.c
 *
 *  Created on: Mar 9, 2025
 *  Author: Joseph Fortino
 */

#include "watch_config.h"
#ifdef AUDIO

#include "audio_library.h"

const char AUDIO_FILENAMES[NUM_FILES][MAX_FILENAME_LENGTH] =
{
		"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",										// 0-9
		"10", "11", "12", "13", "14", "15", "16", "17", "18", "19",								// 10-19
		"20", "30", "40", "50", "60", "70", "80", "90", "hundred",								// tens and hundred
		"sun", "mon", "tue", "wed", "thu", "fri", "sat",										// days of the week
		"jan", "feb", "mar", "apr", "may", "june", "july", "aug", "sept", "oct", "nov", "dec",	// months
		"date", "day", "month", "year",															// date stuff
		"time", "am", "pm",	"minute", "hour",													// time stuff
		"vibe", "length", "short", "medium", "long",														// vibration length
		"hr", "bpm", "spo2", "percent",															// biosignal stuff
		"your", "is", "set", "low", "battery", "format"											// misc
};

#endif
