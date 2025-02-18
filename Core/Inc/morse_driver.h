/*
 * morse_driver.h
 *
 *  Created on: Feb 6, 2025
 *  Author: Joseph Fortino
 */

#ifndef INC_MORSE_DRIVER_H_
#define INC_MORSE_DRIVER_H_

typedef enum
{
	NEXT_CHAR,
	DOT,
	DASH,
	SPACE,
	PAUSE,
	DONE
} EMorseState;

const char* Generate_Morse(unsigned int num);

void MorseFSM_Prime(const char* message);

EMorseState MorseFSM_Run();


#endif /* INC_MORSE_DRIVER_H_ */
