/*
 * morse_driver.h
 *
 *  Created on: Feb 6, 2025
 *  Author: Joseph Fortino
 */

#include <stdint.h>

#ifndef INC_MORSE_DRIVER_H_
#define INC_MORSE_DRIVER_H_

typedef enum
{
	NEXT_CHAR,
	DOT,
	DASH,
	SPACE,
	PAUSE,
	MORSE_DONE
} EMorseFSMState;

extern EMorseFSMState morse_fsm_state;

const char* Generate_Morse(unsigned int num);

void MorseFSM_Prime(char* message, uint8_t multiplyer);

EMorseFSMState MorseFSM_Run();


#endif /* INC_MORSE_DRIVER_H_ */
