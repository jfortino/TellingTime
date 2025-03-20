/*
 * morse_driver.c
 *
 *  Created on: Feb 6, 2025
 *  Author: Joseph Fortino
 */
#include "watch_config.h"
#ifdef VIBRATION

#include "morse_driver.h"
#include <stdint.h>
#include <string.h>

const char MORSE_NUMBERS[10][6] = {"-- ", ". ", ".. ", "... ", ".- ", "- ", "-. ", "-.. ", "-... ", ".-- "};

EMorseFSMState morse_fsm_state;
static uint8_t wait_cycles;
static char* current_char;
static uint8_t primed = 0;


const char* Generate_Morse(unsigned int num)
{
	char* morse = "";
	int temp = num;

	while (temp > 0)
	{
		strcat(morse, MORSE_NUMBERS[temp % 10]);
		temp /= 10;
	}

	return (const char*) morse;
}


void MorseFSM_Prime(const char* message)
{
	if (!primed)
	{
		morse_fsm_state = NEXT_CHAR;
		current_char = (char*) message;
		wait_cycles = 1;
		primed = 1;
	}
}


EMorseFSMState MorseFSM_Run()
{
	if (primed)
	{
		switch (morse_fsm_state)
		{
			case NEXT_CHAR:
				switch (*(current_char++))
				{
					case '.':
						morse_fsm_state = DOT;
						wait_cycles = DOT_LENGTH;
						break;

					case '-':
						morse_fsm_state = DASH;
						wait_cycles = DASH_LENGTH;
						break;

					case ' ':
						morse_fsm_state = SPACE;
						wait_cycles = SPACE_LENGTH;
						break;

					default:
						morse_fsm_state = MORSE_DONE;
						primed = 0;
						break;
				}
				break;

			case DOT:
			case DASH:
				if (--wait_cycles == 0)
				{
					morse_fsm_state = PAUSE;
					wait_cycles = PAUSE_LENGTH;
				}
				break;

			case SPACE:
				if (--wait_cycles == 0)
				{
					morse_fsm_state = NEXT_CHAR;
				}
				break;

			case PAUSE:
				if (--wait_cycles == 0)
				{
					morse_fsm_state = NEXT_CHAR;
				}
				break;

			default:
				break;
		}
	}

	return morse_fsm_state;
}

#endif
