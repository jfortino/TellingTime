/*
 * morse_driver.c
 *
 *  Created on: Feb 6, 2025
 *  Author: Joseph Fortino
 */

#include "morse_driver.h"
#include <stdint.h>
#include <string.h>
#include <watch_config.h>

const char MORSE_NUMBERS[10][6] = {"-- ", ". ", ".. ", "... ", ".- ", "- ", "-. ", "-.. ", "-... ", ".-- "};

EMorseState fsm_state;
uint8_t wait_cycles;
char* current_char;


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
	fsm_state = NEXT_CHAR;
	current_char = (char*) message;
	wait_cycles = 1;
}


EMorseState MorseFSM_Run()
{
	switch (fsm_state)
	{
		case NEXT_CHAR:
			switch (*(current_char++))
			{
				case '.':
					fsm_state = DOT;
					wait_cycles = DOT_LENGTH;
					break;

				case '-':
					fsm_state = DASH;
					wait_cycles = DASH_LENGTH;
					break;

				case ' ':
					fsm_state = SPACE;
					wait_cycles = SPACE_LENGTH;
					break;

				default:
					fsm_state = DONE;
					break;
			}
			break;

		case DOT:
		case DASH:
			if (--wait_cycles == 0)
			{
				fsm_state = PAUSE;
				wait_cycles = PAUSE_LENGTH;
			}
			break;

		case SPACE:
			if (--wait_cycles == 0)
			{
				fsm_state = NEXT_CHAR;
			}
			break;

		case PAUSE:
			if (--wait_cycles == 0)
			{
				fsm_state = NEXT_CHAR;
			}
			break;

		default:
			break;
	}

	return fsm_state;
}
