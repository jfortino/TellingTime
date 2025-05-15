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
static uint8_t length_multiplyer = 1;

void generate_number_morse(char* morse_buffer, unsigned int num)
{
	int temp = num;
	int digits[4];	// We should only ever have to work with 4 digit numbers
	int num_digits = 0;

	// Stores the digits of num into the digits array in ascending order (1s, 10s, 100s, etc.)
	// Also counts the number of digits
	while (temp > 0)
	{
		digits[num_digits] = temp % 10;
		temp /= 10;
		num_digits++;
	}

	// Converts each digit into its morse code in reverse order (100s, 10s, 1s) and generates the morse string
	for (int i = num_digits; i > 0; i--)
	{
		strcat(morse_buffer, MORSE_NUMBERS[digits[i-1]]);
	}
}


void MorseFSM_Prime(char* message, uint8_t multiplyer)
{
	if (!primed)
	{
		morse_fsm_state = NEXT_CHAR;
		current_char = message;
		length_multiplyer = multiplyer;
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
						wait_cycles = DOT_LENGTH * length_multiplyer;
						break;

					case '-':
						morse_fsm_state = DASH;
						wait_cycles = DASH_LENGTH * length_multiplyer;
						break;

					case ' ':
						morse_fsm_state = SPACE;
						wait_cycles = SPACE_LENGTH * length_multiplyer;
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
