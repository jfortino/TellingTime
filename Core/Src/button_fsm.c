/*
 * button_fsm.c
 *
 *  Created on: Mar 24, 2025
 *  Author: Joseph Fortino
 *
 *  I wanted to structure this FSM like the others in the project. However, since the state must be changed from multiple different hardware ISRs,
 *  I could not structure the FSM as a single switch statement inside a single function. As such, the various functions are called to change states.
 */

#include "watch_config.h"
#ifdef EXTI_SIGNALS

#include "button_fsm.h"

static EUserButton button;
static EButtonState button_state = BUTTON_IDLE;

void Button_InitPress(EUserButton btn)
{
	button_state = INITIAL_PRESS;
	button = btn;
}


void Button_DebounceComplete()
{
	button_state = DEBOUNCED;
}


void Button_ShortPress()
{
	button_state = SHORT_PRESS;
}


void Button_LongPress()
{
	button_state = LONG_PRESS;
}


void Button_Release()
{
	button_state = BUTTON_IDLE;
}


EUserButton Button_GetButton()
{
	return button;
}


EButtonState Button_GetState()
{
	return button_state;
}


EWatchEvent Button_GetWatchEvent()
{
	EWatchEvent watch_event = NO_EVENT;

	switch (button)
	{
		case TIME_BTN:
			watch_event = (button_state == SHORT_PRESS ? GET_TIME_MIN : GET_TIME_FULL);
			break;

		case DATE_BTN:
			watch_event = (button_state == SHORT_PRESS ? GET_DATE_DAY : GET_DATE_FULL);
			break;

		case HR_BTN:
			watch_event = GET_HR;
			break;

		case PO_BTN:
			watch_event = GET_PO;
			break;

		case VOL_UP_BTN:
			watch_event = (button_state == SHORT_PRESS ? VOL_UP : VOL_UNMUTE);
			break;

		case VOL_DOWN_BTN:
			watch_event = (button_state == SHORT_PRESS ? VOL_DOWN : VOL_MUTE);
			break;

		case MENU_UP_BTN:
			watch_event = MENU_UP;
			break;

		case MENU_DOWN_BTN:
			watch_event = MENU_DOWN;
			break;

		case MENU_SELECT_BTN:
			watch_event = (button_state == SHORT_PRESS ? MENU_SELECT : MENU_HOME);
			break;

		default:
			break;
	}

	return watch_event;
}

#endif
