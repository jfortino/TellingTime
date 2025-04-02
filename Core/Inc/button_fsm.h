/*
 * button_fsm.h
 *
 *  Created on: Mar 24, 2025
 *  Author: Joseph Fortino
 */

#ifndef INC_BUTTON_FSM_H_
#define INC_BUTTON_FSM_H_

#include "watch_config.h"
#include "event_queue.h"
#include <stdint.h>

typedef enum
{
	BUTTON_IDLE,
	INITIAL_PRESS,
	DEBOUNCED,
	SHORT_PRESS,
	LONG_PRESS,
	RELEASE
} EButtonState;

void Button_InitPress(EUserButton btn);
void Button_DebounceComplete();
void Button_ShortPress();
void Button_LongPress();
void Button_Release();

EButtonState Button_GetState();
EUserButton Button_GetButton();
EWatchEvent Button_GetWatchEvent();



#endif /* INC_BUTTON_FSM_H_ */
