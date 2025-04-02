/*
 * event_queue.h
 *
 *  Created on: Mar 26, 2025
 *  Author: Joseph Fortino
 */

#ifndef INC_EVENT_QUEUE_H_
#define INC_EVENT_QUEUE_H_

#include <stdint.h>
#include "main.h"

#define EVENT_QUEUE_SIZE 5

typedef enum
{
	NO_EVENT,
	GET_TIME_MIN,
	GET_TIME_FULL,
	GET_DATE_DAY,
	GET_DATE_FULL,
	GET_HR,
	GET_SPO2,
	VOL_UP,
	VOL_DOWN,
	VOL_MUTE,
	VOL_UNMUTE,
	MENU_UP,
	MENU_DOWN,
	MENU_SELECT,
	CHARGE_CYCLE_START,
	CHARGE_CYCLE_END,
	LOW_BATTERY_DETECT
} EWatchEvent;

void EventQueue_Enqueue(EWatchEvent event);
EWatchEvent EventQueue_Peek();
EWatchEvent EventQueue_Dequeue();
uint8_t EventQueue_GetNumQueued();

#endif /* INC_EVENT_QUEUE_H_ */
