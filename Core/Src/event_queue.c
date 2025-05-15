/*
 * event_queue.c
 *
 *  Created on: Mar 26, 2025
 *  Author: Joseph Fortino
 */

#include "event_queue.h"

static volatile EWatchEvent event_queue[EVENT_QUEUE_SIZE];
static volatile uint8_t queued_events = 0;

void EventQueue_Enqueue(EWatchEvent event)
{
	if (queued_events < EVENT_QUEUE_SIZE)
	{
		event_queue[queued_events++] = event;
	}
}


EWatchEvent EventQueue_Peek()
{
	return (queued_events > 0 ? event_queue[0] : NO_EVENT);
}


EWatchEvent EventQueue_Dequeue()
{
	EWatchEvent event = EventQueue_Peek();

	if (queued_events > 0)
	{
		for (int i = 1; i < queued_events; i++)
		{
			event_queue[i-1] = event_queue[i];
		}
		queued_events--;
	}

	return event;
}


uint8_t EventQueue_GetNumQueued()
{
	return queued_events;
}


