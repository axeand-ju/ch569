/*
 * Instrumentation.c
 *
 *  Created on: 13 Oct 2024
 *      Author: axeand
 */
#include <stdio.h>
#include "CH56x_common.h"
#include "Instrumentation.h"

__attribute__ ((aligned(16))) uint32_t instr_event_buffer[MAX_NUM_INSTR_EVENTS] __attribute__((section(".DMADATA")));
int instr_event_ix = 0;

void start_instrumentation()
{
	instr_event_ix = 0;
	R32_TMR1_CNT_END = 0xffffffff;
	R8_TMR1_CTRL_MOD = RB_TMR_ALL_CLEAR;
	R8_TMR1_CTRL_MOD = RB_TMR_COUNT_EN | RB_TMR_CAP_COUNT;

	R32_PB_DIR |= 0x0000ffff;

	R32_PB_OUT = (R32_PB_OUT & 0xffff0000);
}

void print_instr_event_buffer()
{
	int event;
	uint32_t prev_time = 0;
	uint32_t time;
	uint32_t wrap_compensation = 0;
	printf("Instrumentation events: %d\n", instr_event_ix);
	for (int i = 0; i < instr_event_ix; i++)
	{
		event = instr_event_buffer[i] >> 26;
		time  = (instr_event_buffer[i] & 0x03FFFFFF) + wrap_compensation;
		if (time < prev_time)
		{
			time += 1<<26;
			wrap_compensation += 1<<26;
		}

		printf("%d %lu\n", event, time);
	}
}
