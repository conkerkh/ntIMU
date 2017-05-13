/*
 * myFunc.c
 *
 *  Created on: Nov 8, 2016
 *      Author: khockuba
 */


#include "myFunc.h"

uint32_t getUs(void) {
	register uint32_t ms, cycle_cnt;
	do {
		ms = HAL_GetTick();
		cycle_cnt = SysTick->VAL;
	} while (ms != HAL_GetTick());
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayUs(uint16_t *micros) {
	uint32_t start = getUs();
	while (getUs()-start < (uint32_t) micros) {
		asm("nop");
	}
}
