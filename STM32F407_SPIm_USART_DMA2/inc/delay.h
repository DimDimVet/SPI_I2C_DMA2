#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f4xx.h"

#define US SystemCoreClock / 1000000 // SystemCoreClock=72000000
#define SYSTICK_MAX_VALUE 16777215
#define US_MAX_VALUE SYSTICK_MAX_VALUE / (US)

static uint16_t count = 0;

void delay_us(uint16_t us);
void delay_ms(uint16_t ms);
void delay_s(uint16_t s);

#endif
