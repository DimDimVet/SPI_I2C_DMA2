#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f4xx.h" // Device header

#define US SystemCoreClock / 1000000 // SystemCoreClock=72000000
#define SYSTICK_MAX_VALUE 16777215
#define US_MAX_VALUE SYSTICK_MAX_VALUE / (US)

void delay_us(int us);
void delay_ms(int ms);
void delay_s(int s);

#endif
