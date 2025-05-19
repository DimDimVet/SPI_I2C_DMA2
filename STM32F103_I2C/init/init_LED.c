#include "init_LED.h"

void Init_LED()
{
	Enable_RCC_LED();
	Config_LED();
}

void Enable_RCC_LED()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
}

void Config_LED()
{
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
	GPIOC->CRH |= (~GPIO_CRH_CNF13) | GPIO_CRH_MODE13_1;
	GPIOC->BSRR = GPIO_BSRR_BS13; // Установить
	GPIOC->BSRR = GPIO_BSRR_BR13; // Сбросить
}

void LED13()
{
	GPIOC->ODR ^= GPIO_ODR_ODR13;
}
