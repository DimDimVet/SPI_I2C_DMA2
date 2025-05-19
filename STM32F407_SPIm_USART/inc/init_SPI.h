#ifndef INIT_SPI
#define INIT_SPI

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "delay.h"

void Init_SPI(void);
void Enable_RCC_SPI1(void);
void Config_GPIO_SPI1(void);
void Config_SPI1(void);

uint8_t SPI2_TransmitReceive(uint8_t data);
uint8_t SPI2_ReadBayt(void);
uint8_t SPI2_SetBayt(char byte);

#endif
