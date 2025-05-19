#ifndef INIT_USART
#define INIT_USART

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "init_LED.h"

#define SIZE_BUF_USART 10
#define SIZE_BUF_USART_MAX 100
#define SIZE_BUF_INFO 35

static uint8_t count_size_buf;

void Init_USART1(uint16_t baudRate);
void Enable_RCC_USART1(void);
void Config_GPIO_USART1(void);
void Config_USART1(uint16_t baudRate);

void USART1_ReadChar(char *ch);
void USART1_SetString(char *str);

void ExecutorTerminal_USART_Irq(void);
#endif