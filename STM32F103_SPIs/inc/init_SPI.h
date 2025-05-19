#ifndef INIT_SPI_MASTER
#define INIT_SPI_MASTER

#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "delay.h"
#include "init_LED.h"

#define SIZE_BUF_SPI 10

static uint8_t count_LED_SPI_RX;

void Init_SPI(void);
void Enable_RCC_SPI1(void);
void Config_GPIO_SPI1(void);
void Config_SPI1(void);

uint8_t SPI1_ReadBayt(void);
void SPI1_SetBayt(uint8_t byte);
void ExecutorData(uint8_t *data);

uint8_t SPI_TransmitReceive(void); // test

#endif
