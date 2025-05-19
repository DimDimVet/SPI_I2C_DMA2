#ifndef INIT_SPI
#define INIT_SPI

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "delay.h"

#define ONESIZE 1
static uint32_t dataBufRxSPI[5];

void Init_SPI(void);
void Enable_RCC_SPI1(void);
void Config_GPIO_SPI1(void);
void Config_SPI1(void);
void Config_SPI1_DMA1(void);

uint8_t SPI2_TransmitReceive(uint8_t data);
uint8_t SPI2_ReadBayt(void);
uint8_t SPI2_SetBayt(char byte);

char SPI1_DMA1_ReadChar(void);
void SPI1_DMA1_SetString(char *str);
void SPI1_DMA1_SetChar(char *ch);

void Executor_SPI_DMA_RX_Irq(void);
// void Executor_SPI_DMA_TX_Irq(void);

#endif
