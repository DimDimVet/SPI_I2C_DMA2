#ifndef INIT_SPI_MASTER
#define INIT_SPI_MASTER

#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "delay.h"

#define SIZE_BUF_SPI 10

void Init_SPI(void);
void Enable_RCC_SPI1(void);
void Config_GPIO_SPI1(void);
void Config_SPI1(void);
void Config_SPI1_DMA1(void);

uint8_t SPI1_ReadBayt(void);
void SPI1_SetBayt(uint8_t byte);
void ExecutorData(uint8_t *data);

void SPI1_DMA1_ReadChar(char *ch);
void SPI1_DMA1_SetString(char *str);

void Executor_SPI_DMA_RX_Irq(void);
// void Executor_SPI_DMA_TX_Irq(void);

uint8_t SPI_TransmitReceive(void); // test

#endif
