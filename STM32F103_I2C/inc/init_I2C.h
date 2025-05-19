#ifndef INIT_I2C
#define INIT_I2C

#include <stdio.h>
#include <string.h>
#include "stm32f10x.h" 
#include "init_LED.h"

#define SIZE_BUF 10
#define CLOCK_SPEED  100000
#define TICK_FREQ_1KHZ 1
#define SLAVE_ADDR  0x68

static uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

static uint8_t rezultReadI2C_[SIZE_BUF];
static uint8_t *rezultReadI2C = rezultReadI2C_;

static char *info_str = "out 103:";

void Init_I2C(void);
void Enable_RCC_I2C(void);
void Config_GPIO_I2C(void);
void Config_I2C(void);

uint32_t RCC_GetPCLK1Freq(void);
uint32_t I2C_Speed(uint32_t pclk, uint32_t speed, uint32_t dutyCycle);
uint32_t I2C_Rise_Time(uint32_t freqrange, uint32_t clockSpeed);
uint8_t I2C_AdresSetTime(void);
uint8_t I2C_RX_SetTime(void);
uint8_t I2C_TX_SetTime(void);

uint8_t I2C_Slave_Receive(uint8_t *pData, uint16_t Size);
uint8_t I2C_Slave_Transmit(uint8_t *pData, uint16_t Size);

void ProcessingData(void);
void Error_Handler(void);

#endif
