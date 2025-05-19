#ifndef INIT_I2C
#define INIT_I2C

#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "init_LED.h"

#define BUFFER_SIZE_I2C 10
#define I2C_ADDRESSINGMODE_7BIT 0x00004000U
#define CLOCK_SPEED 100000

#define I2C_TIMEOUT_BUSY_FLAG 25U
#define I2C_FLAG_BUSY 0x00100002U
#define I2C_FLAG_BTF 0x00010004U
#define I2C_FLAG_MASK 0x0000FFFFU

void Init_I2C(void);
void Enable_RCC_I2C(void);
void Config_GPIO_I2C(void);
void Config_I2C(void);
void Config_I2C_DMA1(void);

uint32_t RCC_GetPCLK1Freq(void);
uint32_t I2C_Speed(uint32_t pclk, uint32_t speed, uint32_t dutyCycle);
uint32_t I2C_Rise_Time(uint32_t freqrange, uint32_t clockSpeed);
uint8_t I2C_AdresSetTime(void);
uint8_t I2C_StartBit_SetTime(void);
uint8_t I2C_BTFBit_SetTime(void);
uint8_t I2C_TX_SetTime(void);
uint8_t I2C_MasterRequestRead(uint16_t DevAddress);
uint8_t I2C_MasterRequestWriteT(uint16_t DevAddress);
uint8_t I2C_RX_SetTime(void);
uint8_t I2C_GET_FLAG(uint32_t flag_BTF);
uint8_t I2C_BUSYBit_SetTime(void);

uint8_t I2C_Master_ReceiveT(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
uint8_t I2C_Master_TransmitT(uint16_t DevAddress, uint8_t *pData, uint16_t Size);

uint8_t I2C_Master_TransmitDMA(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
void Transmit_I2C_DMA(uint8_t *data, uint16_t Size);
uint8_t *Receive_I2C_DMA();

void Error_Handler(void);
#endif
