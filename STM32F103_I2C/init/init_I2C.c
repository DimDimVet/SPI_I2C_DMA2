#include "init_I2C.h"

char dataBufRxSPI[SIZE_BUF];

void Init_I2C()
{
  Enable_RCC_I2C();
  Config_GPIO_I2C();
  Config_I2C();
}

void Enable_RCC_I2C(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

void Config_GPIO_I2C()
{
  // Настраиваем PB6 (SCL), PB7 (SDA) как альтернативные функции

  GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);   // reset
  GPIOB->CRL |= GPIO_CRL_MODE6;                      // PB6
  GPIOB->CRL |= (GPIO_CRL_CNF6_0 | GPIO_CRL_CNF6_1); // PB6 alt

  GPIOB->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);   // reset
  GPIOB->CRL |= GPIO_CRL_MODE7;                      // PB7
  GPIOB->CRL |= (GPIO_CRL_CNF7_0 | GPIO_CRL_CNF7_1); // PB7 alt
}

void Config_I2C()
{
  uint32_t pclk1;
  uint32_t freqrange;

  // Настройка I2C (100kHz)

  pclk1 = RCC_GetPCLK1Freq();

  freqrange = pclk1 / 1000000; // 0x00000010;

  /* Configure I2Cx: Frequency range */
  I2C1->CR2 |= I2C_CR2_FREQ_1;

  I2C1->TRISE |= I2C_Rise_Time(freqrange, CLOCK_SPEED);

  I2C1->CCR |= I2C_Speed(pclk1, CLOCK_SPEED, 0);

  I2C1->CR1 |= (0 | 0);

  I2C1->OAR1 = SLAVE_ADDR;
  I2C1->OAR1 &= ~I2C_OAR1_ADDMODE;

  I2C1->OAR2 &= ~I2C_OAR2_ENDUAL;

  I2C1->CR2 |= I2C_CR2_ITEVTEN; // вкл прерывани

  I2C1->CR1 |= I2C_CR1_PE;

  NVIC_EnableIRQ(I2C1_EV_IRQn);
}

uint32_t RCC_GetPCLK1Freq(void)
{
  uint32_t temp = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> 10]; // 10U=RCC_CFGR_PPRE1_Pos
  return temp;
}

uint32_t I2C_Speed(uint32_t pclk, uint32_t speed, uint32_t dutyCycle)
{
  uint32_t timing = 0;

  if (dutyCycle == I2C_CCR_DUTY)
  {
    // Формула для коэффициента заполнения 16/9
    timing = (pclk / (speed * 9)) - 1;
  }
  else
  {
    // Формула для коэффициента заполнения 1/2
    timing = (pclk / (speed * 2)) - 1;
  }

  return timing;
}

uint32_t I2C_Rise_Time(uint32_t freqrange, uint32_t clockSpeed)
{
  if (clockSpeed <= 100000)
  {
    freqrange = freqrange + 1;
  }
  else
  {
    freqrange = ((freqrange * 300) / 1000) + 1;
  }
  return freqrange;
}

uint8_t I2C_AdresSetTime()
{
  // ждем адрес
  while (!(I2C1->SR1 & I2C_SR1_ADDR))
  {
  }
  return 0;
}

uint8_t I2C_RX_SetTime()
{
  // ждем прием данных
  while (!(I2C1->SR1 & I2C_SR1_RXNE))
  {
  }
  return 0;
}

uint8_t I2C_TX_SetTime()
{
  // ждем передачу данных
  while (!(I2C1->SR1 & I2C_SR1_TXE))
  {
  }
  return 0;
}

///////////////////////
uint8_t I2C_Slave_Receive(uint8_t *pData, uint16_t Size)
{
  // откл POS
  I2C1->CR1 &= ~I2C_CR1_POS;
  // вкл проверку адреса
  I2C1->CR1 |= I2C_CR1_ACK;

  // проверим адрес
  if (I2C_AdresSetTime() != 0)
  {
    return 1;
  }

  // сброс флага адреса
  I2C1->SR2;

  while (Size > 0) // крутим
  {
    // проверим данные
    if (I2C_RX_SetTime() != 0)
    {
      // откл проверку адреса
      I2C1->CR1 &= ~I2C_CR1_ACK;
      return 1;
    }

    // читаем данные
    *pData = (uint8_t)I2C1->DR;
    pData++;
    Size--;
  }

  // сброс флага
  I2C1->SR1;

  // откл проверку адреса
  I2C1->CR1 &= ~I2C_CR1_ACK;

  return 0;
}
////////////////////
uint8_t I2C_Slave_Transmit(uint8_t *pData, uint16_t Size)
{
  // откл POS
  I2C1->CR1 &= ~I2C_CR1_POS;
  // вкл проверку адреса
  I2C1->CR1 |= I2C_CR1_ACK;

  // проверим адрес
  if (I2C_AdresSetTime() != 0)
  {
    return 1;
  }

  // сброс флага адреса
  I2C1->SR2;

  while (Size > 0U) // крутим
  {
    // проверим данные
    if (I2C_TX_SetTime() != 0)
    {
      // откл проверку адреса
      I2C1->CR1 &= ~I2C_CR1_ACK;
      return 1;
    }

    // запишем данные
    I2C1->DR = *pData;
    pData++;
    Size--;
  }

  // сброс флага
  I2C1->SR1;

  // откл проверку адреса
  I2C1->CR1 &= ~I2C_CR1_ACK;

  return 0;
}

////IRQ
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

void I2C1_EV_IRQHandler(void)
{
  I2C1->CR2 &= ~I2C_CR2_ITEVTEN;
  ProcessingData();
  LED13();
}
