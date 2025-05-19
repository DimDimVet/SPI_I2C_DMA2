#include "init_I2C.h"

uint32_t dataBufRx[1];

void Init_I2C()
{
  Enable_RCC_I2C();
  Config_GPIO_I2C();
  Config_I2C();
}

void Enable_RCC_I2C()
{
  RCC->AHB1ENR |= 1 << RCC_AHB1ENR_GPIOBEN_Pos; // Включаем тактирование порта B
  RCC->APB1ENR |= 1 << RCC_APB1ENR_I2C1EN_Pos;  // Включаем тактирование
  RCC->AHB1ENR |= 1 << RCC_AHB1ENR_DMA1EN_Pos;  // Включаем тактирование DMA1
}

void Config_GPIO_I2C()
{
  ////    //PB6 (SCL), PB7 (SDA)

  GPIOB->MODER |= 0 << GPIO_MODER_MODE6_Pos;       // Очистка режима для PB6
  GPIOB->MODER |= 2 << GPIO_MODER_MODE6_Pos;       // Альтернативная функция для PB6 (SCL)
  GPIOB->OTYPER |= 1 << GPIO_OTYPER_OT6_Pos;       // открытый коллектор
  GPIOB->OSPEEDR |= 3 << GPIO_OSPEEDR_OSPEED6_Pos; // скорость

  GPIOB->MODER |= 0 << GPIO_MODER_MODE7_Pos;       // Очистка режима для PB7
  GPIOB->MODER |= 2 << GPIO_MODER_MODE7_Pos;       // Альтернативная функция для PB7 (SDA)
  GPIOB->OTYPER |= 1 << GPIO_OTYPER_OT7_Pos;       // открытый коллектор
  GPIOB->OSPEEDR |= 3 << GPIO_OSPEEDR_OSPEED7_Pos; // скорость

  GPIOB->AFR[0] |= 4 << GPIO_AFRL_AFSEL6_Pos; // AF4 для I2C PB6 (SCL)
  GPIOB->AFR[0] |= 4 << GPIO_AFRL_AFSEL7_Pos; // AF4 для I2C PB7 (SDA)
}

void Config_I2C()
{
  uint32_t freqrange;
  uint32_t pclk1;

  /*Reset I2C*/
  I2C1->CR1 = 1 << I2C_CR1_SWRST_Pos;
  I2C1->CR1 = 0 << I2C_CR1_SWRST_Pos;

  /* Get PCLK1 frequency */
  pclk1 = RCC_GetPCLK1Freq();
  // pclk1 =0x00F42400;
  freqrange = pclk1 / 1000000;

  /* Configure I2Cx: Frequency range */
  I2C1->CR2 |= freqrange << I2C_CR2_FREQ_Pos;

  /* Configure I2Cx: Rise Time */
  I2C1->TRISE |= I2C_Rise_Time(freqrange, CLOCK_SPEED) << I2C_TRISE_TRISE_Pos;

  /* Configure I2Cx: Speed */
  I2C1->CCR |= I2C_Speed(pclk1, CLOCK_SPEED, 0); // << (I2C_CCR_FS_Pos | I2C_CCR_DUTY_Pos | I2C_CCR_CCR_Pos);

  /* Configure I2Cx: Generalcall and NoStretch mode */
  I2C1->CR1 |= (0 | 0) << (I2C_CR1_ENGC_Pos | I2C_CR1_NOSTRETCH_Pos);

  /* Configure I2Cx: Own Address1 and addressing mode */
  I2C1->OAR1 |= (I2C_ADDRESSINGMODE_7BIT | 0) << (I2C_OAR1_ADDMODE_Pos | I2C_OAR1_ADD0_Pos);

  /* Configure I2Cx: Dual mode and Own Address2 */
  I2C1->OAR2 |= (0 | 0) << (I2C_OAR2_ENDUAL_Pos | I2C_OAR2_ADD2_Pos);

  /* Enable the selected I2C peripheral */
  I2C1->CR1 |= 1 << I2C_CR1_PE_Pos;
  NVIC_SetPriority(I2C1_EV_IRQn, 1); // Установите приоритет
  NVIC_EnableIRQ(I2C1_EV_IRQn);
}
/////////////

uint32_t RCC_GetPCLK1Freq(void)
{
  uint32_t temp = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]; // 10U=RCC_CFGR_PPRE1_Pos
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

uint8_t I2C_StartBit_SetTime()
{
  // ждем передачу данных
  while (!(I2C1->SR1 & I2C_SR1_SB))
  {
  }
  return 0;
}

uint8_t I2C_BTFBit_SetTime()
{
  // ждем передачу данных
  while (!(I2C1->SR1 & I2C_SR1_BTF))
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

uint8_t I2C_MasterRequestRead(uint16_t DevAddress)
{

  I2C1->CR1 |= 1 << I2C_CR1_ACK_Pos;

  I2C1->CR1 |= 1 << I2C_CR1_START_Pos;

  I2C_StartBit_SetTime();

  I2C1->DR = (uint8_t)(DevAddress | I2C_OAR1_ADD0); // I2C_7BIT_ADD_READ(DevAddress);

  I2C_AdresSetTime();
  return 0;
}

uint8_t I2C_MasterRequestWriteT(uint16_t DevAddress)
{

  I2C1->CR1 |= 1 << I2C_CR1_START_Pos;

  I2C_StartBit_SetTime();

  I2C1->DR = (uint8_t)(DevAddress & (~I2C_OAR1_ADD0)); // I2C_7BIT_ADD_WRITE(DevAddress);

  I2C_AdresSetTime();

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

uint8_t I2C_GET_FLAG(uint32_t flag_BTF)
{

  if (((uint8_t)(flag_BTF >> 16U)) == 0x01U)
  {
    if (((I2C1->SR1) & (flag_BTF & I2C_FLAG_MASK)) == (flag_BTF & I2C_FLAG_MASK))
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else
  {
    if (((I2C1->SR2) & (flag_BTF & I2C_FLAG_MASK)) == (flag_BTF & I2C_FLAG_MASK))
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
}

uint8_t I2C_BUSYBit_SetTime() // I2C_FLAG_BUSY=25
{
  // ждем передачу данных
  while (I2C_GET_FLAG(I2C_FLAG_BUSY) == I2C_TIMEOUT_BUSY_FLAG)
  {
  }
  return 0;
}

///////////////////////
uint8_t I2C_Master_ReceiveT(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{

  I2C_BUSYBit_SetTime();

  /* Disable Pos */
  I2C1->CR1 |= 0 << I2C_CR1_POS_Pos;

  /* Send Slave Address */
  if (I2C_MasterRequestRead(DevAddress) != 0)
  {
    return 1;
  }

  if (Size == 0U)
  {
    // сброс флага адреса
    I2C1->SR2;

    /* Generate Stop */
    I2C1->CR1 |= 1 << I2C_CR1_STOP_Pos;
  }
  else if (Size == 1U)
  {
    // откл проверку адреса
    I2C1->CR1 &= ~(1 << I2C_CR1_ACK_Pos);

    // сброс флага адреса
    I2C1->SR2;

    /* Generate Stop */
    I2C1->CR1 |= 1 << I2C_CR1_STOP_Pos;
  }
  else if (Size == 2U)
  {

    // откл проверку адреса
    I2C1->CR1 &= ~(1 << I2C_CR1_ACK_Pos);

    /* Enable Pos */
    I2C1->CR1 |= 1 << I2C_CR1_POS_Pos;

    // сброс флага адреса
    I2C1->SR2;
  }
  else
  {

    // вкл проверку адреса
    I2C1->CR1 |= 1 << I2C_CR1_ACK_Pos;

    // сброс флага адреса
    I2C1->SR2;
  }

  while (Size > 0U)
  {
    if (Size <= 3U)
    {
      /* One byte */
      if (Size == 1U)
      {

        I2C_RX_SetTime();

        /* Read data from DR */
        *pData = (uint8_t)I2C1->DR;
        pData++;
        Size--;
      }
      /* Two bytes */
      else if (Size == 2U)
      {

        I2C_BTFBit_SetTime();

        /* Generate Stop */
        I2C1->CR1 |= 1 << I2C_CR1_STOP_Pos;

        /* Read data from DR */
        *pData = (uint8_t)I2C1->DR;
        pData++;
        Size--;

        /* Read data from DR */
        *pData = (uint8_t)I2C1->DR;
        pData++;
        Size--;
      }
      /* 3 Last bytes */
      else
      {
        I2C_BTFBit_SetTime();

        // откл проверку адреса
        I2C1->CR1 &= ~(1 << I2C_CR1_ACK_Pos);

        /* Read data from DR */
        *pData = (uint8_t)I2C1->DR;
        pData++;
        Size--;

        I2C_BTFBit_SetTime();

        /* Generate Stop */
        I2C1->CR1 |= 1 << I2C_CR1_STOP_Pos;

        /* Read data from DR */
        *pData = (uint8_t)I2C1->DR;
        pData++;
        Size--;

        /* Read data from DR */
        *pData = (uint8_t)I2C1->DR;
        pData++;
        Size--;
      }
    }
    else
    {
      I2C_RX_SetTime();
      /* Read data from DR */
      *pData = (uint8_t)I2C1->DR;
      pData++;
      Size--;

      if (I2C_GET_FLAG(I2C_FLAG_BTF) == SET)
      {
        /* Read data from DR */
        *pData = (uint8_t)I2C1->DR;
        pData++;
        Size--;
      }
    }
  }

  return 0;
}

//////////////////////////
uint8_t I2C_Master_TransmitT(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{

  I2C1->CR1 |= 1 << I2C_CR1_PE_Pos;

  /* Disable Pos */
  I2C1->CR1 |= 0 << I2C_CR1_POS_Pos;

  /* Send Slave Address */
  if (I2C_MasterRequestWriteT(DevAddress) != 0)
  {
    return 1;
  }

  // сброс флага адреса
  I2C1->SR2;

  while (Size > 0U)
  {
    /* Write data to DR */
    I2C1->DR = *pData;

    /* Increment Buffer pointer */
    pData++;
    Size--;

    // проверим данные
    if (I2C_TX_SetTime() != 0)
    {
      // откл проверку адреса
      I2C1->CR1 |= 0 << I2C_CR1_ACK_Pos;
      return 1;
    }
  }

  /* Generate Stop */
  I2C1->CR1 |= 1 << I2C_CR1_STOP_Pos;

  return 0;
}

////IRQ
void Error_Handler(void)
{
  //    __disable_irq();
  //    while (1)
  //    {
  //    }
}

// void I2C1_EV_IRQHandler(void)
//{
//	__disable_irq();
//	LED6();
// }
