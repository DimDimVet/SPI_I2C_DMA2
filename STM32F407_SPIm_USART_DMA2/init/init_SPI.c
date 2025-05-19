#include "init_SPI.h"

void Init_SPI(void)
{
    Enable_RCC_SPI1();
    Config_GPIO_SPI1();
    Config_SPI1();
    Config_SPI1_DMA1();
}

void Enable_RCC_SPI1(void)
{
    RCC->AHB1ENR |= 1 << RCC_AHB1ENR_GPIOBEN_Pos; // Включаем тактирование порта B
    RCC->APB1ENR |= 1 << RCC_APB1ENR_SPI2EN_Pos;  // Включаем тактирование SPI2
    RCC->AHB1ENR |= 1 << RCC_AHB1ENR_DMA1EN_Pos;  // Включаем тактирование DMA1
}

void Config_GPIO_SPI1(void)
{
    // PB13(SCK), PB14(MISO), PB15(MOSI)

    GPIOB->MODER |= 0 << GPIO_MODER_MODE13_Pos; // Очистка режима для PB13
    GPIOB->MODER |= 2 << GPIO_MODER_MODE13_Pos; // Альтернативная функция для PB13(SCK)

    GPIOB->MODER |= 0 << GPIO_MODER_MODE14_Pos; // Очистка режима для PB14
    GPIOB->MODER |= 2 << GPIO_MODER_MODE14_Pos; // Альтернативная функция для PB14(MISO)

    GPIOB->MODER |= 0 << GPIO_MODER_MODE15_Pos; // Очистка режима для PB15
    GPIOB->MODER |= 2 << GPIO_MODER_MODE15_Pos; // Альтернативная функция для PB15(MOSI)

    GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL13_Pos; // AF5 для SPI1 PB13(SCK)
    GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL14_Pos; // AF5 для SPI1 PB14(MOSI)
    GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL15_Pos; // AF5 для SPI1 PB15(MISO)
}

void Config_SPI1(void)
{
    SPI2->CR1 = 0; // reset

    SPI2->CR1 |= 1 << SPI_CR1_MSTR_Pos;     // master
    SPI2->CR1 |= 0 << SPI_CR1_BIDIMODE_Pos; // включение режима двунаправленных данных mode:master
    SPI2->CR1 |= 0 << SPI_CR1_BIDIOE_Pos;   // включение вывода в двунаправленном режиме
    SPI2->CR1 |= 0 << SPI_CR1_CRCEN_Pos;    // аппаратный расчет CRC включен 0
    SPI2->CR1 |= 0 << SPI_CR1_CRCNEXT_Pos;  // следующая передача CRC 0
    SPI2->CR1 |= 0 << SPI_CR1_DFF_Pos;      // 16-битный формат кадра данных0
    SPI2->CR1 |= 0 << SPI_CR1_RXONLY_Pos;   // Только прием mode:slave
    SPI2->CR1 |= 1 << SPI_CR1_SSM_Pos;      // Программное управление mode:master
    SPI2->CR1 |= 1 << SPI_CR1_SSI_Pos;      // Внутренний раб выбор mode:master
    SPI2->CR1 |= 0 << SPI_CR1_LSBFIRST_Pos; // Формат кадра LSB0
    SPI2->CR1 |= 4 << SPI_CR1_BR_Pos;       // f/4
    SPI2->CR1 |= 1 << SPI_CR1_CPOL_Pos;     // начальный фронт
    SPI2->CR1 |= 1 << SPI_CR1_CPHA_Pos;     // фаза...

    SPI2->CR2 = 0;
    SPI2->CR2 |= 1 << SPI_CR2_RXDMAEN_Pos; // Включаем DMA
    SPI2->CR2 |= 1 << SPI_CR2_TXDMAEN_Pos; // Включаем DMA
    // SPI2->CR2 = SPI_CR2_RXNEIE; // | SPI_CR2_TXEIE;
    NVIC_EnableIRQ(SPI2_IRQn); // Включаем прерывание SPI2

    SPI2->CR1 |= 1 << SPI_CR1_SPE_Pos; // Вкл SPI
}

void Config_SPI1_DMA1()
{
    // Stream 3-Channel 0 SPI2_RX, Stream 4 Channel 0 SPI2_TX = 000: channel 0 selected
    DMA1_Stream4->CR = 0;
    DMA1_Stream4->CR |= 0 << DMA_SxCR_CHSEL_Pos;  // Stream 4 Channel 0 SPI2_TX
    DMA1_Stream4->CR |= 0 << DMA_SxCR_MBURST_Pos; // Конфигурация передачи пакета памяти
    DMA1_Stream4->CR |= 0 << DMA_SxCR_PBURST_Pos; // Конфигурация периферийной пакетной передачи
    DMA1_Stream4->CR |= 0 << DMA_SxCR_PL_Pos;     // уровень приоритета
    DMA1_Stream4->CR |= 0 << DMA_SxCR_PINCOS_Pos; // размер смещения периферийного приращения связан с PSIZE
    DMA1_Stream4->CR |= 0 << DMA_SxCR_MSIZE_Pos;  // Размер данных памяти1
    DMA1_Stream4->CR |= 0 << DMA_SxCR_PSIZE_Pos;  // Размер периферийных данных
    DMA1_Stream4->CR |= 1 << DMA_SxCR_MINC_Pos;   // Режим приращения памяти
    DMA1_Stream4->CR |= 0 << DMA_SxCR_PINC_Pos;   // Режим приращения периферийных устройств
    DMA1_Stream4->CR |= 0 << DMA_SxCR_CIRC_Pos;   // кольцевой режим
    DMA1_Stream4->CR |= 1 << DMA_SxCR_DIR_Pos;    // направление передачи данных 00: периферийное устройство-память 01: память-периферийное устройство
    DMA1_Stream4->CR |= 1 << DMA_SxCR_TCIE_Pos;   // Разрешение прерывания завершения передачи
    DMA1_Stream4->PAR = (uint32_t)(&SPI2->DR);    // Адрес регистра данных spi
    DMA1_Stream4->NDTR = 0;                       // размер массива
    DMA1_Stream4->M0AR = 0;                       // Адрес буфера
    DMA1_Stream4->CR |= 1 << DMA_SxCR_EN_Pos;     // включение потока

    DMA1_Stream3->CR = 0;
    DMA1_Stream3->CR |= 0 << DMA_SxCR_CHSEL_Pos;  // Stream 3-Channel 0 SPI2_RX
    DMA1_Stream3->CR |= 0 << DMA_SxCR_MBURST_Pos; // Конфигурация передачи пакета памяти
    DMA1_Stream3->CR |= 0 << DMA_SxCR_PBURST_Pos; // Конфигурация периферийной пакетной передачи
    DMA1_Stream3->CR |= 0 << DMA_SxCR_PL_Pos;     // уровень приоритета
    DMA1_Stream3->CR |= 0 << DMA_SxCR_PINCOS_Pos; // размер смещения периферийного приращения связан с PSIZE
    DMA1_Stream3->CR |= 0 << DMA_SxCR_MSIZE_Pos;  // Размер данных памяти
    DMA1_Stream3->CR |= 0 << DMA_SxCR_PSIZE_Pos;  // Размер периферийных данных
    DMA1_Stream3->CR |= 1 << DMA_SxCR_MINC_Pos;   // Режим приращения памяти
    DMA1_Stream3->CR |= 1 << DMA_SxCR_PINC_Pos;   // Режим приращения периферийных устройств
    DMA1_Stream3->CR |= 1 << DMA_SxCR_CIRC_Pos;   // кольцевой режим
    DMA1_Stream3->CR |= 0 << DMA_SxCR_DIR_Pos;    // направление передачи данных 00: периферийное устройство-память 01: память-периферийное устройство
    DMA1_Stream3->CR |= 1 << DMA_SxCR_TCIE_Pos;   // Разрешение прерывания завершения передачи
    DMA1_Stream3->PAR = (uint32_t)(&SPI2->DR);    // Адрес регистра данных spi
    DMA1_Stream3->NDTR = 1;                       // размер массива
    DMA1_Stream3->M0AR = (uint32_t)dataBufRxSPI;  // Адрес буфера
    DMA1_Stream3->CR |= 1 << DMA_SxCR_EN_Pos;     // включение потока

    DMA1->HIFCR |= 1 << DMA_HIFCR_CHTIF4_Pos;
    DMA1->LIFCR |= 1 << DMA_LIFCR_CTCIF3_Pos;
    NVIC_EnableIRQ(DMA1_Stream4_IRQn); // Включение прерываний DMA
    NVIC_EnableIRQ(DMA1_Stream3_IRQn); // Включение прерываний DMA
}

////
uint8_t SPI2_ReadBayt()
{
    uint8_t temp_bayt;

    if (SPI2->SR & SPI_SR_RXNE)
    {
        temp_bayt = SPI2->DR;
    }

    return temp_bayt;
}

uint8_t SPI2_SetBayt(char byte)
{

    SPI2->DR = byte;

    while (!(SPI2->SR & SPI_SR_TXE))
    {
        while (SPI2->SR & SPI_SR_BSY)
        {
        };
    };

    return 1;
}
/////spi
char SPI1_DMA1_ReadChar() // считываем регистр
{
    return dataBufRxSPI[0];
}

void SPI1_DMA1_SetString(char *str) // Установка строки по символьно
{
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;

    uint8_t sizeTxU = strlen(str);

    DMA1_Stream4->NDTR = sizeTxU;
    DMA1_Stream4->M0AR = (uint32_t)str;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
}

void SPI1_DMA1_SetChar(char *ch) // Установка строки по символьно
{
    DMA1_Stream4->CR &= ~DMA_SxCR_EN;

    DMA1_Stream4->NDTR = 1;
    DMA1_Stream4->M0AR = (uint32_t)ch;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
}

// IRQ
void SPI2_IRQHandler(void)
{
}

void DMA1_Stream4_IRQHandler(void)
{
    // LED7();

    if (DMA1->HISR & DMA_HISR_TCIF4)
    {
        //			Executor_SPI_DMA_TX_Irq();
        DMA1->HIFCR |= DMA_HIFCR_CTCIF4; // Сбрасываем флаг
    }
}

void DMA1_Stream3_IRQHandler(void)
{
    // LED6();

    if (DMA1->LISR & DMA_LISR_TCIF3)
    {
        Executor_SPI_DMA_RX_Irq();

        DMA1->LIFCR |= DMA_LIFCR_CTCIF3; // Сбрасываем флаг
    }
}
