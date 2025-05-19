#include "init_SPI.h"

void Init_SPI(void)
{
    Enable_RCC_SPI1();
    Config_GPIO_SPI1();
    Config_SPI1();
}

void Enable_RCC_SPI1(void)
{
    RCC->AHB1ENR |= 1 << RCC_AHB1ENR_GPIOBEN_Pos; // Включаем тактирование порта B
    RCC->APB1ENR |= 1 << RCC_APB1ENR_SPI2EN_Pos;  // Включаем тактирование SPI2
    // RCC->AHB1ENR |= 1 << RCC_AHB1ENR_DMA1EN_Pos;  // Включаем тактирование DMA1
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
    // SPI2->CR2 |= 1 << SPI_CR2_RXDMAEN_Pos;// Включаем DMA
    // SPI2->CR2 |= 1 << SPI_CR2_TXDMAEN_Pos;// Включаем DMA
    // SPI2->CR2 = SPI_CR2_RXNEIE; // | SPI_CR2_TXEIE;
    NVIC_EnableIRQ(SPI2_IRQn); // Включаем прерывание SPI2

    SPI2->CR1 |= 1 << SPI_CR1_SPE_Pos; // Вкл SPI
}

// IRQ
void SPI2_IRQHandler(void)
{
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
