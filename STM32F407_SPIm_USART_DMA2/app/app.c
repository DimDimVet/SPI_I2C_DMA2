#include "app.h"

void ExecutorTerminal_USART_Irq(void)
{

    USART1_ReadChar(receivedChar); // Читаем из консоли

    rezultReadConsol[0] = receivedChar_;

    SPI1_DMA1_SetString(rezultReadConsol);

    rezultReadSPI_[0] = SPI1_DMA1_ReadChar();

    USART1_SetString(rezultReadSPI_);
}

void Executor_SPI_DMA_RX_Irq(void)
{

    LED6();
}

/////////////////

int main(void)
{
    Init_LED();
    Init_USART1(BAUND_RATE);
    Init_SPI();

    while (1)
    {
    }

    return 0;
}
