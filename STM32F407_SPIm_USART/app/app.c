#include "app.h"

void ExecutorTerminal_USART_Irq(void)
{
    char rezultReadSPI_[SIZE_BUF_USART];

    USART1_ReadChar(receivedChar); // Читаем из консоли

    if (count_size_buf >= SIZE_BUF_USART)
    {
        count_size_buf = 0;

        for (int i = 0; i < SIZE_BUF_USART; i++)
        {
            SPI2_SetBayt(rezultReadConsol[i]);

            rezultReadSPI_[i] = SPI2_ReadBayt();
        }

        USART1_SetString(rezultReadSPI_);
				LED6();
    }
    else
    {
        rezultReadConsol[count_size_buf] = receivedChar_;
        count_size_buf++;
    }
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
