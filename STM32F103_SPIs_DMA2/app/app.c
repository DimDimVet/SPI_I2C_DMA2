#include "app.h"

void Executor_SPI_DMA_RX_Irq()
{
	SPI1_DMA1_ReadChar(receivedCharSPI);

	SPI1_DMA1_SetString(receivedCharSPI);

	if (count_LED_SPI_RX > SIZE_SPI)
	{
		count_LED_SPI_RX = 0;
		LED13();
	}
	else
	{
		count_LED_SPI_RX++;
	}
}

int main()
{
	Init_LED();
	Init_SPI();

	while (1)
	{
		// SPI_TransmitReceive();
	}
	return 0;
}