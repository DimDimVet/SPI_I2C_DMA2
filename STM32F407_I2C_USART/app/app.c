#include "app.h"

void ExecutorTerminal_USART_Irq(void)
{

	USART1_ReadChar(receivedChar); // Читаем из консоли
	
	if (count_size_buf >= SIZE_BUF_USART)
	{
		count_size_buf = 0;

		__disable_irq();

		while (I2C_Master_TransmitT(I2C_ADDRESS, (uint8_t *)rezultReadConsol, BUFFER_SIZE_I2C) != 0)
		{
			Error_Handler();
		}

		while (I2C_Master_ReceiveT(I2C_ADDRESS, (uint8_t *)rezultReadI2C, BUFFER_SIZE_I2C) != 0)
		{
			Error_Handler();
		}

		__enable_irq();

		USART1_SetString(rezultReadI2C);
		
		LED7();
	}
	else
	{
		rezultReadConsol[count_size_buf] = receivedChar_;
		count_size_buf++;
	}
}

/////////////////

int main()
{
	Init_LED();
	Init_USART1(BAUND_RATE);
	Init_I2C();

	while (1)
	{
	}

	return 0;
}