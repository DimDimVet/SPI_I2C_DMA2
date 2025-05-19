#include "app.h"

void ProcessingData()
{
	uint8_t size = strlen(info_str);

	for (int i = 0; i < size; i++)
	{
		rezultReadI2C[i] = info_str[i];
	}
}

int main()
{
	Init_LED();
	Init_I2C();

	while (1)
	{
		if (I2C_Slave_Receive(rezultReadI2C, SIZE_BUF) != 0)
		{
		}
		else
		{
			I2C1->CR2 |= I2C_CR2_ITEVTEN;
		}

		if (I2C_Slave_Transmit(rezultReadI2C, SIZE_BUF) != 0)
		{
			Error_Handler();
		}

		delay_us(1000);
	}
	return 0;
}
