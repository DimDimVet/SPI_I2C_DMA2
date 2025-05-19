#include "init_USART.h"

void Init_USART1(uint16_t baudRate)
{
	Enable_RCC_USART1();
	Config_GPIO_USART1();
	Config_USART1(baudRate);
}

void Enable_RCC_USART1(void)
{
	RCC->AHB1ENR |= 1 << RCC_AHB1ENR_GPIOAEN_Pos;  // Включаем тактирование порта A
	RCC->APB2ENR |= 1 << RCC_APB2ENR_USART1EN_Pos; // Включаем тактирование Usart1
												   // RCC->AHB1ENR |= 1 << RCC_AHB1ENR_DMA2EN_Pos; // Включаем тактирование DMA2
}

void Config_GPIO_USART1(void)
{
	// PA9 (TX) и PA10 (RX)
	GPIOA->MODER |= 0 << GPIO_MODER_MODER9_Pos; // Очистка режима для PA9
	GPIOA->MODER |= 2 << GPIO_MODER_MODER9_Pos; // Альтернативная функция для PA9 (TX)

	GPIOA->MODER |= 0 << GPIO_MODER_MODER10_Pos; // Очистка режима для PA10
	GPIOA->MODER |= 2 << GPIO_MODER_MODER10_Pos; // Альтернативная функция для PA10 (RX)

	GPIOA->AFR[1] |= 7 << GPIO_AFRH_AFSEL9_Pos;	 // AF7 для USART1 PA9 (TX)
	GPIOA->AFR[1] |= 7 << GPIO_AFRH_AFSEL10_Pos; // AF7 для USART1 PA10 (RX)
}

void Config_USART1(uint16_t baudRate)
{
	USART1->BRR = SystemCoreClock / baudRate; // SystemCoreClock/Baudrate

	USART1->CR1 |= 1 << USART_CR1_TE_Pos;	  // Включить TX
	USART1->CR1 |= 1 << USART_CR1_RE_Pos;	  // Включить RX
	USART1->CR2 |= 2 << USART_CR2_STOP_Pos;	  // Установили STOP бит
	USART1->CR1 |= 1 << USART_CR1_RXNEIE_Pos; // Включить прерывание
											  //    USART1->CR3 |= 1 << USART_CR3_DMAR_Pos;
											  //    USART1->CR3 |= 1 << USART_CR3_DMAT_Pos;

	USART1->CR1 |= 1 << USART_CR1_UE_Pos; // Включить USART1
	NVIC_SetPriority(USART1_IRQn, 2);	  // Установите приоритет
	NVIC_EnableIRQ(USART1_IRQn);		  // Разрешить прерывания для USART2
}

// IRQ
void USART1_IRQHandler(void)
{
	// LED7();
	//
	ExecutorTerminal_USART_Irq();
	//
	// LED7();
}
///

void USART1_ReadChar(char *ch)
{
	if (USART1->SR & USART_SR_RXNE)
	{
		char temp = USART1->DR;

		ch[0] = temp;
	}
}

void USART1_SetString(char *str) // Установка строки по символьно
{
	uint8_t size = strlen(str);

	if (size == 0)
	{
		return;
	}

	for (int i = 0; i < size; i++)
	{
		USART1->DR = str[i];

		while (!(USART1->SR & USART_SR_TXE)) // Проверим окончание передачи
		{
		};
	}
}
