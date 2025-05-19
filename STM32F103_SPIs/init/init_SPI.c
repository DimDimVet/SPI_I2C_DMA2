#include "init_SPI.h"

uint8_t data;

int countSPI = 0;
char set_infoStr[SIZE_BUF_SPI] = "F103";
uint8_t dataBufSPI_[SIZE_BUF_SPI];
uint8_t *dataBufSPI = dataBufSPI_;

void Init_SPI()
{
	Enable_RCC_SPI1();
	Config_GPIO_SPI1();
	Config_SPI1();
}

void Enable_RCC_SPI1()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // тактирование GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // тактирование SPI1
}

void Config_GPIO_SPI1()
{

	// Настраиваем PA5 (SCK), PA6 (MISO), PA7 (MOSI) как альтернативные функции

	GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5); // reset
	GPIOA->CRL |= GPIO_CRL_MODE5_0;					 // PA5 2MHz
	GPIOA->CRL |= GPIO_CRL_CNF5_0;					 // PA5 alt SCK

	GPIOA->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7); // reset
	GPIOA->CRL |= GPIO_CRL_MODE7_0;					 // PA7 2MHz
	GPIOA->CRL |= GPIO_CRL_CNF7_1;					 // PA7 alt MOSI

	GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
	GPIOA->CRL |= GPIO_CRL_MODE6_0;	 // PA7 2MHz
	GPIOA->CRL |= (GPIO_CRL_CNF6_1); // PA6 input MISO

	// Настраиваем PA4 (nSS) как вход, особеность slave SPI
	GPIOA->CRL &= ~GPIO_CRL_CNF4;	// сбрасываем настройки
	GPIOA->CRL |= GPIO_CRL_MODE4_0; // режим входа с подтяжкой
}

void Config_SPI1()
{
	SPI1->CR1 = 0; // reset

	SPI1->CR1 &= ~SPI_CR1_MSTR;		// master1
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE; // включение режима двунаправленных данных mode:master
	SPI1->CR1 &= ~SPI_CR1_BIDIOE;	// включение вывода в двунаправленном режиме
	SPI1->CR1 &= ~SPI_CR1_CRCEN;	// аппаратный расчет CRC включен
	SPI1->CR1 &= ~SPI_CR1_CRCNEXT;	// следующая передача CRC
	SPI1->CR1 &= ~SPI_CR1_DFF;		// 16-битный формат кадра данных master1
	SPI1->CR1 &= ~SPI_CR1_RXONLY;	// Только прием mode:slave
	SPI1->CR1 &= ~SPI_CR1_SSM;		// Программное управление mode:master1
	SPI1->CR1 &= ~SPI_CR1_SSI;		// Внутренний раб выбор mode:master1
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // Формат кадра LSB
	SPI1->CR1 &= ~SPI_CR1_BR_0;		// f/4
	SPI1->CR1 &= ~SPI_CR1_BR_1;		// f/4
	SPI1->CR1 &= ~SPI_CR1_BR_2;		// f/4
	SPI1->CR1 |= SPI_CR1_CPOL;		// начальный фронт
	SPI1->CR1 |= SPI_CR1_CPHA;		// фаза...

	// SPI1->CR2 |=SPI_CR2_TXDMAEN;//переключили дма на spi - передача, DMAT = Tx
	// SPI1->CR2 |=SPI_CR2_RXDMAEN;//переключили дма на spi - чтение, DMAR = Rx

	SPI1->CR2 = SPI_CR2_RXNEIE;
	// SPI1->CR2 = SPI_CR2_TXEIE;
	NVIC_EnableIRQ(SPI1_IRQn); // Включаем прерывание SPI2

	SPI1->CR1 |= SPI_CR1_SPE; // Вкл SPI
}

//////////////
uint8_t SPI1_ReadBayt() // считываем регистр
{
	uint8_t temp_bayt;

	if (SPI1->SR & SPI_SR_RXNE)
	{
		temp_bayt = SPI1->DR;
	}

	return temp_bayt;
}

void SPI1_SetBayt(uint8_t byte) // Установка строки по символьно
{
	SPI1->DR = byte; // Записываем новое значение в DR

	while (!(SPI1->SR & SPI_SR_TXE))
	{
		while (SPI1->SR & SPI_SR_BSY)
		{
		};
	};
}

//////////////
uint8_t SPI_TransmitReceive()
{
	uint8_t data;

	if (SPI1->SR & SPI_SR_RXNE)
	{
		data = SPI1->DR;
		while (!(SPI1->SR & SPI_SR_TXE))
		{
			while (SPI1->SR & SPI_SR_BSY)
			{
			};
		}; // Ждём, пока TXE станет 1
		SPI1->DR = data;

		return SPI1->DR;
	}
	else
	{
		return SPI1->DR;
	}
}
///
void ExecutorData(uint8_t *data)
{
	uint8_t size = strlen(set_infoStr);
	for (int i = 0; i < size; i++)
	{
		data[i] = set_infoStr[i];
	}

	for (int i = 0; i < SIZE_BUF_SPI; i++)
	{
		SPI1_SetBayt(data[i]);
	}
	__enable_irq();
}

/////IRQ
void SPI1_IRQHandler(void)
{
	data = SPI1_ReadBayt(); // test

	SPI1_SetBayt(data); // test
	
	if (count_LED_SPI_RX > SIZE_BUF_SPI)
		{
			count_LED_SPI_RX = 0;
			LED13();
		}
	else
		{
			count_LED_SPI_RX++;
		}


	//	if((countSPI >= SIZE_BUF_SPI))
	//	{
	//		__disable_irq();
	//		countSPI=0;

	//		ExecutorData(dataBufSPI_);
	//	}
	//	else
	//	{
	//		dataBufSPI_[countSPI]=SPI1_ReadBayt();
	//		countSPI++;
	//	}
}
