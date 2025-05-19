#include "init_SPI.h"

uint8_t data;

int countSPI = 0;
char set_infoStr[SIZE_BUF_SPI] = "F103";
uint8_t dataBufSPI_[SIZE_BUF_SPI];
uint8_t *dataBufSPI = dataBufSPI_;

uint8_t dataBufRxSPI[1];

void Init_SPI()
{
	Enable_RCC_SPI1();
	Config_GPIO_SPI1();
	Config_SPI1();
	Config_SPI1_DMA1();
}

void Enable_RCC_SPI1()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // тактирование GPIOA
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // тактирование SPI1
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;	// Включаем DMA1
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

	SPI1->CR2 |= SPI_CR2_TXDMAEN; // переключили дма на spi - передача, DMAT = Tx
	SPI1->CR2 |= SPI_CR2_RXDMAEN; // переключили дма на spi - чтение, DMAR = Rx

	// SPI1->CR2 = SPI_CR2_RXNEIE; // | SPI_CR2_TXEIE;
	// SPI1->CR2 = SPI_CR2_TXEIE;// | SPI_CR2_TXEIE;
	// NVIC_EnableIRQ(SPI1_IRQn); // Включаем прерывание SPI2

	SPI1->CR1 |= SPI_CR1_SPE; // Вкл SPI
}

void Config_SPI1_DMA1()
{
	// Channel 2 SPI1_RX, Channel 3 SPI1_TX

	DMA1_Channel3->CCR |= 0;
	DMA1_Channel3->CCR &= ~DMA_CCR3_MEM2MEM; // режим памяти в память
	DMA1_Channel3->CCR &= ~DMA_CCR3_PL;		 // уровень приоритета канала
	DMA1_Channel3->CCR &= ~DMA_CCR3_MSIZE_0; // размер памяти
	DMA1_Channel3->CCR &= ~DMA_CCR3_MSIZE_1;
	DMA1_Channel3->CCR &= ~DMA_CCR3_PSIZE_0; // размер периферии
	DMA1_Channel3->CCR &= ~DMA_CCR3_PSIZE_1;
	DMA1_Channel3->CCR |= DMA_CCR3_MINC;		 // Режим приращения памяти
	DMA1_Channel3->CCR &= ~DMA_CCR3_PINC;		 // Режим периферийного приращения
	DMA1_Channel3->CCR &= ~DMA_CCR3_CIRC;		 // Кольцевой режим
	DMA1_Channel3->CCR |= DMA_CCR3_DIR;			 // Направление передачи данных
	DMA1_Channel3->CCR |= DMA_CCR3_TCIE;		 // разрешение прерывания по завершению передачи
	DMA1_Channel3->CPAR = (uint32_t)(&SPI1->DR); // Адрес регистра данных spi
	DMA1_Channel3->CNDTR = 0;					 // размер массива
	DMA1_Channel3->CMAR = 0;					 // Адрес буфера
	DMA1_Channel3->CCR |= DMA_CCR3_EN;			 // Включение канала DMA

	DMA1_Channel2->CCR |= 0;
	DMA1_Channel2->CCR &= ~DMA_CCR2_MEM2MEM; // режим памяти в память
	DMA1_Channel2->CCR &= ~DMA_CCR2_PL;		 // уровень приоритета канала
	DMA1_Channel2->CCR &= ~DMA_CCR2_MSIZE_0; // размер памяти
	DMA1_Channel2->CCR &= ~DMA_CCR2_MSIZE_1;
	DMA1_Channel2->CCR &= ~DMA_CCR2_PSIZE_0; // размер периферии
	DMA1_Channel2->CCR &= ~DMA_CCR2_PSIZE_1;
	DMA1_Channel2->CCR |= DMA_CCR2_MINC;		  // Режим приращения памяти
	DMA1_Channel2->CCR |= DMA_CCR2_PINC;		  // Режим периферийного приращения
	DMA1_Channel2->CCR |= DMA_CCR2_CIRC;		  // Кольцевой режим
	DMA1_Channel2->CCR &= ~DMA_CCR2_DIR;		  // Направление передачи данных
	DMA1_Channel2->CCR |= DMA_CCR2_TCIE;		  // разрешение прерывания по завершению передачи
	DMA1_Channel2->CPAR = (uint32_t)(&SPI1->DR);  // Адрес регистра данных spi
	DMA1_Channel2->CNDTR = 1;					  // размер массива
	DMA1_Channel2->CMAR = (uint32_t)dataBufRxSPI; // Адрес буфера
	DMA1_Channel2->CCR |= DMA_CCR2_EN;			  // Включение канала DMA

	DMA1->IFCR |= DMA_IFCR_CTCIF3;
	DMA1->IFCR |= DMA_IFCR_CTCIF2;

	NVIC_EnableIRQ(DMA1_Channel3_IRQn); // Включение прерываний DMA
	NVIC_EnableIRQ(DMA1_Channel2_IRQn); // Включение прерываний DMA
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
/////spi
void SPI1_DMA1_ReadChar(char *ch) // считываем регистр
{
	ch[0] = dataBufRxSPI[0];
}

void SPI1_DMA1_SetString(char *str) // Установка строки по символьно
{

	DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
	;

	uint8_t sizeTxU = strlen(str);

	DMA1_Channel3->CNDTR = sizeTxU;
	DMA1_Channel3->CMAR = (uint32_t)str;
	DMA1_Channel3->CCR |= DMA_CCR3_EN;
}

/////IRQ
void SPI1_IRQHandler(void)
{

	data = SPI1_ReadBayt(); // test

	SPI1_SetBayt(data); // test

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

void DMA1_Channel3_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF3)
	{
		// Executor_SPI_DMA_TX_Irq();
	}
}

void DMA1_Channel2_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_TCIF2)
	{
		Executor_SPI_DMA_RX_Irq();

		DMA1->IFCR |= DMA_IFCR_CTCIF2; // Очистка флага
	}
}
