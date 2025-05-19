#ifndef APP_H_
#define APP_H_

#include "delay.h"
#include "init_LED.h"
#include "init_SPI.h"
#include "init_USART.h"

#define BAUND_RATE 9600

static char rezultReadConsol_[SIZE_BUF_USART];
static char *rezultReadConsol = rezultReadConsol_;

static char rezultReadSPI_[SIZE_BUF_USART];
static char *rezultReadSPI = rezultReadSPI_;

static char receivedChar_;
static char *receivedChar = &receivedChar_;

#endif
