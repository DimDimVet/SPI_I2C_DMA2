#ifndef APP_H_
#define APP_H_

#include "delay.h"
#include "init_LED.h"
#include "init_SPI.h"

#define SIZE_SPI 10

static uint8_t count_LED_SPI_RX;

static char receivedCharSPI_;
static char *receivedCharSPI = &receivedCharSPI_;

#endif
