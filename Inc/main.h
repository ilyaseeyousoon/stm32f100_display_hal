#ifndef MAIN
#define MAIN

#include "stm32f1xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#define KS0108_SCREEN_WIDTH		128
#define KS0108_SCREEN_HEIGHT	64
#define DISPLAY_SET_Y       0x40
#define DISPLAY_SET_X       0xB8
#define DISPLAY_START_LINE  0xC0
#define DISPLAY_ON_CMD		0x3E



#endif