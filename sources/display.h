#ifndef DISPLAY_H
#define DISPLAY_H
#include "i2c.h"
#include "usart.h"

#define DISPLAY_ADDR			0x4e
#define DISPLAY_CURS_SPEED_ADDR		0x88

#define COMMAND_D	0x0C
//#define COMMAND_D	0x0
#define DATA_D		0x0D

void display_set_speed (uint32_t i2c, uint8_t *speedStr);
void display_init(uint32_t i2c);

#endif
