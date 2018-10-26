#ifndef I2C_H
#define I2C_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include "display.h"
#include "rcc.h"

#define NUM_OF_INIT_CMDS	14	
typedef struct iic
{
	uint8_t address;
	uint8_t reg_addr; 
	uint8_t lenth;
	uint8_t *data_pointer;
	uint8_t init_cmds[20];
	uint8_t init_seq;
	unsigned int busy:1;
	unsigned int r:1;
	unsigned int w:1;
	unsigned int rs:1;
};
void i2c2_setup(void);
void i2c2_process_sequence(void);



#endif
