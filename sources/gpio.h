#ifndef MY_GPIO_H
#define MY_GPIO_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <string.h>
#include "defines.h"
#include "usart.h"

void gpio_init(void);

#endif
