#ifndef GPIO_H
#define GPIO_H
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "gpio.h"

void gpio_init(){

	/* Enable GPIO clock. */
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	/* Enable led as output */
	gpio_set_mode(GREEN_LED_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GREEN_LED);
	gpio_set_mode(RED_LED_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, RED_LED);
	gpio_set_mode(YELLOW_LED_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, YELLOW_LED);

	/*Enable pins for multiplexers*/

	gpio_set_mode(GAS_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GAS_PIN);
	gpio_set_mode(WELD_PORT, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, WELD_PIN);
	/* Toggle LED to indicate compare event. */
	gpio_set(GREEN_LED_PORT, GREEN_LED);
}
#endif
