#ifndef GPIO_H
#define GPIO_H
#include <libopencm3/stm32/rcc.h>
#include "gpio.h"

void gpio_init(){

	/* Enable GPIO clock. */
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_set_mode(WELD_GUN_PORT, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, WELD_GUN_PIN);
	gpio_set_mode(SWITCH_PORT, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, SWITCH_PIN);
	/*gpio_set_mode(WELD_GUN_PORT, GPIO_MODE_OUTPUT_50_MHZ,*/
			/*GPIO_CNF_OUTPUT_PUSHPULL, WELD_GUN_PIN);*/
	/* Enable led as output */
	/*Green for BLuePill*/
	gpio_set_mode(GREEN_LED_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GREEN_LED);

	/*PWM output pin PA8*/
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      GPIO_TIM1_CH1);
	/*Break output pin*/
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      GPIO_TIM3_CH1);
	/*Frequency measurement input pin PA15*/
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_TIM2_REMAP_PARTIAL_REMAP1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_INPUT_FLOAT,
		      GPIO_TIM2_FR_CH1_ETR);
	/*Enable pins for gas, welding, motor breaking*/
	gpio_set_mode(GAS_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GAS_PIN);
	gpio_set_mode(WELD_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, WELD_PIN);
	gpio_set_mode(BREAK_PORT, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, BREAK_PIN);

}

/*
 *void gpio_isr()
 *{
 *        
 *}
 */
#endif
