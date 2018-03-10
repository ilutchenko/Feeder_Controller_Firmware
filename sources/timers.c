#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include "timers.h"
#include "usart.h"
extern uint16_t motorFreq; 
uint16_t freqCounter = 0;

/*
 *PWM timer
 *Freq: 39 KHz
 *we are using a low level on output pin to drive a MOSFET,
 so PWM is configured with output polarity low.
 */
void tim1_init(void)
{

	/* Enable TIM1 clock. */
	rcc_periph_clock_enable(RCC_TIM1);
	/* Reset TIM1 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM1);


	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
			TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
timer_enable_break_main_output(TIM1);
	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM2 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 5kHz
	 */
	timer_set_prescaler(TIM1, TIMER1_PRESCALER);
	timer_set_period(TIM1, TIMER1_TOP);
	/* Disable preload. */
	timer_disable_preload(TIM1);
	timer_continuous_mode(TIM1);
	timer_enable_oc_preload(TIM1,TIM_OC1);
	timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
	/*Output compare polarity*/
	timer_set_oc_polarity_low(TIM1, TIM_OC1);

	/* Set the initual output compare value for OC1. */
	tim1_set_pwm(START_PWM_VALUE);

	timer_enable_oc_output(TIM1, TIM_OC1);
	/* Enable TIM1 interrupt. */
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM1_UP_IRQ);

	/*Enable timer 1 overflow and compare int */
	timer_enable_irq(TIM1, (TIM_DIER_UIE));
	timer_enable_irq(TIM1, (TIM_DIER_CC1IE));

}

/*
 *Timer for motor's frequency measurement
 *It generates interrupts on rising edges of square wave signal
 *and gets period in input capture 1 register
 */
void tim2_init(void)
{

	rcc_periph_clock_enable(RCC_TIM2);

	/* Timer Base configuration */
	rcc_periph_reset_pulse(RST_TIM2);
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
			TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(TIM2, TIMER2_TOP);
	timer_set_prescaler(TIM2, TIMER2_PRESCALER);

	/*Input capture configuration*/
	/*Enable timer 2 capture init */
	timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
	/*Triggered on rising edge*/
	timer_set_oc_polarity_high(TIM2, TIM_OC1);
	timer_slave_set_trigger(TIM2, TIM_SMCR_TS_TI1FP1);	
	/*Reset at rising edge*/
	timer_slave_set_mode(TIM2, TIM_SMCR_SMS_RM);
	timer_ic_enable(TIM2, TIM_IC1);

	/*Capture 1 interrupt*/
	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_enable_irq(TIM2, (TIM_DIER_CC1IE));

	timer_enable_counter(TIM2);
} 

/*
 *Timer 3 used to generate single impulse to motor breaking circuit
 *Lenth of breaking impulse defines with BREAK_IMPULSE_LENTH (in ms)
 */
void tim3_init(void)
{
	/* Enable TIM1 clock. */
	rcc_periph_clock_enable(RCC_TIM3);
	/* Reset TIM1 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM3);


	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
			TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*Set cycle duty to 3 seconds*/
	timer_set_prescaler(TIM3, TIMER3_PRESCALER);
	timer_set_period(TIM3, TIMER3_TOP);

	/* Disable preload. */
	timer_disable_preload(TIM3);
	timer_one_shot_mode(TIM3);
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
	timer_enable_oc_preload(TIM3,TIM_OC3);
	/*Output compare polarity*/
	timer_set_oc_polarity_high(TIM3, TIM_OC3);

	/* Set the initual output compare value for OC1. */
	tim3_set_pwm(BREAK_IMPULSE_LENTH);
	/*timer_set_oc_value(TIM3, TIM_OC1, BREAK_IMPULSE_LENTH); */

	timer_enable_oc_output(TIM3, TIM_OC1);
	/* Enable TIM3 interrupt. */
	nvic_enable_irq(NVIC_TIM3_IRQ);
	/*nvic_enable_irq(NVIC_TIM3_UP_IRQ);*/

	/*Enable timer 3 overflow and compare int */
	/*timer_enable_irq(TIM3, (TIM_DIER_UIE));*/
	timer_enable_irq(TIM3, (TIM_DIER_CC1IE));
	/*timer_enable_irq(TIM3, (TIM_DIER_UIE));*/

}

void tim1_enable(uint8_t param)
{
	if (param == true)
		/* Counter enable. */
		timer_enable_counter(TIM1);
	else if (param == false)
		timer_disable_counter(TIM1);
}

void tim3_enable(uint8_t param)
{
	if (param == true)
		/* Counter enable. */
		timer_enable_counter(TIM3);
	else if (param == false)
		timer_disable_counter(TIM3);
}
/* @brief Sets PWM duty
 * @param PWM duty in percents
 * */
void tim1_set_pwm (uint8_t pwm)
{
	uint16_t compareVal;
	compareVal = (uint16_t)(TIMER1_TOP * pwm / 100);
	timer_set_oc_value(TIM1, TIM_OC1, compareVal); 
}

/* @brief Sets break pulse duty
 * @param pulse duty in ms
*/
void tim3_set_pwm (uint16_t pwm)
{
	uint16_t compareVal;
	compareVal = (uint16_t)((pwm / 1000) * (TIMER3_TOP / 3));
	timer_set_oc_value(TIM3, TIM_OC1, compareVal); 
}

/*
 *This two interrupts handlers not realy needed in welding controller
 *Writed just for indication with led that timer 1 works properly 
 */
void tim1_up_isr(void)
{
	/* Clear update interrupt flag. */
	timer_clear_flag(TIM1, TIM_SR_UIF);
	/*gpio_set(RED_LED_PORT, RED_LED);*/
	gpio_clear(GREEN_LED_PORT, GREEN_LED);
}

void tim1_cc_isr (void)
{
	/* Clear compare interrupt flag. */
	timer_clear_flag(TIM1, TIM_SR_CC1IF);
	gpio_set(GREEN_LED_PORT, GREEN_LED);
	/*gpio_clear(RED_LED_PORT, RED_LED);*/
}

/*Motor square wave rising edge interrupt*/
void tim2_isr(void)
{
	/*If input capture 1 (rising edge) occurs*/
	if (timer_get_flag(TIM2, TIM_SR_CC1IF))
	{ 
		timer_clear_flag(TIM2, TIM_SR_CC1IF);
		uint16_t freq;
		/* 50 here is timer 2 counting frequency*/
		freq = (uint16_t)(50 * (TIMER2_TOP / TIM_CCR1(TIM2)));
		motorFreq = freq;
		if (freqCounter++ >= 50)
		{
			freqCounter = 0;
			usart_send_string(USART1, "TIM2_INT\n", sizeof("TIM2_INT\n")-1);	
		}
	}
}

/*Break impulse complited interrupt*/
void tim3_isr(void)
{
	if (timer_get_flag(TIM3, TIM_SR_CC1IF))
	{
		timer_clear_flag(TIM3, TIM_SR_CC1IF);
		usart_send_string(USART1, "Break impulse executed\n", sizeof("Break impulse executed\n")-1);

	}
	if (timer_get_flag(TIM3, TIM_SR_UIF))
	{
		timer_clear_flag(TIM3, TIM_SR_UIF);
		usart_send_string(USART1, "TIM3_UP\n", sizeof("TIM3_UP\n")-1);
		tim3_enable(true);
	}
}
