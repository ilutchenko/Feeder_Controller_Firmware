#include <stdint.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include "sources/gpio.h"
#include "sources/rcc.h"
#include "sources/usart.h"
#include "sources/timers.h"
#include "sources/adc.h"
#include "sources/defines.h"

uint16_t motorFreq; 
uint8_t sequence = 0;
uint8_t task = 0;
extern uint16_t weldExtiInt;
extern uint8_t weldPinStatus;
void process_task(void);
void initiate_start_sequence(void);
void initiate_stop_sequence(void);
void gas_set(uint8_t val);
void break_set(uint8_t val);
void welding_set(uint8_t val);
void break_motor(void);
uint8_t channel_array[16];
int main(void)
{
	rcc_init();
	gpio_init();
	gas_set(false);
	welding_set(false);
	break_set(false);

	usart_init(USART1, 115200, false);

	tim1_init();
	tim2_init();
	tim3_init();
	tim4_init();
	adc_init();
	/* Select the channel we want to convert. 16=temperature_sensor. */
	channel_array[0] = 16;
	/* Set the injected sequence here, with number of channels */
	adc_set_regular_sequence(ADC1, 1, channel_array);

	channel_array[0] = 8;
	/* Set the injected sequence here, with number of channels */
	adc_set_regular_sequence(ADC2, 1, channel_array);
	usart_send_string(USART1, "Welding controller started \n", strlen("Welding controller started \n"));
	systick_setup();

	int i;
	while (1) {
		for (i = 0; i < 800000; i++)	/* Wait a bit. */
			__asm__("nop");
	}
	return 0;

}
void sys_tick_handler(void){
uint16_t adcVal;
float div;
	/*PID regulation here?*/
	if(!gpio_get(SWITCH_PORT, SWITCH_PIN))
	{
		adcVal = adc_get();
		div = adcVal / 4095.0;
		div = div * 100;
		adcVal = (uint16_t)div;
		tim1_set_pwm(adcVal);

		if (weldExtiInt != 0)
		{
			/*usart_send_string(USART1, "Exti != 0\n", 10);*/
			if (weldExtiInt-- == 1)
			{
				if (!gpio_get(WELD_GUN_PORT, WELD_GUN_PIN))
				{
					initiate_start_sequence();
				}else{
					initiate_stop_sequence();
				}
			}
		}
	}
}
/*Gas, break and welding are low-active circuits*/
void gas_set(uint8_t val)
{
	if (val == true)
		gpio_clear(GAS_PORT, GAS_PIN);
	else 
		gpio_set(GAS_PORT, GAS_PIN);
}

void welding_set(uint8_t val)
{
	if (val == true)
		gpio_clear(WELD_PORT, WELD_PIN);
	else
		gpio_set(WELD_PORT, WELD_PIN);
}

void break_set(uint8_t val)
{
	if (val == true)
		gpio_clear(BREAK_PORT, BREAK_PIN);
	else
		gpio_set(BREAK_PORT, BREAK_PIN);
}
void break_motor(void)
{
	timer_disable_break_main_output(TIM1);
	break_set(true);
	tim1_enable(false);
	tim3_enable(true);
}
void initiate_start_sequence(void)
{
	gas_set(true);
	sequence = START_SEQUENCE;
	usart_send_string(USART1, "START GAS_TASK\n", strlen("START GAS_TASK\n"));
	task = WELD_TASK;
	tim4_set_pwm(500);
	tim4_enable(true);
}

void initiate_stop_sequence(void)
{
	break_motor();
	sequence = STOP_SEQUENCE;
	usart_send_string(USART1, "STOP MOTOR_TASK\n", strlen("STOP MOTOR_TASK\n"));
	task = WELD_TASK;
	tim4_set_pwm(100);
	tim4_enable(true);
}

void process_task(void)
{
	if (sequence == START_SEQUENCE)
	{
		switch (task)
		{
			case (WELD_TASK):
				usart_send_string(USART1, "START WELD_TASK\n", strlen("START WELD_TASK\n"));
				welding_set(true);
				task = MOTOR_TASK;
				tim4_set_pwm(100);
				tim4_enable(true);
				break;
			case (MOTOR_TASK):
				usart_send_string(USART1, "START MOTOR_TASK\n", strlen("START MOTOR_TASK\n"));
				break_set(false);
				tim1_enable(true);
				timer_enable_break_main_output(TIM1);
				break;
		}
	}else if (sequence == STOP_SEQUENCE){
		switch (task)
		{
			case (WELD_TASK):
				usart_send_string(USART1, "STOP WELD_TASK\n", strlen("STOP WELD_TASK\n"));
				welding_set(false);
				task = GAS_TASK; 
				tim4_set_pwm(500);
				tim4_enable(true);
				break;
			case (GAS_TASK):
				usart_send_string(USART1, "STOP GAS_TASK\n", strlen("STOP GAS_TASK\n"));
				gas_set(false);
				break;
		}
	}
}
