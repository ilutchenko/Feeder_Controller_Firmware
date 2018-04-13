#include <stdint.h>
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
#include <string.h>
uint16_t motorFreq; 
extern uint16_t weldExtiInt;
extern uint8_t weldPinStatus;

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

	tim1_init();
	/*tim1_enable(true);*/
	tim2_init();
	tim3_init();
	adc_init();
	/* Select the channel we want to convert. 16=temperature_sensor. */
	channel_array[0] = 16;
	/* Set the injected sequence here, with number of channels */
	adc_set_regular_sequence(ADC1, 1, channel_array);

	channel_array[0] = 8;
	/* Set the injected sequence here, with number of channels */
	adc_set_regular_sequence(ADC2, 1, channel_array);
	usart_init(USART1, 115200, false);
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
		/*adcVal = (uint16_t)(adcVal /0x0FFF * 100);*/
		div = adcVal / 4095.0;
		div = div * 100;
		adcVal = (uint16_t)div;
		/*
		 *usart_send_string(USART1, "ADC: ", 5);
		 *usart_send_byte(USART1, adcVal >> 8);
		 *usart_send_byte(USART1, adcVal & 0xFF);
		 *usart_send_string(USART1, "\n", 1);
		 */

		if (weldExtiInt != 0)
		{
			/*usart_send_string(USART1, "Exti != 0\n", 10);*/
			if (weldExtiInt-- == 1)
			{
				/*usart_send_string(USART1, "Exti = 1\n", 9);*/
				if (gpio_get(WELD_GUN_PORT, WELD_GUN_PIN) == weldPinStatus)
				{
					/*usart_send_string(USART1, "WPST\n", 5);*/
					if (gpio_get(WELD_GUN_PORT, WELD_GUN_PIN) == 0)
					{
						initiate_start_sequence();
					}else{
						initiate_stop_sequence();
					}
				}
			}
		}
		/*
		 *if(!gpio_get(WELD_GUN_PORT, WELD_GUN_PIN))
		 *{
		 *        [>usart_send_string(USART1, "Button pressed\n", strlen("Button pressed\n"));<]
		 *        tim1_set_pwm(adcVal);	// look like don't work again =)		
		 *}
		 */
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
	/*gpio_set(BREAK_PORT, BREAK_PIN);*/
}
void initiate_start_sequence(void)
{
	usart_send_string(USART1, "START\n", 6);
}

void initiate_stop_sequence(void)
{
	usart_send_string(USART1, "STOP\n", 5);
}
