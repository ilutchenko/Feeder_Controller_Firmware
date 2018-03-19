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
	/*PID regulation here?*/
	if(!gpio_get(SWITCH_PORT, SWITCH_PIN))
	{
		adcVal = adc_get();
		adcVal = (uint16_t)(adcVal /0x0FFF * 100);
		if(!gpio_get(WELD_GUN_PORT, WELD_GUN_PIN))
		{
			tim1_set_pwm(adcVal);	// look like don't work again =)		
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
	/*gpio_set(BREAK_PORT, BREAK_PIN);*/
}
