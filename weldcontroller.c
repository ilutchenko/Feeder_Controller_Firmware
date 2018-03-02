#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include "sources/gpio.h"
#include "sources/rcc.h"
#include "sources/adc.h"
#include "sources/usart.h"
#include "sources/timers.h"
#include "sources/defines.h"
#include <string.h>
uint8_t	upcount = 1;
uint32_t temp = 0x12345678;
uint8_t temp2;
uint16_t compare_time = 30000;

void gas_set(uint8_t val);
void welding_set(uint8_t val);
void break_motor(void);
int main(void)
{
	rcc_init();
	gpio_init();
	usart_init(USART1, 115200, false);

	adc_init();

	tim1_init();
	/*tim1_enable(true);*/
	/*tim2_init();*/
	usart_send_string(USART1, "Welding controller started \n", strlen("Welding controller started \n"));
	/*gpio_clear(GREEN_LED_PORT, GREEN_LED);*/

	int i;
        while (1) {
                for (i = 0; i < 800000; i++)	/* Wait a bit. */
			__asm__("nop");
	}
	return 0;
}
void sys_tick_handler(void){
/*PID regulation here?*/
}

void gas_set(uint8_t val)
{
	if (val == true)
		gpio_set(GAS_PORT, GAS_PIN);
	else 
		gpio_clear(GAS_PORT, GAS_PIN);
}

void welding_set(uint8_t val)
{
	if (val == true)
		gpio_set(WELD_PORT, WELD_PIN);
	else
		gpio_clear(WELD_PORT, WELD_PIN);
}

void break_motor(void)
{
	tim1_enable(false);
	gpio_set(BREAK_PORT, BREAK_PIN);
}
