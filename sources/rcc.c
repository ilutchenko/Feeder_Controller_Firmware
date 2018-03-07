#include <libopencm3/stm32/gpio.h>
#include "rcc.h"

void rcc_init(void){
	/*rcc_clock_setup_in_hsi_out_64mhz();*/
	/*rcc_clock_setup_in_hse_12mhz_out_72mhz();*/
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);
#ifdef USART_CRC
	rcc_periph_clock_enable(RCC_CRC);
#endif

}
