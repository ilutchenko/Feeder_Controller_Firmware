#ifndef ADC_H
#define ADC_H

#include "gpio.h"
#include "defines.h"
//#include "usart.h"

void adc_init(void); 
uint16_t adc_get(void); 
void adc_get_temperature(void); 

#endif
