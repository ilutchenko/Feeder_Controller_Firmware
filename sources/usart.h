#ifndef MY_USART_H
#define MY_USART_H

#include <libopencm3/stm32/usart.h>
#include "gpio.h"
#include "defines.h"
#define START_STRING	"start"
#define STOP_STRING	"stop"
#define SET_PWM_STRING	"pwm"
#define GAS_STRING	"gas"
#define WELDING_STRING	"weld"
typedef struct{
	uint32_t *global_pointer;
	uint8_t *data_pointer;
	uint8_t data1;
	uint8_t data2;
	uint8_t data3;
	uint8_t data4;
	uint8_t lenth;
	uint8_t byte_counter;
	unsigned int busy:1;
}USART_t;
bool usart_init(uint32_t usart, uint32_t baudrate,  bool remap);
void usart_send_32(uint32_t , uint32_t * , uint8_t );
void process_command(uint8_t *cmd);
void usart_send_data(uint32_t USART, uint32_t *data, uint8_t lenth);
void usart_send_byte(uint32_t USART, uint8_t data);
void usart_send_string(uint32_t USART, char *BufferPtr, uint16_t Length);
void my_usart_print_int(uint32_t usart, int value);
uint32_t usart_crc_calc(uint32_t data);
void ftoa(float num, uint8_t *str, uint8_t precision);


#endif
