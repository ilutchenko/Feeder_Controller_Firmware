#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include "usart.h"
#include "bq76pl455.h"
uint8_t testdata[8] = {1, 2, 3, 4, 5, 6, 7, 8};
uint16_t voltageArray[16];
static char help_msg[] = "Welding automatic controller: \n   Hardware version: 0.1 \n   Firmware version: 1.0 \n";
static uint8_t resiever1[50];
static uint8_t rec_len1;
static uint8_t resiever2[50];
static uint8_t rec_len2;
USART_t usart1;
USART_t usart2;
USART_t usart3;
extern void gas_set(uint8_t val);
extern void welding_set(uint8_t val);
extern void break_motor(void);

bool usart_init(uint32_t usart, uint32_t baudrate,  bool remap)
{
	switch (usart)
	{
		case (USART1):
			rcc_periph_clock_enable(RCC_USART1);
			/* Enable the USART1 interrupt. */
			nvic_enable_irq(NVIC_USART1_IRQ);
			if (remap)
			{
				AFIO_MAPR |= AFIO_MAPR_USART1_REMAP;
				rcc_periph_clock_enable(RCC_GPIOB);
				rcc_periph_clock_enable(RCC_AFIO);
				gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
						GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_RE_TX);

				gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RE_RX);
			}else{
				rcc_periph_clock_enable(RCC_GPIOA);
				gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
						GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

				gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
			}
			break;

		case (USART2):
			rcc_periph_clock_enable(RCC_USART2);
			nvic_enable_irq(NVIC_USART2_IRQ);
			if (remap)
			{
				AFIO_MAPR |= AFIO_MAPR_USART2_REMAP;
				rcc_periph_clock_enable(RCC_GPIOD);
				rcc_periph_clock_enable(RCC_AFIO);
				gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ,
						GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_RE_TX);

				gpio_set_mode(GPIOD, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RE_RX);
			}else{
				rcc_periph_clock_enable(RCC_GPIOA);
				gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
						GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

				gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);
			}
			break;

		case (USART3):
			rcc_periph_clock_enable(RCC_USART3);
			nvic_enable_irq(NVIC_USART3_IRQ);
			if (remap)
			{
				AFIO_MAPR |= AFIO_MAPR_USART3_REMAP_FULL_REMAP;
				rcc_periph_clock_enable(RCC_GPIOC);
				rcc_periph_clock_enable(RCC_AFIO);
				gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
						GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_FR_TX);

				gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, GPIO_USART3_FR_RX);
			}else{
				rcc_periph_clock_enable(RCC_GPIOB);
				gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
						GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

				gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);
			}
			break;
		default:
			return 0;
	}

	/* Setup UART parameters. */
	usart_set_baudrate(usart, baudrate);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, USART_STOPBITS_1);
	usart_set_parity(usart, USART_PARITY_NONE);
	usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
	usart_set_mode(usart, USART_MODE_TX_RX);

	/* Enable USART Receive interrupt. */
	USART_CR1(usart) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(usart);
	return 1;
}

void usart1_isr(void)
{
	uint8_t tmp;

	//Check if we were called because of RXNE. 
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
			((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		tmp = usart_recv(USART1);
		if (tmp == '\n')
		{
			resiever1[rec_len1++] = 0;	/* Make null-terminated string */
			process_command(resiever1);
			rec_len1 = 0;
		}else{
			resiever1[rec_len1++] = tmp;
		}

		/*USART_CR1(USART1) &= ~USART_CR1_RXNEIE;*/
	}
	//Check if we were called because of TXE. 
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
			((USART_SR(USART1) & USART_SR_TXE) != 0)) {
		/*Check the count of non-sended words*/	
		if (usart1.lenth != 0){
			/*send bytes until it will be send*/
			if (usart1.byte_counter-- != 0){
				usart_send(USART1, *usart1.data_pointer++);
			}else{
				if(--usart1.lenth !=0){
					/*Reconfig usarts pointers and byte array*/
					usart1.byte_counter = 3;
					usart1.global_pointer++;
					usart1.data1 = (*usart1.global_pointer >> 24) & 0xff;
					usart1.data2 = (*usart1.global_pointer >> 16) & 0xff;
					usart1.data3 = (*usart1.global_pointer >> 8) & 0xff;
					usart1.data4 = (*usart1.global_pointer) & 0xff;
					usart1.data_pointer = &usart1.data1;
					usart_send(USART1, *usart1.data_pointer++);
				}else{
					//Disable the TXE interrupt as we don't need it anymore. 
					USART_CR1(USART1) &= ~USART_CR1_TXEIE;
					usart1.busy = 0;
				}
			}

		}else{
			//Disable the TXE interrupt as we don't need it anymore. 
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
			usart1.busy = 0;
		}
	}
}

void usart2_isr(void)
{
	uint8_t tmp;

	//Check if we were called because of RXNE. 
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
			((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		tmp = usart_recv(USART2);
		if (tmp == '\n')
		{
			resiever2[rec_len2++] = 0;	/* Make null-terminated string */
			process_command(resiever2);
			rec_len2 = 0;
		}else{
			resiever2[rec_len2++] = tmp;
		}

		/*USART_CR1(USART1) &= ~USART_CR1_RXNEIE;*/
	}
	//Check if we were called because of TXE. 
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
			((USART_SR(USART2) & USART_SR_TXE) != 0)) {
		/*Check the count of non-sended words*/	
		if (usart2.lenth != 0){
			/*send bytes until it will be send*/
			if (usart2.byte_counter-- != 0){
				usart_send(USART2, *usart2.data_pointer++);
			}else{
				if(--usart2.lenth !=0){
					/*Reconfig usarts pointers and byte array*/
					usart2.byte_counter = 3;
					usart2.global_pointer++;
					usart2.data1 = (*usart2.global_pointer >> 24) & 0xff;
					usart2.data2 = (*usart2.global_pointer >> 16) & 0xff;
					usart2.data3 = (*usart2.global_pointer >> 8) & 0xff;
					usart2.data4 = (*usart2.global_pointer) & 0xff;
					usart2.data_pointer = &usart2.data1;
					usart_send(USART2, *usart1.data_pointer++);
				}else{
					//Disable the TXE interrupt as we don't need it anymore. 
					USART_CR1(USART2) &= ~USART_CR1_TXEIE;
					usart2.busy = 0;
				}
			}

		}else{
			//Disable the TXE interrupt as we don't need it anymore. 
			USART_CR1(USART2) &= ~USART_CR1_TXEIE;
			usart2.busy = 0;
		}
	}
}

void usart_send_string(uint32_t USART, char *BufferPtr, uint16_t Length )
{

	while ( Length != 0 )
	{
		usart_send_blocking(USART, *BufferPtr);
		BufferPtr++;
		Length--;
	}

	return;
}

void usart_send_32(uint32_t USART, uint32_t *data, uint8_t lenth)
{
	while (usart1.busy);
	usart1.busy = 1;	
	//Divide 32bit to 8bit
	usart1.data1 = (*data >> 24) & 0xff;
	usart1.data2 = (*data >> 16) & 0xff;
	usart1.data3 = (*data >> 8) & 0xff;
	usart1.data4 = (*data) & 0xff;
	usart1.lenth = lenth;
	usart1.byte_counter = 4;
	usart1.global_pointer = data;
	usart1.data_pointer = &usart1.data1;	//
	usart_send_blocking(USART, *usart1.data_pointer++);
	usart1.byte_counter--;
	//Enable TxE interrupt
	USART_CR1(USART) |= USART_CR1_TXEIE;
}
/*
 *@brief Processing input commands
 *@param pointer to resieved string
 */
void process_command(char *cmd)
{
	gpio_clear(GREEN_LED_PORT, GREEN_LED);
	uint16_t t;
	if (strncmp(cmd, "LED", 3) == 0)
	{
		gpio_toggle(GREEN_LED_PORT, GREEN_LED);
		usart_send_byte(USART1, 'l');
	}    

	if (strncmp(cmd, START_STRING, strlen(START_STRING)) == 0)
	{
		tim1_enable(true);
		usart_send_string(USART1, "Started\n", strlen("Started\n"));
	}

	if (strncmp(cmd, STOP_STRING, strlen(STOP_STRING)) == 0)
	{
		break_motor();
		usart_send_string(USART1, "Stopped\n", strlen("Stopped\n"));
	}

	if (strncmp(cmd, SET_PWM_STRING, strlen(SET_PWM_STRING)) == 0)
	{
		uint8_t pwmVal = atoi(cmd + 1);
		if (pwmVal <= 100){
			tim1_set_pwm(pwmVal);
			usart_send_string(USART1, "PWM updated\n", strlen("PWM updated\n"));
		}else{
			usart_send_string(USART1, "ERROR\n", strlen("ERROR\n"));
		}
	}

	if (strncmp(cmd, GAS_STRING, strlen(GAS_STRING)) == 0){
		if ( atoi(cmd + 1) == 1 ){
			gas_set(true);
			usart_send_string(USART1, "Gas on\n", strlen("Gas on\n"));
		}else{
			gas_set(false);
			usart_send_string(USART1, "Gas off\n", strlen("Gas off\n"));
		}
	}

	if (strncmp(cmd, WELDING_STRING, strlen(WELDING_STRING)) == 0){
		if ( atoi(cmd + 1) == 1 ){
			welding_set(true);
		usart_send_string(USART1, "Welding on\n", strlen("Welding onf\n"));
		}else{
			welding_set(false);
		usart_send_string(USART1, "Welding off\n", strlen("Welding off\n"));
		}
	}

	/* Manual  */
	if (strncmp(cmd, "info", 4) == 0)
	{
		usart_send_string(USART1, help_msg, sizeof(help_msg)-1);
	}
}

void usart_send_data (uint32_t USART, uint32_t *data, uint8_t lenth)
{
	while (usart1.busy);
	usart1.busy = 1;	
	usart1.lenth = lenth;
	usart1.byte_counter = 4;
	usart1.global_pointer = data;
	usart1.data_pointer = &usart1.data1;	//
	usart_send_blocking(USART, *usart1.data_pointer++);
	usart1.byte_counter--;
	//Enable TxE interrupt
	USART_CR1(USART) |= USART_CR1_TXEIE;
}

void usart_send_byte (uint32_t USART, uint8_t data)
{

	while (usart1.busy);
	usart_send_blocking(USART, data);
}

double atof (const char *s)
{
	// This function stolen from either Rolf Neugebauer or Andrew Tolmach. 
	// Probably Rolf.
	double a = 0.0;
	int e = 0;
	int c;
	uint8_t neg_flag = 0;
	if ((c = *s++) == '-')
	{
		neg_flag = 1;
	}
	while ((c = *s++) != '\0' && isdigit(c)) {
		a = a*10.0 + (c - '0');
	}
	if (c == '.') {
		while ((c = *s++) != '\0' && isdigit(c)) {
			a = a*10.0 + (c - '0');
			e = e-1;
		}
	}
	if (c == 'e' || c == 'E') {
		int sign = 1;
		int i = 0;
		c = *s++;
		if (c == '+')
			c = *s++;
		else if (c == '-') {
			c = *s++;
			sign = -1;
		}
		while (isdigit(c)) {
			i = i*10 + (c - '0');
			c = *s++;
		}
		e += i*sign;
	}
	while (e > 0) {
		a *= 10.0;
		e--;
	}
	while (e < 0) {
		a *= 0.1;
		e++;
	}
	if (neg_flag == 1)
		a = a*(-1);
	return a;
}

void ftoa (float num, uint8_t *str, uint8_t precision)
{
    int intpart = num;
    int intdecimal;
    int i;
    float decimal_part;
    char decimal[20];

    memset(str, 0x0, 20);
    if (num > (-1) && num < (0))
    {
        strcat(str, "-");
        itoa(num, str+1, 10);
    }else{
        itoa(num, str, 10);
    }
    strcat(str, ".");

    decimal_part = num - intpart;
    intdecimal = decimal_part * 1000000;

    if(intdecimal < 0)
    {
        intdecimal = -intdecimal;
    }
    itoa(intdecimal, decimal, 10);
    for(i =0;i < (precision - strlen(decimal));i++)
    {
        strcat(str, "0");
    }
    strcat(str, decimal);
}

