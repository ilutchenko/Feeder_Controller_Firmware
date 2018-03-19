#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "usart.h"
#include "timers.h"
#include "adc.h"
static uint8_t help_msg[] = "Welding automatic controller: \n   Hardware version: 0.1 \n   Firmware version: 1.0 \n";
static uint8_t resiever1[50];
static uint8_t rec_len1;
static uint8_t resiever2[50];
static uint8_t rec_len2;
USART_t usart1;
USART_t usart2;
USART_t usart3;
extern void gas_set(uint8_t val);
extern void break_set(uint8_t val);
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
			process_command(resiever1, rec_len1-2);
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
			process_command(resiever2, rec_len2-2);
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
					usart_send(USART2, *usart2.data_pointer++);
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
	uint8_t *strPointer = BufferPtr;
	uint8_t strCRCpointer[4];
	uint8_t len = Length;	
	uint32_t strCRC;

#ifdef USART_CRC
	uint8_t len = Length - 1;	/*Ignore \n symbol*/
	/*But allocate memory for string with \n (+4 for CRC and +1 for \n)*/
	strPointer = (uint8_t) malloc(len + 5);
	memcpy(strPointer, BufferPtr, len);

	strCRC = crc_calculate_block(BufferPtr, len-4);
	strCRCpointer[0] = strCRC >> 24; 	
	strCRCpointer[1] = strCRC >> 16; 	
	strCRCpointer[2] = strCRC >> 8; 	
	strCRCpointer[3] = strCRC & 0xFF; 	
	/*itoa(strCRC, strCRCpointer, 10);*/

	/*Add 4 CRC bytes to string*/
	strncat(strPointer, strCRCpointer, 4);
	strncat(strPointer, "\n", 1);

	/*If CRC defined, we should send more bytes*/
	len = Length + 4;
#endif
	while ( len != 0 )
	{
		usart_send_blocking(USART, *strPointer);
		strPointer++;
		len--;
	}

#ifdef USART_CRC
	free(strPointer);
#endif
	return;
}

void usart_send_32(uint32_t USART, uint32_t *data, uint8_t lenth)
{
	if (USART == USART1){
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
	}else if (USART == USART2){
		while (usart2.busy);
		usart2.busy = 1;	
		//Divide 32bit to 8bit
		usart2.data1 = (*data >> 24) & 0xff;
		usart2.data2 = (*data >> 16) & 0xff;
		usart2.data3 = (*data >> 8) & 0xff;
		usart2.data4 = (*data) & 0xff;
		usart2.lenth = lenth;
		usart2.byte_counter = 4;
		usart2.global_pointer = data;
		usart2.data_pointer = &usart2.data1;	//
		usart_send_blocking(USART, *usart2.data_pointer++);
		usart2.byte_counter--;
		//Enable TxE interrupt
		USART_CR1(USART) |= USART_CR1_TXEIE;
	}
}
/*
 *@brief Processing input commands
 *@param pointer to resieved string
 *@param length of resieved command without \n symbol
 */ 
uint8_t process_command(uint8_t *cmd, uint8_t cmdLength)
{

#ifdef USART_CRC
	uint32_t resCRC;
	/*resCRC = atoi(cmd+cmdLength-4);*/
	resCRC = (&(cmd+cmdLength-3) << 24 | &(cmd+cmdLength-2) << 16
			| &(cmd+cmdLength-1) << 8 | &(cmd+cmdLength));
	if (resCRC != crc_calculate_block(cmd, cmdLength-4))
	{
		return -1;
	}

#endif
	if (strncmp(cmd, "LED", 3) == 0)
	{
		gpio_toggle(GREEN_LED_PORT, GREEN_LED);
		usart_send_byte(USART1, 'l');
		return 0;
	}    

	if (strncmp(cmd, START_STRING, strlen(START_STRING)) == 0)
	{
		break_set(false);
		tim1_enable(true);
		timer_enable_break_main_output(TIM1);
		usart_send_string(USART1, "Started\n", strlen("Started\n"));
		return 0;
	}

	if (strncmp(cmd, STOP_STRING, strlen(STOP_STRING)) == 0)
	{
		break_motor();
		usart_send_string(USART1, "Stopped\n", strlen("Stopped\n"));
		return 0;
	}

	if (strncmp(cmd, SET_PWM_STRING, strlen(SET_PWM_STRING)) == 0)
	{
		uint8_t pwmVal = atoi(cmd + strlen(SET_PWM_STRING) + 1);
		usart_send_byte(USART1, pwmVal);
		if (pwmVal <= 100){
			tim1_set_pwm(pwmVal);
			usart_send_string(USART1, "PWM updated\n", strlen("PWM updated\n"));
		}else{
			usart_send_string(USART1, "ERROR\n", strlen("ERROR\n"));
		}
		return 0;
	}

	if (strncmp(cmd, GAS_STRING, strlen(GAS_STRING)) == 0){
		if ( atoi(cmd + strlen(GAS_STRING) + 1) == 1 ){
			gas_set(true);
			usart_send_string(USART1, "Gas on\n", strlen("Gas on\n"));
		}else{
			gas_set(false);
			usart_send_string(USART1, "Gas off\n", strlen("Gas off\n"));
		}
		return 0;
	}

	if (strncmp(cmd, WELDING_STRING, strlen(WELDING_STRING)) == 0){
		if ( atoi(cmd + strlen(WELDING_STRING) + 1) == 1 ){
			welding_set(true);
			usart_send_string(USART1, "Welding on\n", strlen("Welding onf\n"));
		}else{
			welding_set(false);
			usart_send_string(USART1, "Welding off\n", strlen("Welding off\n"));
		}
		return 0;
	}
	if(strncmp(cmd, "TEMP", 4) == 0)
	{
		adc_get_temperature();
		/*gpio_toggle(GREEN_LED_PORT, GREEN_LED);*/
		/*UART0_send("\nStarted\n", 9);*/
	}    
	if(strncmp(cmd, "ADC", 3) == 0)
	{
		uint16_t t = 0;
		t = adc_get();
		usart_send_byte(USART1, (t >> 8) & 0xFF);
		usart_send_byte(USART1, t & 0xFF);
		/*gpio_toggle(GREEN_LED_PORT, GREEN_LED);*/
		/*UART0_send("\nStarted\n", 9);*/
	}    
	/* Manual  */
	if (strncmp(cmd, "info", 4) == 0)
	{
		usart_send_string(USART1, help_msg, sizeof(help_msg)-1);
		return 0;
	}

	/*Return that command wasn't recognised*/
	return -1;
}

void usart_send_data (uint32_t USART, uint32_t *data, uint8_t lenth)
{
	if (USART == USART1)
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
	}else if (USART == USART2){
		while (usart2.busy);
		usart2.busy = 1;	
		usart2.lenth = lenth;
		usart2.byte_counter = 4;
		usart2.global_pointer = data;
		usart2.data_pointer = &usart2.data1;	//
		usart_send_blocking(USART, *usart2.data_pointer++);
		usart2.byte_counter--;
		//Enable TxE interrupt
		USART_CR1(USART) |= USART_CR1_TXEIE;
	}
}

void usart_send_byte (uint32_t USART, uint8_t data)
{
	if (USART == USART1)
	{
		while (usart1.busy);
		usart_send_blocking(USART, data);
	}else if (USART == USART2){
		while (usart2.busy);
		usart_send_blocking(USART, data);
	}
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
	uint16_t intpart = num;
	int16_t intdecimal;
	uint16_t i;
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
	for(i = 0; i < (precision - strlen(decimal)); i++)
	{
		strcat(str, "0");
	}
	strcat(str, decimal);
}
void convertBaseVersion(uint16_t input, int base, char *output, int digits)
{
	int i, remainder;
	char digitsArray[17] = "0123456789ABCDEF";


	for (i = digits; i > 0; i--)
	{
		remainder = input % base;
		input = input / base;
		output[i - 1] = digitsArray[remainder];
	}
	output[digits] = '\0';
}
