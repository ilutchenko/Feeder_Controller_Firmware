#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include "usart.h"


static char help_msg[] = "Feeder control system: \n   Hardware version: 0.1 \n   Firmware version: 0.1 \n   CANopen objects: N/A \n";
static uint8_t resiever1[50];
static uint8_t rec_len1;
static uint8_t resiever2[50];
static uint8_t rec_len2;
static uint8_t resiever3[50];
static uint8_t rec_len3;
USART_t usart1;
USART_t usart2;
USART_t usart3;
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

				gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RE_RX);
			}else{
				rcc_periph_clock_enable(RCC_GPIOA);
				gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
						GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

				gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
						GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

				nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
				nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);

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
		if (usart1.lenth-- != 0){
			/*send bytes until it will be send*/
				usart_send(USART1, usart1.data_pointer++);
			/*
			 *        if(--usart1.lenth !=0){
			 *                usart1.byte_counter = 3;
			 *                usart1.global_pointer++;
			 *                usart1.data1 = (*usart1.global_pointer >> 24) & 0xff;
			 *                usart1.data2 = (*usart1.global_pointer >> 16) & 0xff;
			 *                usart1.data3 = (*usart1.global_pointer >> 8) & 0xff;
			 *                usart1.data4 = (*usart1.global_pointer) & 0xff;
			 *                usart1.data_pointer = &usart1.data1;
			 *                usart_send(USART1, *usart1.data_pointer++);
			 *        }else{
			 *                //Disable the TXE interrupt as we don't need it anymore. 
			 *                USART_CR1(USART1) &= ~USART_CR1_TXEIE;
			 *                usart1.busy = 0;
			 *}
			 */

		}else{
			//Disable the TXE interrupt as we don't need it anymore. 
			USART_CR1(USART1) &= ~USART_CR1_TXEIE;
			free(usart1.buffer);
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
		usart_send_byte(USART1, tmp);
		/*
		 *if (tmp == '\n')
		 *{
		 *        resiever2[rec_len2++] = 0;	[> Make null-terminated string <]
		 *        process_command(resiever2);
		 *        rec_len2 = 0;
		 *}else{
		 *        resiever2[rec_len2++] = tmp;
		 *}
		 */

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

void usart3_isr(void)
{
	uint8_t tmp;

	//Check if we were called because of RXNE. 
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
			((USART_SR(USART3) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		tmp = usart_recv(USART3);
		if (tmp == '\n')
		{
			resiever3[rec_len3++] = 0;	/* Make null-terminated string */
			process_command(resiever3);
			rec_len3 = 0;
		}else{
			resiever3[rec_len3++] = tmp;
		}

		/*USART_CR1(USART1) &= ~USART_CR1_RXNEIE;*/
	}
	//Check if we were called because of TXE. 
	if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0) &&
			((USART_SR(USART3) & USART_SR_TXE) != 0)) {
		/*Check the count of non-sended words*/	
		if (usart3.lenth != 0){
			/*send bytes until it will be send*/
			if (usart3.byte_counter-- != 0){
				usart_send(USART3, *usart3.data_pointer++);
			}else{
				if(--usart3.lenth !=0){
					/*Reconfig usarts pointers and byte array*/
					usart3.byte_counter = 3;
					usart3.global_pointer++;
					usart3.data1 = (*usart3.global_pointer >> 24) & 0xff;
					usart3.data2 = (*usart3.global_pointer >> 16) & 0xff;
					usart3.data3 = (*usart3.global_pointer >> 8) & 0xff;
					usart3.data4 = (*usart3.global_pointer) & 0xff;
					usart3.data_pointer = &usart3.data1;
					usart_send(USART3, *usart3.data_pointer++);
				}else{
					//Disable the TXE interrupt as we don't need it anymore. 
					USART_CR1(USART3) &= ~USART_CR1_TXEIE;
					usart3.busy = 0;
				}
			}

		}else{
			//Disable the TXE interrupt as we don't need it anymore. 
			USART_CR1(USART3) &= ~USART_CR1_TXEIE;
			usart3.busy = 0;
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

void process_command(char *cmd)
{
	uint16_t t;
	if(strncmp(cmd, "LED", 3) == 0)
	{
		gpio_toggle(GREEN_LED_PORT, GREEN_LED);
		usart_send_byte(USART1, 'l');
	}    

	
	if(strncmp(cmd, "TEMP", 4) == 0)
	{
		/*adc_get_temperature();*/
	}    
		/* Manual  */
	if(strncmp(cmd, "info", 4) == 0)
	{
		usart_printf(USART1, help_msg);
	}
	if(strncmp(cmd, "TIM", 3) == 0)
	{
		usart_printf(USART1, "TIM4 = %d\n", timer_get_counter(TIM4));
	}
}

void usart_send_data(uint32_t USART, uint32_t *data, uint8_t lenth)
{
    switch(USART)
    {
        case USART1:
	while (usart1.busy);
	usart1.busy = 1;	
	usart1.lenth = lenth;
	usart1.buffer = calloc(1, lenth);	//
	memcpy(usart1.buffer, data, lenth);
	usart1.data_pointer = usart1.buffer;
	usart1.lenth--;
	usart_send_blocking(USART, usart1.data_pointer++);
	USART_CR1(USART) |= USART_CR1_TXEIE;
	//Enable TxE interrupt
    break;
        case USART2:

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
    break;
    }
    }

void usart_send_byte(uint32_t USART, uint8_t data)
{
	switch(USART)
	{
		case USART1:
	while (usart1.busy);
	usart_send_blocking(USART, data);
	break;
		case USART2:
	while (usart2.busy);
	usart_send_blocking(USART, data);
	break;
	}


}

double atof(const char *s)
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

void ftoa(float num, uint8_t *str, uint8_t precision)
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
void my_usart_print_int(uint32_t usart, int value)
{
	int8_t i;
	uint8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(usart, '-');
		value = value * -1;
	}

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = (nr_digits - 1); i >= 0; i--) {
		usart_send_blocking(usart, buffer[i]);
	}

	usart_send_blocking(usart, '\r');
}


/* 
 * String limited by heap size. */


void usart_printf(uint32_t USART, const char * format, ...)
{
    char * strp;

    va_list args;
    va_start(args, format);
    const int stringLength = vasprintf(&strp, format, args);
    va_end(args);

    if (stringLength < 0) {
	return;
    }

    usart1.buffer = calloc(1, stringLength);
    memcpy(usart1.buffer, strp, stringLength);

    usart_dma_write(usart1.buffer, stringLength);
    free(strp);
}

void debug(uint32_t USART, const char * format, ...)
{
    char str[200];
    int i;
    int str_len;

    va_list args;
    va_start(args, format);
    str_len = vsnprintf(str, 200, format, args);
    va_end(args);

    for(i = 0; i < str_len; i++)
    {
        usart_send_byte(USART, str[i]);
    }
}

static void usart_dma_write(char *data, int size)
{
	/*
	 * Using channel 4 for USART1_TX
	 */

	usart1.busy = 1;
	/* Reset DMA channel*/
	dma_channel_reset(DMA1, DMA_CHANNEL4);

	dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR);
	dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)data);
	dma_set_number_of_data(DMA1, DMA_CHANNEL4, size);
	dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
	dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

	dma_enable_channel(DMA1, DMA_CHANNEL4);

        usart_enable_tx_dma(USART1);
}


void dma1_channel4_isr(void)
{
	if ((DMA1_ISR &DMA_ISR_TCIF4) != 0) {
		DMA1_IFCR |= DMA_IFCR_CTCIF4;

		free(usart1.buffer);
	 gpio_clear(GREEN_LED_PORT, GREEN_LED);
	}

	usart1.busy = 0;
	dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

	usart_disable_tx_dma(USART1);

	dma_disable_channel(DMA1, DMA_CHANNEL4);
}
