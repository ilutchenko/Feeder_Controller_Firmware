#include "i2c.h"
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "i2c.h"
#define LEDPORT GPIOE
#define LED GPIO15
#define PCA_9532 0xc0
struct iic i2c;
void i2c2_ev_isr(void)
{
	uint32_t reg32 __attribute__((unused));
	/* If writing proceed*/
	if (i2c.w== 1){
	/*Sending address*/
	if ((I2C_SR1(I2C2) & I2C_SR1_SB)){
		i2c_send_data(I2C2, i2c.address & 0xfe);
		return;
		}
	 /*When address sent, send register addr*/
	if ((I2C_SR1(I2C2) & I2C_SR1_ADDR)){
		reg32 = I2C_SR2(I2C2);
		i2c_send_data(I2C2, i2c.reg_addr);
		/*usart_printf(USART1, "I2C_DAT %d\n", i2c.reg_addr);*/
		return;
		}
	/*Send data while lenth is valid*/
	if ((I2C_SR1(I2C2) & I2C_SR1_BTF)){
/*
 *                i2c_send_data(I2C2, *i2c.data_pointer);
 *                i2c.data_pointer++;
 *
 *        [> If all bytes have been sent <]
 *                if (i2c.lenth-- == 0){
 *                        i2c_send_stop(I2C2);
 *                        i2c.busy = 0;
 *                        return;
 *                        }
 *                }
 */
                  i2c_send_stop(I2C2);
                  i2c.busy = 0;
		  /*i2c2_process_sequence();*/
                  return;
	}
	}
	/*If reading needed*/
	else if (i2c.r == 1){
	if (i2c.rs == 0){
	/*Start send*/
	if ((I2C_SR1(I2C2) & I2C_SR1_SB)){
		i2c_send_data(I2C2, (i2c.address & 0xfe));
		return;
		}
	 
	 /*When address sent, send register addr*/
	if ((I2C_SR1(I2C2) & I2C_SR1_ADDR)){
		reg32 = I2C_SR2(I2C2);
		i2c_send_data(I2C2, i2c.reg_addr);
		return;
		}
	/*When reg addr send, repeat start	*/
	if ((I2C_SR1(I2C2) & I2C_SR1_BTF) != 0){
		i2c_send_start(I2C2);
		i2c.rs = 1;	//repeated start send
		return;
		}
	}
	/*We set i2c.rs, so now can read data from i2c*/
	else{
		if((I2C_SR1(I2C2) & I2C_SR1_SB)){
			i2c_send_data(I2C2, (i2c.address | 0x01));
			return;
			}
		if ((I2C_SR1(I2C2) & I2C_SR1_ADDR)){
			i2c_enable_ack(I2C2);
			return;
			}
		if ((I2C_SR1(I2C2) & I2C_SR1_BTF)){
			*i2c.data_pointer = i2c_get_data(I2C2);
			i2c.data_pointer++;
			
			if (--i2c.lenth == 1){
				i2c_nack_current(I2C2);
				return;
				}
			}	
		if ((I2C_SR1(I2C2) & I2C_SR1_RxNE) !=0) {
			i2c_send_stop(I2C2);
			i2c.busy = 0;
			}	
	}
}
}

void i2c2_er_isr(void)
{
	/*TODO errors handling*/
	usart_printf(USART1, "I2C_ERROR");

 	gpio_set(LEDPORT, LED);
	
}

void i2c2_setup(void)
{
	/* Enable clocks for I2C2 and AFIO. */
	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_AFIO);

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
	 		GPIO_I2C2_SCL | GPIO_I2C2_SDA);

	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C2);

	/* APB1 is running at 36MHz. */
	i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_36MHZ);

	/* 100KHz - I2C Standart Mode */
	i2c_set_standard_mode(I2C2);

	/*
	 * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
	 * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
	 * Datasheet suggests 0x1e.
	 */
	i2c_set_ccr(I2C2, 0xb4);

	/*
	 * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
	 * 400kHz => 300ns and 100kHz => 1000ns; 1000ns/28ns = 36;
	 * Incremented by 1 -> 37.
	 */
	i2c_set_trise(I2C2, 0x25);

	nvic_enable_irq(NVIC_I2C2_EV_IRQ);
	nvic_enable_irq(NVIC_I2C2_ER_IRQ);
	i2c_enable_interrupt(I2C2, I2C_CR2_ITEVTEN);
	i2c_enable_interrupt(I2C2, I2C_CR2_ITERREN);
	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(I2C2);
}


void i2c2_write(uint8_t address, uint8_t reg_addr)
{
	while(i2c.busy);
	i2c.busy = 1;
	i2c.r = 0;
	i2c.w = 1;
	i2c.reg_addr = reg_addr;
	i2c.address = address;
	/*i2c.data_pointer = data;*/
	/*i2c.lenth = lenth;*/
	i2c_send_start(I2C2);
}

void i2c2_read(uint8_t address, uint8_t reg_addr, uint8_t *data, uint8_t lenth)
{
	i2c.busy = 1;
	i2c.r = 1;
	i2c.w = 0;
	i2c.reg_addr = reg_addr;
	i2c.address = address;
	i2c.data_pointer = data;
	i2c.lenth = lenth;
	i2c_send_start(I2C2);
}
void i2c2_process_sequence(void)
{
	uint32_t i;
	while (i2c.init_seq < NUM_OF_INIT_CMDS)
	{
		milli_sleep(5);
		usart_printf(USART1, "SEQ %d\n", i2c.init_cmds[i2c.init_seq]);
		i2c2_write(DISPLAY_ADDR, i2c.init_cmds[i2c.init_seq]);
		milli_sleep(5);
		/*usart_printf(USART1, "SEQ %d\n", i2c.init_cmds[i2c.init_seq]);*/
		i2c2_write(DISPLAY_ADDR, i2c.init_cmds[i2c.init_seq++] & 0xFB);
	}
}
