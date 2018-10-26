#include "display.h"

void display_init(uint32_t i2c)
{
	uint32_t reg32 __attribute__((unused));
	uint32_t i;
	/* Send START condition. */
	/*i2c_send_start(i2c);*/

	/*i2c1_write(DISPLAY_ADDR, DISPLAY_CURS_SPEED_ADDR, NULL, 0);*/
	/* Waiting for START is send and switched to master mode. */
	/*
	 *while (!((I2C_SR1(i2c) & I2C_SR1_SB)
	 *        & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));
	 */
	i2c2_process_sequence();

	/* Send destination address. */
	/*i2c_send_7bit_address(i2c, DISPLAY_ADDR, I2C_WRITE);*/

	/* Waiting for address is transferred. */
	/*while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));*/

	/* Cleaning ADDR condition sequence. */
	/*reg32 = I2C_SR2(i2c);*/

	/* Sending the data. */
/*
 *        i2c_send_data(i2c, DISPLAY_CURS_SPEED_ADDR); [>  <]
 *        while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
 *                i2c_send_data(i2c, 0x6);	//moving cursor left 
 *                while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
 *        [> After the last byte we have to wait for TxE too. <]
 *        while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
 *
 *        [> Send STOP condition. <]
 *        i2c_send_stop(i2c);
 */
	
	/*usart_printf(USART1, " I2C_CR1 = %d\n I2C_CR2 = %d\n I2C_DR = %d\n I2C_SR1 = %d\n I2C_SR2 = %d\n I2C_CCR = %d\n I2C_TRISE = %d\n", I2C_CR1(i2c), I2C_CR2(i2c), I2C_DR(i2c), I2C_SR1(i2c), I2C_SR2(i2c), I2C_CCR(i2c), I2C_TRISE(i2c));*/
}

void display_set_speed (uint32_t i2c, uint8_t *speedStr)
{
	uint32_t reg32 __attribute__((unused));
	int i;
	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Send destination address. */
	i2c_send_7bit_address(i2c, DISPLAY_ADDR, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Sending the data. */
	i2c_send_data(i2c, DISPLAY_CURS_SPEED_ADDR); /*  */
	/* After the last byte we have to wait for TxE too. */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/* Send STOP condition. */
	i2c_send_stop(i2c);
	for (i = 0; i < 30000; i++);
	
	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Send destination address. */
	i2c_send_7bit_address(i2c, DISPLAY_ADDR, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	for (i = 0; i < sizeof(speedStr); i++)
	{
		i2c_send_data(i2c, *(speedStr+i)); 
		while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	}
	/* After the last byte we have to wait for TxE too. */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/* Send STOP condition. */
	i2c_send_stop(i2c);
}
