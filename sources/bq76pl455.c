/*Library for cell voltage measurement IC BQ76PL455 from texas Instruments*/
#include "bq76pl455.h"
#include <stdlib.h>
#include <stdio.h>

uint16_t bq76_channel_read (uint8_t channel)
{
	
}

uint16_t bq76_get_status (void)
{

}
uint16_t bq76_convert_to_volts(uint16_t voltage)
{

}

uint8_t frame_Builder(uint8_t FRM_TYPE, uint8_t REQ_TYPE, uint8_t ADDR_SIZE, uint8_t DATA_SIZE){
	uint8_t temp=0;

	temp=DATA_SIZE;
	temp+=(ADDR_SIZE<<3);
	temp+=(REQ_TYPE<<4);
	temp+=(FRM_TYPE<<7);

	return temp;
}

/**
 * \fn      uint16_T  crc_16_ibm(uint8_T *buf, uint16_T len)
 * \brief   There are many good sources for algorithms and efficient techniques for generating and checking CRCs
 available on the Internet. The following byte-oriented C language routine has been developed and verified as a
 reference. The only complication needed to take into account using this function is that the low byte of the CRC
 value returned is the CRC (MSB) and the high byte is the CRC (LSB)so the last istruction invert the bytes
 * @param   buf     pointer to the message
 * @param   len     dimension of the message
 * @return  crc     resulting CRC-> MSB + LSB
 */
uint16_t  crc_16_ibm(uint8_t *buf, uint8_t len) {
	uint16_t crc = 0;
	uint16_t j;
	uint8_t msb, lsb=0;

	while (len--) {
		crc ^= *buf++;
		for (j = 0; j < 8 ; j++)
			crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
	}

	//read the description, this is the inversion of MSB and LSB
	msb=crc;
	lsb=(crc>>8);

	crc=msb;
	crc=crc<<8;
	crc+=lsb;

	return crc;
}





/**
 * \fn unsigned int* write_to_Dev(unsigned char req_typ, unsigned char addr_size, unsigned char len, unsigned char dev_addr, unsigned char reg_addr, unsigned char *buf)
 * \brief Composition of the message. Not for broadcast messages.
 * @param REQ_TYPE      REQ_TYPE_SINGLE_DEV_WRITE_RESPONSE      0
 *                      REQ_TYPE_SINGLE_DEV_WRITE_NO_RESPONSE   1
 *                      REQ_TYPE_GROUP_WRITE_RESPONSE           2
 *                      REQ_TYPE_GROUP_WRITE_NORESPONSE         3
 *                      REQ_TYPE_BROADCAST_WRITE_RESPONSE       6
 *                      REQ_TYPE_BROADCAST_WRITE_NORESPONSE     7
 * @param ADDR_SIZE     ADDR_SIZE_8bit                          0
 *                      ADDR_SIZE_16bit                         1
 * @param len           Data size
 * @param dev_addr      device address.
 * @param reg_addr      register address of the device
 * @param buf           pointer to the data
 * @return              pointer of the message, the total size is length+ (dev_addr) + (reg_addr) + crc = length+4
 */
unsigned int* write_to_Dev(unsigned char REQ_TYP, unsigned char ADDR_SIZE, unsigned char len, unsigned int dev_addr, unsigned int reg_addr, unsigned int *buf){
	unsigned char mess[len+3+ADDR_SIZE];
	unsigned char frame=frame_Builder(1,REQ_TYP,ADDR_SIZE,len);
	unsigned int* temp=buf;
	mess[0]=frame;
	mess[1]=dev_addr;
	if(ADDR_SIZE>0){
		mess[2]=(reg_addr>>8);
		mess[2+ADDR_SIZE]=reg_addr;
	}
	else{
		mess[2]=(reg_addr);
	}

	int i;
	for(i=3+ADDR_SIZE;i<(len+3+ADDR_SIZE);i++){
		mess[i]=*temp;
		temp++;
	}
	unsigned int crc=crc_16_ibm(&mess,len+3+ADDR_SIZE);
	mess[i]=(crc>>8);
	i+=1;
	mess[i]=crc;

	return &mess;
}



unsigned int* write_to_Gr(unsigned char REQ_TYP, unsigned char ADDR_SIZE, unsigned char len, unsigned int group_ID, unsigned int reg_addr, unsigned int *buf){
	unsigned char mess[len+3+ADDR_SIZE];
	unsigned char frame=frame_Builder(1,REQ_TYP,ADDR_SIZE,len);
	unsigned int* temp=buf;
	mess[0]=frame;
	mess[1]=(group_ID);
	if(ADDR_SIZE>0){
		mess[2]=(reg_addr>>8);
		mess[2+ADDR_SIZE]=reg_addr;
	}
	else{
		mess[2]=(reg_addr);
	}

	int i;
	for(i=3+ADDR_SIZE;i<(len+3+ADDR_SIZE);i++){
		mess[i]=*temp;
		temp++;
	}
	unsigned int crc=crc_16_ibm(&mess,len+3+ADDR_SIZE);
	mess[i]=(crc>>8);
	i+=1;
	mess[i]=crc;

	return &mess;
}




unsigned int* write_to_Broad(unsigned char REQ_TYP, unsigned char ADDR_SIZE, unsigned char len, unsigned int reg_addr, unsigned int *buf){
	unsigned char mess[len+2+ADDR_SIZE];
	unsigned char frame=frame_Builder(1,REQ_TYP,ADDR_SIZE,len);
	unsigned int* temp=buf;
	mess[0]=frame;
	if(ADDR_SIZE>0){
		mess[1]=(reg_addr>>8);
		mess[1+ADDR_SIZE]=reg_addr;
	}
	else{
		mess[1]=(reg_addr);
	}


	int i;
	for(i=2+ADDR_SIZE;i<(len+2+ADDR_SIZE);i++){
		mess[i]=*temp;
		temp++;
	}
	unsigned int crc=crc_16_ibm(&mess,len+2+ADDR_SIZE);
	mess[i]=(crc>>8);
	i+=1;
	mess[i]=crc;

	return &mess;
}


char checkCRC(unsigned int crc, unsigned int* buf, unsigned char len){
	unsigned int crc2=crc_16_ibm(buf,len);
	if(crc==crc2){
		return 0;
	}
	else
		return -1;
}
