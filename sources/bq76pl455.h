#ifndef BQ76PL455_LIB
#define BQ76PL455_LIB

#include <libopencm3/stm32/rcc.h>
//#include <inttypes.h>

uint16_t bq76_channel_read (uint8_t channel);
uint16_t bq76_get_status (void);
uint16_t bq76_convert_to_volts(uint16_t voltage);

#define FRM_TYPE_RESPONSE                       0
#define FRM_TYPE_COMMAND                        1
#define REQ_TYPE_SINGLE_DEV_WRITE_RESPONSE      0
#define REQ_TYPE_SINGLE_DEV_WRITE_NO_RESPONSE   1
#define REQ_TYPE_GROUP_WRITE_RESPONSE           2
#define REQ_TYPE_GROUP_WRITE_NORESPONSE         3
#define REQ_TYPE_BROADCAST_WRITE_RESPONSE       6
#define REQ_TYPE_BROADCAST_WRITE_NORESPONSE     7
#define ADDR_SIZE_8bit                          0
#define ADDR_SIZE_16bit                         1
#define DATA_SIZE_0bit                          0
#define DATA_SIZE_1bit                          1
#define DATA_SIZE_2bit                          2
#define DATA_SIZE_3bit                          3
#define DATA_SIZE_4bit                          4
#define DATA_SIZE_5bit                          5
#define DATA_SIZE_6bit                          6
#define DATA_SIZE_8bit                          7


/**
 *
 */

#define SREV                0x00
#define CMD                 0x02
#define CHANNELS            0x03
#define OVERSMPL            0x07
#define ADDR                0x0A
#define GROUP_ID            0x0B
#define DEV_CTRL            0x0C
#define NCHAN               0x0D
#define DEVCONFIG           0x0E
#define PWRCONFIG           0x0F
#define COMCONFIG           0x10
#define TXHOLDOFF           0x12
#define CBCONFIG            0x13
#define CBENBL              0x14
#define TSTCONFIG           0x1E
#define TESTCTRL            0x20
#define TEST_ADC            0x22
#define TESTAUXPU           0x25
#define CTO                 0x28
#define CTO_CNT             0x29
#define AM_PER              0x32
#define AM_CHAN             0x33
#define AM_OSMPL            0x37
#define SMPL_DLY1           0x3D
#define CELL_SPER           0x3E
#define AUX_SPER            0x3F
#define TEST_SPER           0x43
#define SHDN_STS            0x50
#define STATUS              0x51
#define FAULT_SUM           0x52
#define FAULT_UV            0x54
#define FAULT_OV            0x56
#define FAULT_AUX           0x58
#define FAULT_2UV           0x5A
#define FAULT_2OV           0x5C
#define FAULT_COM           0x5E
#define FAULT_SYS           0x60
#define FAULT_DEV           0x61
#define FAULT_GPI           0x63
#define MASK_COMM           0x68
#define MASK_SYS            0x6A
#define MASK_DEV            0x6B
#define FO_CTRL             0x6E
#define GPIO_DIR            0x78
#define GPIO_OUT            0x79
#define GPIO_PU             0x7A
#define GPIO_PD             0x7B
#define GPIO_IN             0x7C
#define GP_FLT_IN           0x7D
#define MAGIC1              0x82
#define COMP_UV             0x8C
#define COMP_OV             0x8D
#define CELL_UV             0x8E
#define CELL_OV             0x90
#define AUX0_UV             0x92
#define AUX0_OV             0x94
#define AUX1_UV             0x96
#define AUX1_OV             0x98
#define AUX2_UV             0x9A
#define AUX2_OV             0x9C
#define AUX3_UV             0x9E
#define AUX3_OV             0xA0
#define AUX4_UV             0xA2
#define AUX4_OV             0xA4
#define AUX5_UV             0xA6
#define AUX5_OV             0xA8
#define AUX6_UV             0xAA
#define AUX6_OV             0xAC
#define AUX7_UV             0xAE
#define AUX7_OV             0xB0
#define LOT_NUM             0xBE
#define SER_NUM             0xC6
#define SCRATCH             0xC8
#define VSOFFSET            0xD2
#define VSGAIN              0xD3
#define AX0OFFSET           0xD4
#define AX1OFFSET           0xD6
#define AX2OFFSET           0xD8
#define AX3OFFSET           0xDA
#define AX4OFFSET           0xDC
#define AX5OFFSET           0xDE
#define AX6OFFSET           0xE0
#define AX7OFFSET           0xE2
#define TSTR_ECC            0xE6
#define CSUM                0xF0
#define CSUM_RSLT           0xF4
#define TEST_CSUM           0xF8
#define EE_BURN             0xFA
#define MAGIC2              0xFC

//------------------------------------
















#endif
