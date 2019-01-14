#ifndef DEFINES_H
#define DEFINES_H

//Task defines
#define WELD_TASK		1
#define MOTOR_TASK		2
#define GAS_TASK		3

#define STOPPED_SEQUENCE	0
#define START_SEQUENCE		1
#define STOP_SEQUENCE		2
#define COMPARE_STEP		50
#define GREEN_LED_PORT	GPIOC
#define GREEN_LED	GPIO13
#define RED_LED_PORT	GPIOB
#define RED_LED		GPIO13
#define YELLOW_LED_PORT GPIOB
#define YELLOW_LED	GPIO12

#define DEBUGVAL	GPIOB_CRL 

#define GAS_PIN		GPIO10
#define GAS_PORT	GPIOB

#define WELD_PIN	GPIO11
#define WELD_PORT	GPIOB

#define BREAK_PIN	GPIO8
#define BREAK_PORT	GPIOB

#define WELD_GUN_PIN	GPIO9
#define WELD_GUN_PORT	GPIOB

#define SWITCH_PIN	GPIO5
#define SWITCH_PORT	GPIOA

//Encoder inputs
#define ENCODER_A_PIN	GPIO7
#define ENCODER_B_PIN	GPIO6
#define ENCODER_A_PORT	GPIOB
#define ENCODER_B_PORT	GPIOB

#define ENCODER_SYSTICS		100
#define ENCODER_MM_PER_TIC	120.0/360
typedef struct
{
	uint16_t current_cnt;
	uint16_t previous_cnt;
	uint16_t speed_mm;
	uint8_t  direction;
	uint8_t systics;
}Encoder_t;

typedef struct
{
	uint16_t timeout;	//until next task
	uint8_t sequence;	//star/stop sequence
	uint8_t subtask;

}Tasks_t;

typedef struct
{
	Encoder_t *encoder;
	Tasks_t	*tasks;
}Data_t;

#endif
