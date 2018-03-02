#ifndef TIMERS_H
#define TIMERS_H

#include "defines.h"
#define TIMER1_TOP	1846
#define START_PWM_VALUE 90
#define BREAK_IMPULSE_LENTH 1000
#define TIMER3_TOP	400
void tim1_init(void);
void tim2_init(void);
void tim1_enable(uint8_t param);
void tim1_set_pwm (uint8_t pwm);

#endif
