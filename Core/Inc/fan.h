/* Author: TTTBYT
 *
 * Brief: ERV Motor Control.
 * Two fan: FAN1 & FAN2 (2 signal wire: PWM-speed cotrol and TACH-speed feedback (2 or 3 ppr) )
 * Pin connect:
 * 		FAN1_TACH: PA6 (TIM3_CH1_IC)
 * 		FAN2_TACH: PA7 (TIM3_CH2_IC)
 
 * 		FAN1_PWM:  PA8 (TIM1_CH1_PWM)
 * 		FAN2_PWM:  PA11(TIM1_CH4_PWM)
 *
 * 	Program:
 * 	 1. Add fan.h and fan.c
 * 	 2. In CubeMX, config: TIM1_CH1_PWM and TIM1_CH4_PWM
 * 	 3. Add fan_init();
 * 	 4. Using stop, run... -> speed: 0 -> 100
 */

#ifndef FAN_H
#define FAN_H

#include "stm32f4xx.h"

enum fan_speed_t{
	LOW_SPEED = 1,
	MED_SPEED,
	HIGH_SPEED
};
//Motor init, fix in use TIM3 (for speed control) and TIM1 (for speed feedback)

void xiaomi_fan_stop();
void xiaomi_fan_run(level);

#endif
