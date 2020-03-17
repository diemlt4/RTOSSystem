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
 * f(STM32F4) = 84MHz ; f(PWM) = 10KHz ; Counter Period = 100 -> Prescaler = 84MHz/(10KHz*100) = 84
 * 
 */
#include "fan.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
TIM_HandleTypeDef *fan_htim = &htim1;
TIM_HandleTypeDef *fan_read_htim = &htim5;
extern uint8_t cap_speed_available;
void xiaomi_fan_run(level)
{
	if(level == HIGH_SPEED)
	{
		TIM1->ARR = 1999;
		__HAL_TIM_SET_COMPARE(fan_htim, TIM_CHANNEL_4, 1000);
	}
	else if(level == MED_SPEED)
	{
		TIM1->ARR = 2999;
		__HAL_TIM_SET_COMPARE(fan_htim, TIM_CHANNEL_4, 1500);
	}
	else if(level == LOW_SPEED)
	{
		TIM1->ARR = 3999;
		__HAL_TIM_SET_COMPARE(fan_htim, TIM_CHANNEL_4, 2000);
	}
}

void xiaomi_fan_stop(){
	TIM1->ARR = 0;
	__HAL_TIM_SET_COMPARE(fan_htim, TIM_CHANNEL_4, 0);
}

