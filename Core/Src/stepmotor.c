/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stepmotor.h
  * @brief          : Nguyen Trung Hieu
  *                   hieunt91@viettel.com.vn
  *                   Dai Hoc Dien Luc
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stepmotor.h"
#include "stdbool.h"
#include "stdio.h"

void directionOfRotation(int damper, bool direction, int degree) {
	GPIO_TypeDef *damper_port0 = IN1_Port;
	GPIO_TypeDef *damper_port1 = IN2_Port;
	GPIO_TypeDef *damper_port2 = IN3_Port;
	GPIO_TypeDef *damper_port3 = IN4_Port;
	uint16_t pin0 = IN1_Pin;
	uint16_t pin1 = IN2_Pin;
	uint16_t pin2 = IN3_Pin;
	uint16_t pin3 = IN4_Pin;

	if(damper == DAMPER_IN){
		damper_port0 = DAMPER_IN_0_GPIO_Port;
		damper_port1 = DAMPER_IN_1_GPIO_Port;
		damper_port2 = DAMPER_IN_2_GPIO_Port;
		damper_port3 = DAMPER_IN_3_GPIO_Port;

		pin0 = DAMPER_IN_0_Pin;
		pin1 = DAMPER_IN_1_Pin;
		pin2 = DAMPER_IN_2_Pin;
		pin3 = DAMPER_IN_3_Pin;

	}else if(damper == DAMPER_OUT){
		damper_port0 = DAMPER_OUT_0_GPIO_Port;
		damper_port1 = DAMPER_OUT_1_GPIO_Port;
		damper_port2 = DAMPER_OUT_2_GPIO_Port;
		damper_port3 = DAMPER_OUT_3_GPIO_Port;

		pin0 = DAMPER_OUT_0_Pin;
		pin1 = DAMPER_OUT_1_Pin;
		pin2 = DAMPER_OUT_2_Pin;
		pin3 = DAMPER_OUT_3_Pin;
	}

	uint16_t t = degree*512/360;
	for (uint16_t i = 0; i <t; i++) {
		if(direction) {
			HAL_GPIO_WritePin (damper_port0, pin0, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port1, pin1, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port2, pin2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port3, pin3, GPIO_PIN_RESET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (damper_port0, pin0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port1, pin1, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port2, pin2, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port3, pin3, GPIO_PIN_RESET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (damper_port0, pin0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port1, pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port2, pin2, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port3, pin3, GPIO_PIN_SET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (damper_port0, pin0, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port1, pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port2, pin2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port3, pin3, GPIO_PIN_SET);

			HAL_Delay(delayTime);
		} else {
			HAL_GPIO_WritePin (damper_port0, pin0, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port1, pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port2, pin2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port3, pin3, GPIO_PIN_SET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (damper_port0, pin0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port1, pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port2, pin2, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port3, pin3, GPIO_PIN_SET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (damper_port0, pin0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port1, pin1, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port2, pin2, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port3, pin3, GPIO_PIN_RESET);

			HAL_Delay(delayTime);

			HAL_GPIO_WritePin (damper_port0, pin0, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port1, pin1, GPIO_PIN_SET);
			HAL_GPIO_WritePin (damper_port2, pin2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin (damper_port3, pin3, GPIO_PIN_RESET);

			HAL_Delay(delayTime);
		}
	}
}
