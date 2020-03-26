/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define UV_Pin GPIO_PIN_0
#define UV_GPIO_Port GPIOC
#define ION_Pin GPIO_PIN_1
#define ION_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define DAMPER_OUT_0_Pin GPIO_PIN_4
#define DAMPER_OUT_0_GPIO_Port GPIOA
#define DAMPER_OUT_1_Pin GPIO_PIN_5
#define DAMPER_OUT_1_GPIO_Port GPIOA
#define DAMPER_OUT_2_Pin GPIO_PIN_6
#define DAMPER_OUT_2_GPIO_Port GPIOA
#define DAMPER_OUT_3_Pin GPIO_PIN_7
#define DAMPER_OUT_3_GPIO_Port GPIOA
#define DAMPER_IN_2_Pin GPIO_PIN_4
#define DAMPER_IN_2_GPIO_Port GPIOC
#define DAMPER_IN_3_Pin GPIO_PIN_5
#define DAMPER_IN_3_GPIO_Port GPIOC
#define DAMPER_IN_0_Pin GPIO_PIN_0
#define DAMPER_IN_0_GPIO_Port GPIOB
#define DAMPER_IN_1_Pin GPIO_PIN_1
#define DAMPER_IN_1_GPIO_Port GPIOB
#define IR_RECEIVE_PIN_Pin GPIO_PIN_2
#define IR_RECEIVE_PIN_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SPEED_BUTTON_Pin GPIO_PIN_6
#define SPEED_BUTTON_GPIO_Port GPIOB
#define SPEED_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define MODE_BUTTON_Pin GPIO_PIN_7
#define MODE_BUTTON_GPIO_Port GPIOB
#define MODE_BUTTON_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */
enum status_t{
	ST_OFF,
	ST_ON
};

enum filter_mode_t{
	INDOOR,
	FRESH
};

enum ctrl_mode_t{
	MANUAL,
	AUTOMATIC
};

enum sys_mode_t{
	AUTO_MODE = 1,
	FRESH_MODE,
	INDOOR_MODE,
	NIGHT_MODE
};


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
