/*
 * sm_uart-04l.c
 *
 *  Created on: Mar 6, 2020
 *      Author: VHT
 */

#include "sm_uart-04l.h"
#include "stm32f4xx_hal.h"
#include "fan.h"

extern UART_HandleTypeDef huart2;
UART_HandleTypeDef *huart_sm_uart_04l;
static uint8_t rxBuffer[32];
extern uint8_t Rx_data[2];
amphenol amphenol1;

uint16_t crc(const uint8_t *data, uint32_t length){

}

void SM_UART_04L_Init(UART_HandleTypeDef * huart_handler) {
	huart_sm_uart_04l = huart_handler;
	HAL_Delay(10);
}


void startToRevSM04L(void) {
	HAL_UART_Receive_DMA(huart_sm_uart_04l, rxBuffer, sizeof(rxBuffer));
	//HAL_Delay(100);

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
	uint32_t speed;
	uint8_t str[50];
  if (huart->Instance == USART6) {
	if ( rxBuffer[0] == 'B' && rxBuffer[1] == 'M') {

	  amphenol1.PM1_Standard = rxBuffer[4]*256 + rxBuffer[5];
	  amphenol1.PM2p5_Standard = rxBuffer[6]*256 + rxBuffer[7];
	  amphenol1.PM10_Standard = rxBuffer[8]*256 + rxBuffer[9];

	  amphenol1.PM1_Environment = rxBuffer[10]*256 + rxBuffer[11];
	  amphenol1.PM2p5_Environment = rxBuffer[12]*256 + rxBuffer[13];
	  amphenol1.PM10_Environment = rxBuffer[14]*256 + rxBuffer[15];
	  printf("finish get pm2.5\r\n");
	}

	for (int i = 0; i<32 ; i++) {
	  rxBuffer[i] = 0;
	}
	HAL_UART_DMAStop(huart_sm_uart_04l);
  }
  if(huart->Instance == USART2){
	  HAL_UART_Transmit(&huart2, (uint8_t *)&Rx_data[0], 1, 0xFFFF);
	  if(Rx_data[0] == 'h'){
		  xiaomi_fan_run(HIGH_SPEED);
	  }else if(Rx_data[0] == 'm'){
		  xiaomi_fan_run(MED_SPEED);
	  }else if(Rx_data[0] == 'l'){
		  xiaomi_fan_run(LOW_SPEED);
	  }else if(Rx_data[0] == 'r'){
		  speed = xiaomi_fan_read();
		  sprintf(str, "speed %u\r\n", speed);
		  HAL_UART_Transmit(&huart2, str, strlen(str), 0xFFFF);
	  }
	  HAL_UART_Receive_IT(&huart2, Rx_data, 1);
  }
}

