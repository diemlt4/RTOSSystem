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
amphenol_t amphenol1;

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

void initSMUARTPort(UART_HandleTypeDef *huart_handler) {
	huart_sm_uart_04l = huart_handler;
	uint8_t rxbuf[100] = {0};
	// Clear RX buffer
	HAL_UART_Receive(huart_sm_uart_04l, rxbuf, 100, 1000);
}

int setSMOutputMode(int mode){
	uint8_t txbuf[7];
	uint8_t rxbuf[10] = {0};
	uint16_t crc= 0;
	uint16_t cs = 0;
	int i = 0;
	txbuf[0] = 0x42;
	txbuf[1] = 0x4D;
	txbuf[2] = 0xE1; // CMD
	txbuf[3] = 0; //DATA_H
	txbuf[4] = mode*0xff; //DATA_L
	for(i = 0; i < 5; i++){
		crc += txbuf[i];
	}
	txbuf[5] = crc >> 8;
	txbuf[6] = crc & 0xff;
	HAL_UART_Transmit(huart_sm_uart_04l, txbuf, 7, 0xFFFF);
	HAL_UART_Receive(huart_sm_uart_04l, rxbuf, 8, 5000);
	crc = (rxbuf[6] << 8)| rxbuf[7];
	for(i = 0; i < 8; i++){
		printf("0x%02x ", rxbuf[i]);
	}
	for(i = 0; i < 6; i++){
		cs += rxbuf[i];
	}
	printf("\r\n");
	return (cs == crc);
}

int setSMStandbyControl(int mode){
	uint8_t txbuf[7];
	uint8_t rxbuf[10] = {0};
	uint16_t crc= 0;
	uint16_t cs = 0;
	int i = 0;
	txbuf[0] = 0x42;
	txbuf[1] = 0x4D;
	txbuf[2] = 0xE4; // CMD
	txbuf[3] = 0; //DATA_H
	txbuf[4] = mode*0xff; //DATA_L
	for(i = 0; i < 5; i++){
		crc += txbuf[i];
	}
	txbuf[5] = crc >> 8;
	txbuf[6] = crc & 0xff;
	HAL_UART_Transmit(huart_sm_uart_04l, txbuf, 7, 0xFFFF);
	HAL_UART_Receive(huart_sm_uart_04l, rxbuf, 8, 5000);
	crc = (rxbuf[6] << 8)| rxbuf[7];
	for(i = 0; i < 8; i++){
		printf("0x%02x ", rxbuf[i]);
	}
	for(i = 0; i < 6; i++){
		cs += rxbuf[i];
	}
	printf("\r\n");
	return (cs == crc);
}

uint8_t getSMData(amphenol_t *amp, uint8_t *alarm){
	uint8_t txbuf[7];
	uint8_t rxbuf[32] = {0};
	uint16_t crc= 0;
	uint16_t cs = 0;
	uint8_t ret;
	int i = 0;
	txbuf[0] = 0x42;
	txbuf[1] = 0x4D;
	txbuf[2] = 0xE2; // CMD
	txbuf[3] = 0; //DATA_H
	txbuf[4] = 0; //DATA_L
	for(i = 0; i < 5; i++){
		crc += txbuf[i];
	}
	txbuf[5] = crc >> 8;
	txbuf[6] = crc & 0xff;
	HAL_UART_Transmit(huart_sm_uart_04l, txbuf, 7, 0xFFFF);
	ret = HAL_UART_Receive(huart_sm_uart_04l, rxbuf, 32, 10000);
	if(ret != HAL_OK){
		return ret;
	}
	crc = (rxbuf[30] << 8)| rxbuf[31];
	//Check sum
	for(i = 0; i < 30; i++){
		cs += rxbuf[i];
	}
	if ( rxbuf[0] == 'B' && rxbuf[1] == 'M' && cs == crc) {
		amp->PM1_Standard = rxbuf[4]*256 + rxbuf[5];
		amp->PM2p5_Standard = rxbuf[6]*256 + rxbuf[7];
		amp->PM10_Standard = rxbuf[8]*256 + rxbuf[9];

		amp->PM1_Environment = rxbuf[10]*256 + rxbuf[11];
		amp->PM2p5_Environment = rxbuf[12]*256 + rxbuf[13];
		amp->PM10_Environment = rxbuf[14]*256 + rxbuf[15];
		*alarm = rxbuf[29];
		return 0;
	}
	return -1;
}

