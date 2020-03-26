/*
 * sm_uart-04l.h
 *
 *  Created on: Mar 6, 2020
 *      Author: VHT
 */

#ifndef INC_SM_UART_04L_H_
#define INC_SM_UART_04L_H_

#include "main.h"

typedef struct amphenol{
	uint16_t PM1_Standard;
	uint16_t PM2p5_Standard;
	uint16_t PM10_Standard;

	uint16_t PM1_Environment;
	uint16_t PM2p5_Environment;
	uint16_t PM10_Environment;
}amphenol_t;

enum output_mode_t{
	ASK_ANSWER,
	DIRECT
};

void SM_UART_04L_Init(UART_HandleTypeDef * huart_handler);
void startToRevSM04L(void);
int setSMStandbyControl(int mode);
int setSMOutputMode(int mode);
uint8_t getSMData(amphenol_t *amp, uint8_t *alarm);
void initSMUARTPort(UART_HandleTypeDef *huart_handler);

#endif /* INC_SM_UART_04L_H_ */
