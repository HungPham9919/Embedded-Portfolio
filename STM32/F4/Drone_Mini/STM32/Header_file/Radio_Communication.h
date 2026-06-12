/*
 * Radio_Communication.h
 *
 *  Created on: May 23, 2026
 *      Author: hung
 */

#ifndef INC_RADIO_COMMUNICATION_H_
#define INC_RADIO_COMMUNICATION_H_

#include "stdio.h"
#include "string.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "main.h"

int converted(char *buffer);
void USART_Configuration(void);
void USART6_Send_Char(char c);
void USART6_Send_String(char *s);
#endif /* INC_RADIO_COMMUNICATION_H_ */
