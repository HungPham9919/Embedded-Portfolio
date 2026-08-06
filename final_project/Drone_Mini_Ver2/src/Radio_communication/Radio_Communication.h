#ifndef RADIO_COMMUNICATION_H
#define RADIO_COMMUNICATION_H

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"

int PWM_Converted(char *buffer);
void USART_Configuration(void);
void USART6_Send_Char(char c);
void USART6_Send_String(char *s);

#endif