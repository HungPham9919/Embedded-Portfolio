#ifndef UART_H
#define UART_H

#include "zephyr/kernel.h"
#include "stdint.h"
#include "stdio.h"
#include "nrf.h"
#include "string.h"
#include "Radio/Radio.h"

void USART_Initialize(uint8_t TX_pin, uint8_t RX_pin);
void USART0_Send_Char(char c);
void USART_Disable(void);
void USART0_Send_String(char *s);
void SWI0_Configuration(void);
void delay_ms(uint32_t ms);

#endif