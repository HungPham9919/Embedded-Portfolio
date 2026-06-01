#ifndef USART_CONFIG_H
#define USART_CONFIG_H

#include "stdint.h"
#include "stdio.h"
#include "nrf.h"
#include "string.h"
#include "Radio.h"

void USART_Initialize(uint8_t TX_pin, uint8_t RX_pin);
void USART0_Send_Char(char c);
void USART_Disable(void);
void USART0_Send_String(char *s);

/*

void delay_ms(uint32_t ms){
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->PRESCALER = 4; // 1 MHz (1 tick = 1us)
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;

    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->TASKS_START = 1;

    while (1){
        NRF_TIMER0->TASKS_CAPTURE[0] = 1;   // copy COUNTER → CC[0]
        if (NRF_TIMER0->CC[0] >= ms * 1000){
            break;
        }
    }

    NRF_TIMER0->TASKS_STOP = 1;
}
*/

#endif