/*
 * Radio_Communication.c
 *
 *  Created on: May 23, 2026
 *      Author: hung
 */

#include "Radio_Communication.h"

void USART_Configuration(void){
	RCC->APB2ENR |= (1 << 5); // UART 6
	for(volatile int i = 0; i < 100; i++); // wait for stable

	// USART 6 PC6_TX, PC7_RX 24MHz
	GPIOC->MODER &= ~(3 << 12) &~(3 << 14);
	GPIOC->MODER |= (2 << 12)|(2 << 14);
	GPIOC->OSPEEDR |= (3 << 12)|(3 << 14);
	GPIOC->AFR[0] &= ~(0x0F << 24) &~(0x0F << 28);
	GPIOC->AFR[0] |= (8 << 24)|(8 << 28);

	USART6->CR1 &= ~(1 << 13);
	USART6->BRR = (156 << 4)|(4 << 0); // 9600
	USART6->CR1 |= (1 << 2)|(1 << 3)|(1 << 5)|(1 << 13);
	NVIC->ISER[2] |= (1 << 7);
}

int Drone_State = 0,idx = 0;
char Data[] = {0};
extern osThreadId_t Radio_Thread;

void USART6_IRQHandler(void) {
    if (USART6->SR & (1 << 5)) { // Kiểm tra cờ RXNE (có dữ liệu đến)
        Data[idx++] = USART6->DR;
        // Xử lý dữ liệu ở đây
        if(strcmp(Data,"Radio_Ready") == 0){
        	osThreadFlagsSet(Radio_Thread, 0x0001);
        }
        if(strcmp(Data,"Move_Forward") == 0){
        	// Do something
        	idx = 0;
        	memset(Data,0,sizeof(Data));
        }
        if(strcmp(Data,"Move_back") == 0){
        	// Do something
        	idx = 0;
        	memset(Data,0,sizeof(Data));
        }
    }
}
