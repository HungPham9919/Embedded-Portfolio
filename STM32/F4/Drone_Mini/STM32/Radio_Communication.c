/*
 * Radio_Communication.c
 *
 *  Created on: May 23, 2026
 *      Author: hung
 */

#include "Radio_Communication.h"

void USART_Configuration(void){
	RCC->APB2ENR |= (1 << 5); // UART 6
	RCC->AHB1ENR |= (1 << 2); // GPIOC
	for(volatile int i = 0; i < 100; i++); // wait for stable

	// USART 6 PC6_TX, PC7_RX 84MHz
	GPIOC->MODER &= ~(3 << 12) &~(3 << 14);
	GPIOC->MODER |= (2 << 12)|(2 << 14);

	GPIOC->OSPEEDR |= (3 << 12)|(3 << 14);
	GPIOC->AFR[0] &= ~(0x0F << 24) &~(0x0F << 28);
	GPIOC->AFR[0] |= (8 << 24)|(8 << 28); // AF8

	USART6->CR1 &= ~(1 << 13);
	USART6->BRR = (5 << 4)|(11 << 0); // 921600
	USART6->CR1 |= (1 << 2)|(1 << 3)|(1 << 5)|(1 << 13);
	NVIC_SetPriority(USART6_IRQn, 5);
	NVIC->ISER[2] |= (1 << 7);
}

void USART6_Send_Char(char c){
	while((USART6->SR & (1 << 6)) == 0){};
	USART6->DR = c;
}

void USART6_Send_String(char *s){
	while(*s){
		USART6_Send_Char(*s++);
	}
}

int converted(char *buffer){
	char check[11] = {'0','1','2','3','4','5','6','7','8','9'};
	char temp[6];
	int k = 0;
	for(int i = 0; i < 5; i++){
		for(int j = 0; j < 10; j++){
			if(buffer[i] == check[j]){
				temp[k++] = buffer[i];
				break;
			}
		}
	if(buffer[i] == '\0') break;
    }
	return atoi(temp); // convert char to integer
}

extern osThreadId_t Radio_Thread;
extern osMessageQueueId_t radioQueueHandle;
void USART6_IRQHandler(void) {

    uint32_t sr = USART6->SR;
    if (sr & (1 << 3)) {
        volatile uint32_t dummy = USART6->DR; // Đọc DR để ép xóa cờ lỗi ORE vĩnh viễn
        (void)dummy;
    }
    if (sr & (1 << 5)) {
        char rx_byte = (char)USART6->DR;
    	static char rx_buffer[RADIO_MSG_LEN];
		static int idx = 0;
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (idx > 0) {
            	rx_buffer[idx] = '\0';
                osMessageQueuePut(radioQueueHandle, rx_buffer, 0, 0);
                idx = 0;
            }
        }
        else {
            if (idx < 31) {
                if (rx_byte >= 32 && rx_byte <= 126) {
                	rx_buffer[idx++] = rx_byte;
                }
            } else {
                idx = 0;
            }
        }
    }
}
