#include "Radio_Communication.h"
void uart6_irqhandler(const void *arg);

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

	IRQ_CONNECT(71,6,uart6_irqhandler,NULL,0);
	irq_enable(71);
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
volatile int duty = 0;
int PWM_Converted(char *buffer){
	char check[11] = {'0','1','2','3','4','5','6','7','8','9'};
	char temp[4];
	int k = 0;
	for(int i = 0; i < strlen(buffer); i++){
		for(int j = 0; j < 10; j++){
			if(buffer[i] == check[j]){
				temp[k++] = buffer[i];
				break;
			}
			if(k >= 3) break;
		}
		if(buffer[i] == '\0') break;
    }
	int val_return;
	duty = atoi(temp);
	if(duty > 90){
		val_return = 3024; // 90% - pwm
	}
	else {
		val_return = (3559*duty)/100;
	}
	return val_return;
}

int Atoi_Converted(char *buffer){
	char check[11] = {'0','1','2','3','4','5','6','7','8','9'};
	char temp[2]; // max = local + NULL
	int k = 0;
	for(int i = 0; i < strlen(buffer); i++){
		for(int j = 0; j < 10; j++){
			if(buffer[i] == check[j]){
				temp[k++] = buffer[i];
				break;
			}
			if(k > 2) break;
		}
		if(buffer[i] == '\0') break;
    }
	return atoi(temp);
}

void uart6_irqhandler(const void *arg){
	ARG_UNUSED(arg);
	
    if ((USART6->SR) & (1 << 5)) {
        char rx_byte = (char)USART6->DR;
    	static char rx_buffer[32];
		static int idx = 0;
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (idx > 0) {
            	rx_buffer[idx] = '\0';
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