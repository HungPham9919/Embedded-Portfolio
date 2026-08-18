#include "Init_reg.h"
#include "RTOS/RTOS_Init.h"
#include "stm32f405xx.h"
#include "zephyr/irq.h"
#include "zephyr/kernel.h"

void exti0_irqhandler(const void *arg);
void exti9_5irqhandler(const void *arg);
void exti15_10irqhandler(const void *arg);
void tim5_irqhandler(const void *arg);
void dma1_stream2_irqhandler(const void *arg);
void tim3_irqhandler(const void *arg);
void dma1_stream5_irqhandler(const void *arg);
void dma1_stream4_irqhandler(const void *arg);
void dma1_stream6_irqhandler(const void *arg);

void BUS_Init(void){
	RCC->AHB1ENR |= (1 << 1)|(1 << 0)|(1 << 2); // GPIOB, A,C
	RCC->APB1ENR |= (1 << 0)|(1 << 2); // TIMER 2-4 ENABLE - 84MHz
	RCC->APB2ENR |= (1 << 14); // SYS ENABLE
	RCC->APB2ENR |= (1 << 5); // UART 6
	// RCC->AHB1ENR |= (1 << 21); // DMA1 enable

	for(volatile int i = 0; i < 100; i++); // wait for stable
}

void Init_The_Config_Of_Drone(void){
	// LED
	GPIOC->MODER &= ~(3 << 2); // pC1
	GPIOC->MODER |= (1 << 2); // out
	GPIOC->BSRR = (1 << 17); // off led
	// Timer for motor Timer 2 CH 24- Timer 4 CH4
	// PB 9-11 PA 1-15

	GPIOA->MODER &= ~(3 << 2) &~(3 << 30);
	GPIOA->MODER |= (2 << 2)|(2 << 30);
	GPIOB->MODER &= ~(3 << 18) &~(3 << 22); // AF mode
	GPIOB->MODER |= (2 << 18)|(2 << 22);	// AF mode

	GPIOA->AFR[0] &= ~(0x0F << 4);
	GPIOA->AFR[0] |= (1 << 4);  // PA1 -> AF1 (TIM2_CH2)
	GPIOA->AFR[1] &= ~(0x0F << 28);
	GPIOA->AFR[1] |= (1 << 28); // PA15 -> AF1 (TIM2_CH1)

	GPIOB->AFR[1] &= ~(0x0F << 4) &~(0x0F << 12);
	GPIOB->AFR[1] |= (2 << 4)|(1 << 12); // PB9 AF2 - PB11-AF1

	//84 MHz
	TIM2->ARR = 1679;
	TIM2->PSC = 0; // F =  84MHZ/(ARR + 1)(PSC + 1); = 50KHZ
	TIM2->CNT = 0;
	TIM2->DIER = 0;
	TIM2->CCER &= ~(1 << 0) &~(1 << 4) &~(1 << 12);
	TIM2->CCER |= (1 << 0)|(1 << 4)|(1 << 12);

	TIM2->CCMR1 &= ~((7 << 4) | (1 << 3) | (7 << 12) | (1 << 11));
	TIM2->CCMR1 |= (6 << 4)|(6 << 12)|(1 << 3)|(1 << 11); // OCPE CHANNEL 1 & 2 PWM MODE

	TIM2->CCMR2 &= ~((7 << 12) | (1 << 11));
	TIM2->CCMR2 |= (6 << 12)|(1 << 11); // CHANNEL 4 - OCPE

	TIM2->CCR1 = 0; 							// MOTOR 2
	TIM2->CCR2 = 0;								// MOTOR 4
	TIM2->CCR4 = 0;								// MOTOR 3

	TIM2->EGR |= (1 << 0);
	TIM2->CR1 |= (1 << 0);

	// TIMER 4 - 84MHz
	TIM4->ARR = 1679;
	TIM4->PSC = 0;
	TIM4->CNT = 0;

	TIM4->DIER = 0;
	TIM4->CCER &= ~(1 << 12);
	TIM4->CCER |= (1 << 12);

	TIM4->CCMR2 &= ~((1 << 11)|(7 << 12));
	TIM4->CCMR2 |= (6 << 12)|(1 << 11); // OCPE

	TIM4->CCR4 = 0; 							// MOTOR 1

	TIM4->EGR |= (1 << 0);
	TIM4->CR1 |= (1 << 0);

	// PC0 HMC5883 - DRDY - Active low - EXTI-0
	GPIOC->MODER &= ~(3 << 0);
	SYSCFG->EXTICR[0] &= ~(0x0F << 0);
	SYSCFG->EXTICR[0] |= (2 << 0);
	EXTI->FTSR |= (1 << 0);
	EXTI->PR |= (1 << 0);
	EXTI->IMR |= (1 << 0);
	irq_connect_dynamic(6, 6, exti0_irqhandler, NULL, 0);
	irq_enable(6);
	// PC8 - EXTI8 for PMW3901
    // 4. Cấu hình PC8 (EXTI / MOT) -> Input Pull-up - Pheriperal has been enabled
    GPIOC->MODER &= ~(3 << 16);
	SYSCFG->EXTICR[2] &= ~(0x0F << 0);
	SYSCFG->EXTICR[2] |= (2 << 8);
    EXTI->FTSR |= (1 << 8); // Falling trigger
	EXTI->PR |= (1 << 8);
    EXTI->IMR |= (1 << 8); // enable

	irq_connect_dynamic(23, 5, exti9_5irqhandler, NULL, 0);
	irq_enable(23);
	// PC11 - Alert - Active low - EXTI11
	GPIOC->MODER &= ~(3 << 22); // PC11
	SYSCFG->EXTICR[2] &= ~(0x0F << 12);
	SYSCFG->EXTICR[2] |= (2 << 12);

	EXTI->FTSR |= (1 << 11);
	EXTI->PR |= (1 << 11);
	EXTI->IMR |= (1 << 11);

	// PC13-PC14 -- External Interrupt

	GPIOC->MODER &= ~(3 << 26) &~(3 << 28); // INPUT

	SYSCFG->EXTICR[3] &= ~((0xF << 4) | (0xF << 8));
	SYSCFG->EXTICR[3] |= (2 << 4)|(2 << 8);

	EXTI->RTSR |= (1 << 13)|(1 << 14); // active high
	EXTI->PR = (1 << 13)|(1 << 14); // clear flag

	irq_connect_dynamic(40, 5, exti15_10irqhandler, NULL, 0);
	irq_enable(40);
	// Timer 3 for battery

	RCC->APB1ENR |= (1 << 1); // TIMER 3 - 84MHZ
	TIM3->CR1 &= ~(1 << 0);
	TIM3->CNT = 0;
	TIM3->PSC = 1999; // -> F = 2HZ -> T = 0,5s
	TIM3->ARR = 11999;
	TIM3->EGR |= (1 << 0); // update -> count++
	TIM3->SR &= ~(1 << 0);		//xóa cờ ngắt update
	TIM3->DIER |= (1 << 0);
	TIM3->CR1 |= (1 << 0); // enable

	IRQ_CONNECT(29,8,tim3_irqhandler,NULL,0);
	irq_enable(29);

	// Timer 5 for landing and takeoff -> 8MS-> 125HZ
	RCC->APB1ENR |= (1 << 3); // TIMER 5
	TIM5->CR1 &= ~(1 << 0);
	TIM5->CNT = 0;
	TIM5->PSC = 1999;
	TIM5->ARR = 335;
	TIM5->EGR |= (1 << 0);
	TIM5->SR &= ~(1 << 0); 	// xóa cờ ngắt update
	TIM5->DIER |= (1 << 0);
	TIM5->CR1 |= (1 << 0);

	IRQ_CONNECT(50,7,tim5_irqhandler,NULL,0);
	irq_enable(50);

	IRQ_CONNECT(DMA1_Stream2_IRQn, 2, dma1_stream2_irqhandler, NULL, 0);
    irq_enable(DMA1_Stream2_IRQn);

    IRQ_CONNECT(DMA1_Stream4_IRQn, 2, dma1_stream4_irqhandler, NULL, 0);
    irq_enable(DMA1_Stream4_IRQn);

    IRQ_CONNECT(DMA1_Stream5_IRQn, 2, dma1_stream5_irqhandler, NULL, 0);
    irq_enable(DMA1_Stream5_IRQn);

    IRQ_CONNECT(DMA1_Stream6_IRQn, 2, dma1_stream6_irqhandler, NULL, 0);
    irq_enable(DMA1_Stream6_IRQn);

};

void exti0_irqhandler(const void *arg){
	ARG_UNUSED(arg);
	if(EXTI->PR & (1 << 0)){
		EXTI->PR = (1 << 0);
		// send signal
	}
}

volatile int exti8_count = 0;
void exti9_5irqhandler(const void *arg){
	ARG_UNUSED(arg);

	if(EXTI->PR & (1 << 8)){
		EXTI->PR = (1 << 8); // clear pending
		exti8_count++;
		k_sem_give(&pmw3901_signal);
	}
}

volatile int exti15_10_count = 0, exti_11_count = 0;
void exti15_10irqhandler(const void *arg){
	ARG_UNUSED(arg);
	
	if(EXTI->PR & (1 << 11)){ // PC11
		EXTI->PR = (1 << 11);
		exti_11_count++;
		// k_sem_give(&ina226_signal);
	}

	if(EXTI->PR & (1 << 13)){ // acc
		EXTI->PR = (1 << 13);
	}
	if(EXTI->PR & (1 << 14)){
		EXTI->PR = (1 << 14);
		exti15_10_count++;
		k_sem_give(&bmi088_signal);
	}
}

volatile int tim5_count = 0;
void tim5_irqhandler(const void *arg){
	ARG_UNUSED(arg);
	if(TIM5->SR & (1 << 0)){
		tim5_count++;
		TIM5->SR &= ~(1 << 0);
	}
}

volatile int tim3_count = 0;
void tim3_irqhandler(const void *arg){
	ARG_UNUSED(arg);
	if(TIM3->SR & (1 << 0)){
		// clear flag
		tim3_count++;
		TIM3->SR &= ~(1 << 0);
	}
}

void dma1_stream2_irqhandler(const void *arg){
	ARG_UNUSED(arg);
	if(DMA1->LISR & (1 << 21)){
	    DMA1->LIFCR = (0x3D << 16); // clear flag
		k_sem_give(&dma1_stream2_signal);
	}
}

void dma1_stream4_irqhandler(const void *arg){
	ARG_UNUSED(arg);
	if(DMA1->HISR & (1 << 5)){
	    DMA1->HIFCR = (0x3D << 0); // clear flag
		k_sem_give(&dma1_stream4_signal);
	}
}

void dma1_stream5_irqhandler(const void *arg){
	ARG_UNUSED(arg);
	if(DMA1->HISR & (1 << 11)){
        DMA1->HIFCR = (0x3D << 6); // clear flag
		k_sem_give(&dma1_stream5_signal);
	}
}

void dma1_stream6_irqhandler(const void *arg){
	ARG_UNUSED(arg);
	if(DMA1->HISR & (1 << 21)){
        DMA1->HIFCR = (0x3D << 16); // clear flag
		k_sem_give(&dma1_stream6_signal);
	}
}

void The_First_State(void){
	for(int i = 0; i < 5; i++){
		switch(i)
		{
			case 0: // M1
				TIM2->CCR1 = 0;
				TIM2->CCR2 = 0;
				TIM2->CCR4 = 0;
				TIM4->CCR4 = 100;
	            k_msleep(100);
				break;
			case 1: // M2
				TIM2->CCR1 = 100;
				TIM2->CCR2 = 0;
				TIM2->CCR4 = 0;
				TIM4->CCR4 = 0;
	            k_msleep(100);
				break;
			case 2: // M3
				TIM2->CCR1 = 0;
				TIM2->CCR2 = 0;
				TIM2->CCR4 = 100;
				TIM4->CCR4 = 0;
	            k_msleep(100);
				break;
			case 3:
				TIM2->CCR1 = 0;
				TIM2->CCR2 = 100;
				TIM2->CCR4 = 0;
				TIM4->CCR4 = 0;
	            k_msleep(100);
				break;
			default:
				TIM2->CCR1 = 0;
				TIM2->CCR2 = 0;
				TIM2->CCR4 = 0;
				TIM4->CCR4 = 0;
	            k_msleep(100);
				break;
		}
	}
}

