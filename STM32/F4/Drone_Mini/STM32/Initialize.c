/*
 * Initialize.c
 *
 *  Created on: Apr 5, 2026
 *      Author: hung
 */

#include "stdio.h"
#include "stdint.h"
#include "Initialize.h"
#include "stm32f4xx.h"
#include "bmi088.h"
#include "string.h"
#include "cmsis_os.h"


void delay_ms(uint32_t time_Delay){
	SysTick->CTRL |= (1 << 0)|(1 << 2);
	SysTick->VAL = 0;
	SysTick->LOAD = 15999; // 16MHz/1000 - 1
	for(uint32_t i = 0; i < time_Delay; i++){
		while(!(SysTick->CTRL & (1 << 16)));
	}
	SysTick->CTRL = 0;
}

void BUS_Init(void){
	RCC->AHB1ENR |= (1 << 2); // GPIOC
	RCC->AHB1ENR |= (1 << 1)|(1 << 0); // GPIOB, A
	RCC->APB1ENR |= (1 << 0)|(1 << 2); // TIMER 2-4 ENABLE - 84MHz
	RCC->APB2ENR |= (1 << 14); // SYS ENABLE
	RCC->APB2ENR |= (1 << 5); // UART 6
	RCC->AHB1ENR |= (1 << 21); // DMA1 enable

	for(volatile int i = 0; i < 100; i++); // wait for stable
}

void Init_The_Config_Of_Drone(void){
	// LED
	GPIOC->MODER &= ~(3 << 0);
	GPIOC->MODER |= (1 << 0); // out
	GPIOC->BSRR = (1 << 16); // off led
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

	TIM2->ARR = 3559;
	TIM2->PSC = 0; // F =  84MHZ/(ARR + 1)(PSC + 1); = 25KHZ
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
	TIM4->ARR = 3559;
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
	// PC13-PC14 -- External Interrupt

	GPIOC->MODER &= ~(3 << 26) &~(3 << 28); // INPUT
	EXTI->IMR |= (1 << 13)|(1 << 14);
	EXTI->RTSR |= (1 << 13)|(1 << 14);
	SYSCFG->EXTICR[3] &= ~((0xF << 4) | (0xF << 8));
	SYSCFG->EXTICR[3] |= (2 << 4)|(2 << 8);

	EXTI->PR = (1 << 13)|(1 << 14); // clear flag

	NVIC->ISER[1] |= (1 << 8);
	NVIC->IP[40] = (uint8_t)(5 << 4); // mức 4

	// Timer 3 for battery
	RCC->APB1ENR |= (1 << 1); // TIMER 3 - 84MHZ
	TIM3->CR1 &= ~(1 << 0);
	TIM3->CNT = 0;
	TIM3->PSC = 1999; // -> F = 2HZ -> T = 0,5s
	TIM3->ARR = 11999;
	TIM3->EGR |= (1 << 0);
	TIM3->DIER |= (1 << 0);
	TIM3->CR1 |= (1 << 0);
	NVIC->ISER[0] |= (1 << 29); // TIMER 3 IRQ
	NVIC->IP[29] = (uint8_t)(8 << 4); // Mức 7

	// Timer 5 for landing and takeoff -> 8MS-> 125HZ
	RCC->APB1ENR |= (1 << 3); // TIMER 5
	TIM5->CR1 &= ~(1 << 0);
	TIM5->CNT = 0;
	TIM5->PSC = 1999;
	TIM5->ARR = 335;
	TIM5->EGR |= (1 << 0);
	TIM5->DIER |= (1 << 0);
//	TIM5->CR1 |= (1 << 0); // The first state -> off
	NVIC->ISER[1] |= (1 << 18);
	NVIC->IP[50] = (uint8_t)(7 << 4); // PRIORITY = 6
};


void I2C1_ClearBus(void) {
	RCC->AHB1ENR |= (1 << 1); // GPIOB
	osDelay(2);
    // 1.PB6 (SCL) và PB7 (SDA) GPIO Output Open-drain
    GPIOB->MODER &= ~(3 << 12) &~(3 << 14);
    GPIOB->MODER |= (1 << 12)|(1 << 14); // PB6,7 Output

    GPIOB->OTYPER |= (1 << 6)|(1 << 7); // Open-drain
    GPIOB->OSPEEDR &= ~(3 << 12) &~(3 << 14);
    GPIOB->OSPEEDR |= (3 << 12)|(3 << 14);
    GPIOB->PUPDR &= ~(3 << 12) &~(3 << 14);

    GPIOB->BSRR |= (1 << 6)|(1 << 7); // High

    osDelay(2);

    for(int i = 0; i < 20; i++) {
    	if (GPIOB->IDR & (1 << 7)) { // SDA
            break;
         }

         GPIOB->BSRR = (1 << (6 + 16)); // SCL = Low
         osDelay(2);
         GPIOB->BSRR = (1 << 6);        // SCL = High
         osDelay(2);
      }

        GPIOB->BSRR = (1 << (6 + 16)); // SCL Low
        osDelay(2);
        GPIOB->BSRR = (1 << (7 + 16)); // SDA Low
        osDelay(2);

        GPIOB->BSRR = (1 << 6);        // SCL High
        osDelay(2);
        GPIOB->BSRR = (1 << 7);        // SDA High
        osDelay(2);
}

void I2C1_RST_APB(void){
	RCC->APB1RSTR |= (1 << 21); // reset I2C1
	osDelay(2);
	RCC->APB1RSTR &= ~(1 << 21);
	osDelay(2);
}

void I2C1_Initialized(void){
	RCC->APB1ENR |= (1 << 21); // ENABLE I2C1 - 42MHz
	osDelay(2);
	// PB6-SCL PB7-SDA
	GPIOB->MODER &= ~(3 << 12) &~(3 << 14);
	GPIOB->MODER |= (2 << 12)|(2 << 14);

	GPIOB->OTYPER |= (1 << 6)|(1 << 7); // OPEN DRAIN
	GPIOB->PUPDR &= ~(3 << 12) &~(3 << 14);

	GPIOB->OSPEEDR &= ~((3 << 12) | (3 << 14));
	GPIOB->OSPEEDR |= (3 << 12)|(3 << 14);

	GPIOB->AFR[0] &= ~(0x0F << 24) &~(0x0F << 28);
	GPIOB->AFR[0] |= (4 << 24)|(4 << 28); //AFR

	I2C1->CR1 &= ~(1 << 0);
	I2C1->CR2 = 42; // 42 MHz

	I2C1->CCR = (1 << 15) | 35; // FM = 42MHz /(3 * 400K);
	I2C1->TRISE = 14; // TRISE = 0.3*24 + 1
	I2C1->CR1 |= (1 << 0); // ON
	osDelay(2);
}

volatile int i2c3_count = 0;
void I2C3_ClearBus(void) {
	I2C3->CR1 &= ~(1 << 0);
	i2c3_count++;
	RCC->AHB1ENR |= (1 << 2)|(1 << 0); // GPIOC,A
	osDelay(3);

    GPIOA->MODER &= ~(3 << 16); GPIOA->MODER |= (1 << 16); // PA8 Output
    GPIOC->MODER &= ~(3 << 18); GPIOC->MODER |= (1 << 18); // PC9 Output

    GPIOA->OTYPER |= (1 << 8);  // PA8 Open-Drain
    GPIOC->OTYPER |= (1 << 9);  // PC9 Open-Drain

	GPIOA->OSPEEDR &= ~(3 << 16);
	GPIOC->OSPEEDR &= ~(3 << 18);

	GPIOA->OSPEEDR |= (3 << 16);
	GPIOC->OSPEEDR |= (3 << 18);

	GPIOA->PUPDR &= ~(3 << 16);
	GPIOC->PUPDR &= ~(3 << 18);

    // Thả SCL và SDA lên cao mặc định
    GPIOA->BSRR = (1 << 8);
    GPIOC->BSRR = (1 << 9);
    osDelay(2);

    for (int i = 0; i < 20; i++) {
        if (GPIOC->IDR & (1 << 9)) {
            break;
        }
        GPIOA->BSRR = (1 << (8 + 16)); // SCL = Low
        osDelay(2);
        GPIOA->BSRR = (1 << 8);        // SCL = High
        osDelay(2);
    }

    GPIOA->BSRR = (1 << (8 + 16)); // SCL Low
    osDelay(2);
    GPIOC->BSRR = (1 << (9 + 16)); // SDA Low
    osDelay(2);

    GPIOA->BSRR = (1 << 8);        // SCL High
    osDelay(2);
    GPIOC->BSRR = (1 << 9);        // SDA High
    osDelay(2);
}

void I2C3_RST_APB(void){
	RCC->APB1RSTR |= (1 << 23);
	osDelay(2);
	RCC->APB1RSTR &= ~(1 << 23);
	osDelay(2);
}

void I2C3_Initialized(void){
	// I2C3 - PA8-SCL - PC9-SDA
	RCC->APB1ENR |= (1 << 23); // ENABLE I2C3 - 42 MHz
	osDelay(2);

	I2C3->CR1 |= (1 << 15);
    osDelay(2);
	I2C3->CR1 &= ~(1 << 15);
    osDelay(2);

	RCC->APB1RSTR |= (1 << 23);
    osDelay(2);
	RCC->APB1RSTR &= ~(1 << 23);
    osDelay(2);

	GPIOA->MODER &= ~(3 << 16);
	GPIOC->MODER &= ~(3 << 18);

	GPIOA->MODER |= (2 << 16);
	GPIOC->MODER |= (2 << 18);

	GPIOA->OTYPER |= (1 << 8);
	GPIOC->OTYPER |= (1 << 9);

	GPIOA->OSPEEDR &= ~(3 << 16);
	GPIOC->OSPEEDR &= ~(3 << 18);

	GPIOA->OSPEEDR |= (3 << 16);
	GPIOC->OSPEEDR |= (3 << 18);

	GPIOA->PUPDR &= ~(3 << 16);
	GPIOC->PUPDR &= ~(3 << 18);

	GPIOA->AFR[1] &= ~(0xF << 0);
	GPIOC->AFR[1] &= ~(0xF << 4);

	GPIOA->AFR[1] |= (4 << 0);
	GPIOC->AFR[1] |= (4 << 4);

	I2C3->CR1 &= ~(1 << 0);
	I2C3->CR2 = 42;
//	I2C3->OAR1 = 0;
//	I2C3->OAR2 = 0;
	I2C3->CCR = 0;
	I2C3->CCR |= (1 << 15)| 35;
	I2C3->TRISE = 14;
	I2C3->CR1 |= (1 << 0);
    osDelay(3);

}

Sensor_address addr;
volatile int adr;
#include "cmsis_os.h"
extern osThreadId_t Error_Thread;

int Check_Address_I2C3(void){
    addr.sensor3 = 0;
    addr.sensor4 = 0;

    int nos = 0;
    for(adr = 1; adr < 128; adr++){
    	uint32_t timeout = 10000;
        while ((I2C3->SR2 & (1 << 1)) && --timeout){};
		if(timeout == 0) {
			return 0;
		}
        I2C3->CR1 |= (1 << 8);
    	timeout = 10000;
        while (!(I2C3->SR1 & (1 << 0)) && --timeout){};
		if(timeout == 0) {
			return 0;
		}
        I2C3->DR = (adr << 1);
    	timeout = 10000;
        while (!(I2C3->SR1 & ((1 << 1) | (1 << 10))) && --timeout){};
		if(timeout == 0) {
			return 0;
		}
        if(I2C3->SR1 & (1 << 1)){
            nos++;
            if(nos == 1) addr.sensor3 = adr;
            else if(nos == 2) addr.sensor4 = adr;

            (void)I2C3->SR1;
            (void)I2C3->SR2;
        }

        if(I2C3->SR1 & (1 << 10)){
            I2C3->SR1 &= ~(1 << 10);
        }

        I2C3->CR1 |= (1 << 9);
        timeout = 10000;
        while ((I2C3->CR1 & (1 << 9)) && --timeout){};
		if(timeout == 0) {
			return 0;
		}

        if(nos == 2) break;
    };
    return (nos == 2) ? 1 : 0;
}

int Check_Address_I2C1(void){ // HAVE 2 SENSORS
	addr.sensor1 = 0;
	addr.sensor2 = 0;

	int nos = 0;
	for(int ADDR = 1; ADDR < 128; ADDR++){
	    uint32_t timeout = 10000;
		while((I2C1->SR2 & (1 << 1)) && --timeout);
		if(timeout == 0) {
			I2C1->CR1 |= (1 << 9);
			I2C1->CR1 &= ~(1 << 0); // off PE
			return 0;
		}

		I2C1->CR1 |= (1 << 8); // START-BIT
		timeout = 10000;
		while(!(I2C1->SR1 & (1 << 0)) && --timeout); // WAIT FOR START-BIT
		if(timeout == 0) {
			I2C1->CR1 |= (1 << 9);
			I2C1->CR1 &= ~(1 << 0); // off PE
			return 0;
		}

		I2C1->DR = (ADDR << 1);
		timeout = 10000;
		while(!(I2C1->SR1 & ((1 << 1)|(1 << 10))) && --timeout);
		if(timeout == 0) {
			I2C1->CR1 |= (1 << 9);
			I2C1->CR1 &= ~(1 << 0); // off PE
			return 0;
		}
		if(I2C1->SR1 & (1 << 1)){
			nos++;
			if(nos == 1) addr.sensor1 = ADDR;
			else addr.sensor2 = ADDR;
			(void)I2C1->SR1;
			(void)I2C1->SR2;

		}
        if(I2C1->SR1 & (1 << 10)){
            I2C1->SR1 &= ~(1 << 10);
        }

		timeout = 10000;
		I2C1->CR1 |= (1 << 9); // STOP
		while((I2C1->CR1 & (1 << 9)) &&--timeout);
		if(timeout == 0) {
			return 0;
		}
		if(nos == 1) break;
	}

	return (nos == 1) ? 1 : 0;
}

extern uint8_t dma_gyro_buffer[6];
extern uint8_t dma_acc_buffer[6];

void DMA_I2C3_Stream(void){
	// channel 3 stream 2 I2C3_RX
	// channel 3 stream 4 I2C3_TX
	RCC->AHB1ENR |= (1 << 21); // DMA1 enable
	for(volatile int i = 0; i < 1000;i++);

	DMA1_Stream2->CR &= ~(1 << 0); // off to config
	while((DMA1_Stream2->CR & (1 << 0)));
	DMA1->LIFCR = (0x3D << 16); // clear

	DMA1_Stream2->PAR = (uint32_t)&(I2C3->DR);

	uint32_t config = 0;
	config |= (3 << 25); // channel 3
	config |= (3 << 16); // High priority
	config &= ~(3 << 13); // 8 bit memory
	config &= ~(3 << 12); // 8bit for peripheral
	config |= (1 << 10); // memory increment mode
	config &= ~(1 << 9);
	config &= ~(1 << 8); // normal
	config &= ~(3 << 6); // peripheral to mem
	config |= (1 << 4); // interrupt

	DMA1_Stream2->CR = config;
	DMA1_Stream2->FCR = 0;

	NVIC_SetPriority(DMA1_Stream2_IRQn,6); // high
	NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

#include "cmsis_os.h"
extern osThreadId_t bmi088_thread;
extern osMutexId_t bmi088_mutex;

void EXTI15_10_IRQHandler(void){
	if(EXTI->PR & (1 << 13)){ // acc
		EXTI->PR = (1 << 13);
	}
	if(EXTI->PR & (1 << 14)){
		EXTI->PR = (1 << 14);
		if(bmi088_thread != NULL){
			osThreadFlagsSet(bmi088_thread, 0x0001);
		}
	}
}

// Battery
extern osThreadId_t BatteryReadTaskHandle;
volatile int time3_count = 0;
void TIM3_IRQHandler(void){
	if(TIM3->SR & (1 << 0)){
		// clear flag
		TIM3->SR &= ~(1 << 0);
		time3_count++;
		osThreadFlagsSet(BatteryReadTaskHandle, BATTERY);
	}
}

extern int Landing_Takeoff_State;
volatile int time5_count = 0;
extern osThreadId_t Motor_Thread;
void TIM5_IRQHandler(void){

	if(TIM5->SR & (1 << 0)){
		time5_count++;
		TIM5->SR &= ~(1 << 0);
		if(Landing_Takeoff_State == 1){
			osThreadFlagsSet(Motor_Thread, LANDING); // landing
		}
		else if(Landing_Takeoff_State == 2) {
			osThreadFlagsSet(Motor_Thread, FLY_FROM_BASE); // takeoff
		}
	}
}

void DMA1_Stream2_IRQHandler(void){
	    // Kiểm tra cờ TCIF2 (Bit 21)
	 if(DMA1->LISR & (1 << 21)){
	    DMA1->LIFCR = (0x3D << 16); // clear flag
	    I2C3->CR2 &= ~(1 << 11);
	     osThreadFlagsSet(bmi088_thread,0x0002);
	 }
}

void The_First_State(void){
	for(int i = 0; i < 5; i++){
		switch(i)
		{
			case 0: // M1
				TIM2->CCR1 = 0;
				TIM2->CCR2 = 0;
				TIM2->CCR4 = 300;
				TIM4->CCR4 = 0;
			    osDelay(100);
				break;
			case 1: // M2
				TIM2->CCR1 = 0;
				TIM2->CCR2 = 300;
				TIM2->CCR4 = 0;
				TIM4->CCR4 = 0;
				osDelay(100);
				break;
			case 2: // M3
				TIM2->CCR1 = 0;
				TIM2->CCR2 = 0;
				TIM2->CCR4 = 0;
				TIM4->CCR4 = 300;
				osDelay(100);
				break;
			case 3:
				TIM2->CCR1 = 300;
				TIM2->CCR2 = 0;
				TIM2->CCR4 = 0;
				TIM4->CCR4 = 0;
				osDelay(100);
				break;
			default:
				TIM2->CCR1 = 0;
				TIM2->CCR2 = 0;
				TIM2->CCR4 = 0;
				TIM4->CCR4 = 0;
				osDelay(100);
				break;
		}
	}
	osThreadFlagsSet(Motor_Thread, 1);
}
