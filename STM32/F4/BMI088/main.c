# The Configuration - I2C3 - 400KHz
#include "stm32f4xx.h"
#include "stdio.h"
#include "bmi088.h"

void delay_ms(uint32_t time_Delay){
	SysTick->CTRL |= (1 << 0)|(1 << 2);
	SysTick->VAL = 0;
	SysTick->LOAD = 15999; 			// 16MHz/1000 - 1
	for(uint32_t i = 0; i < time_Delay; i++){
		while(!(SysTick->CTRL & (1 << 16)));
	}
	SysTick->CTRL = 0;
}

void Initialization(void){
  // I2C3 - PA8-SCL - PC9-SDA
	RCC->AHB1ENR |= (1 << 0)|(1 << 2); // Enable GPIPO- AC
	RCC->APB1ENR |= (1 << 23); // ENABLE I2C3 - 24MHz

	GPIOA->MODER &= ~(3 << 16);
	GPIOC->MODER &= ~(3 << 18);

	GPIOA->MODER |= (2 << 16);
	GPIOC->MODER |= (2 << 18);

	GPIOA->OTYPER |= (1 << 8);
	GPIOC->OTYPER |= (1 << 9);

	GPIOA->OSPEEDR |= (3 << 16);
	GPIOC->OSPEEDR |= (3 << 18);

	GPIOA->PUPDR |= (1 << 16);
	GPIOC->PUPDR |= (1 << 18);

	GPIOA->AFR[1] |= (4 << 0);
	GPIOC->AFR[1] |= (4 << 4);

	I2C3->CR1 &= ~(1 << 0);
	I2C3->CR2 |= 24;
	I2C3->CCR = (1 << 15)| 20;
	I2C3->TRISE = 9;
	I2C3->CR1 |= (1 << 0);
}

void main(void){
	Initialization();
	drone_angle.Roll_angle = 0.0f;
	drone_angle.Pitch_angle = 0.0f;
	drone_angle.Yaw_angle = 0.0f;
	
	GYRO_ACC_LSB(sens.acc_range, sens.gyro_range); // Determine the LSB
	BMI088_Initialize();
	delay_ms(50);
	BMI088_Calib();
	delay_ms(50);

	uint8_t acc_data[6] = {0};
    uint8_t gyro_data[6] = {0};
	float dt = 0.005f; // Because i choose the ODR is 200Hz -> So the time is 1/200
	Read_Data(GYRO_ADDR, GYRO_Data, gyro_data, 6);
    Read_Data(ACC_ADDR, ACC_Data, acc_data, 6);
    Calculate_And_Filter_Angle(acc_data,gyro_data, dt);
}

