# The Configuration - I2C3 - 400KHz
#include "stm32f4xx.h"
#include "stdio.h"

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
