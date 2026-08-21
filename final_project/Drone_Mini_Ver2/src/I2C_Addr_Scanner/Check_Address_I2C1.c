#include "Check_Address_I2C1.h"
volatile DRONE_SENSOR drone_sensor_addr;

void I2C1_ClearBus(void) {
	RCC->AHB1ENR |= (1 << 1); // GPIOB
	k_msleep(2);

    GPIOB->MODER &= ~(3 << 12) &~(3 << 14);
    GPIOB->MODER |= (1 << 12)|(1 << 14); // PB6,7 Output

    GPIOB->OTYPER |= (1 << 6)|(1 << 7); // Open-drain
    GPIOB->OSPEEDR &= ~(3 << 12) &~(3 << 14);
    GPIOB->OSPEEDR |= (3 << 12)|(3 << 14);
    GPIOB->PUPDR &= ~(3 << 12) &~(3 << 14);

    GPIOB->BSRR |= (1 << 6)|(1 << 7); // High

	k_msleep(2);

    for(int i = 0; i < 20; i++) {
    	if (GPIOB->IDR & (1 << 7)) { // SDA
            break;
        }
        GPIOB->BSRR = (1 << (6 + 16)); // SCL = Low
        k_msleep(2);
        GPIOB->BSRR = (1 << 6);        // SCL = High
	    k_msleep(2);
        }

        GPIOB->BSRR = (1 << (6 + 16)); // SCL Low
	    k_msleep(2);
        GPIOB->BSRR = (1 << (7 + 16)); // SDA Low
	    k_msleep(2);

        GPIOB->BSRR = (1 << 6);        // SCL High
	    k_msleep(2);
        GPIOB->BSRR = (1 << 7);        // SDA High
        k_msleep(2);
}

void I2C1_RST_APB(void){
	RCC->APB1RSTR |= (1 << 21); // reset I2C1
	k_msleep(2);
	RCC->APB1RSTR &= ~(1 << 21);
	k_msleep(2);
}

void I2C1_Initialized(void){
	RCC->AHB1ENR |= (1 << 1); // GPIOB
	RCC->APB1ENR |= (1 << 21); // ENABLE I2C1 - 42MHz
	k_msleep(2);
	RCC->AHB1ENR |= (1 << 1);
	k_msleep(2);

	I2C1->CR1 |= (1 << 15);
	k_msleep(2);
	I2C1->CR1 &= ~(1 << 15);
	k_msleep(2);

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
	k_msleep(2);
}

volatile int i2c3_count = 0;
void I2C3_ClearBus(void) {
	I2C3->CR1 &= ~(1 << 0);
	i2c3_count++;
	RCC->AHB1ENR |= (1 << 2)|(1 << 0); // GPIOC,A
	k_msleep(2);

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
	k_msleep(2);

    for (int i = 0; i < 20; i++) {
        if (GPIOC->IDR & (1 << 9)) {
            break;
        }
        GPIOA->BSRR = (1 << (8 + 16)); // SCL = Low
	    k_msleep(2);
        GPIOA->BSRR = (1 << 8);        // SCL = High
	    k_msleep(2);
    }

    GPIOA->BSRR = (1 << (8 + 16)); // SCL Low
	k_msleep(2);
    GPIOC->BSRR = (1 << (9 + 16)); // SDA Low
	k_msleep(2);

    GPIOA->BSRR = (1 << 8);        // SCL High
	k_msleep(2);
    GPIOC->BSRR = (1 << 9);        // SDA High
	k_msleep(2);
}

void I2C3_RST_APB(void){
	RCC->APB1RSTR |= (1 << 23);
	k_msleep(2);
	RCC->APB1RSTR &= ~(1 << 23);
	k_msleep(2);
}

void I2C3_Initialized(void){
	// I2C3 - PA8-SCL - PC9-SDA
    RCC->AHB1ENR |= (1 << 2)|(1 << 0); // GPIOC,A
	RCC->APB1ENR |= (1 << 23); // ENABLE I2C3 - 42 MHz
	k_msleep(2);

	I2C3->CR1 |= (1 << 15);
	k_msleep(2);
	I2C3->CR1 &= ~(1 << 15);
	k_msleep(2);

	RCC->APB1RSTR |= (1 << 23);
	k_msleep(2);
	RCC->APB1RSTR &= ~(1 << 23);
	k_msleep(2);

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
	I2C3->CCR = 0;
	I2C3->CCR |= (1 << 15)| 35;
	I2C3->TRISE = 14;
	I2C3->CR1 |= (1 << 0);
	k_msleep(2);
}

int Check_Address_I2C3(void){
    drone_sensor_addr.sensor4 = 0;
    drone_sensor_addr.sensor5 = 0;
	drone_sensor_addr.sensor6 = 0;
	drone_sensor_addr.sensor7 = 0;

    int nos = 0;
    for(int adr = 1; adr < 128; adr++){
    	uint32_t timeout = 10000;
        while ((I2C3->SR2 & (1 << 1)) && --timeout){};
		if(timeout == 0) {
			I2C3->CR1 |= (1 << 9);
			return 0;
		}
        I2C3->CR1 |= (1 << 8);
    	timeout = 10000;
        while (!(I2C3->SR1 & (1 << 0)) && --timeout){};
		if(timeout == 0) {
			I2C3->CR1 |= (1 << 9);
			return 0;
		}
        I2C3->DR = (adr << 1);
    	timeout = 10000;
        while (!(I2C3->SR1 & ((1 << 1) | (1 << 10))) && --timeout){};
		if(timeout == 0) {
			I2C3->CR1 |= (1 << 9);
			return 0;
		}
        if(I2C3->SR1 & (1 << 1)){
            nos++;

			if(nos == 1) drone_sensor_addr.sensor4 = adr;
			else if (nos == 2) drone_sensor_addr.sensor5 = adr;
			else if(nos == 3) drone_sensor_addr.sensor6 = adr;
			else drone_sensor_addr.sensor7 = adr;

            (void)I2C3->SR1;
            (void)I2C3->SR2;
        }

        if(I2C3->SR1 & (1 << 10)){
            I2C3->SR1 &= ~(1 << 10);
        }

        I2C3->CR1 |= (1 << 9);

        if(nos == 3) break;
    };
    return (nos == 3) ? 1 : 0;
}

int Check_Address_I2C1(void){
	drone_sensor_addr.sensor1 = 0;
	drone_sensor_addr.sensor2 = 0;
	drone_sensor_addr.sensor3 = 0;

	int nos = 0;
	for(int ADDR = 1; ADDR < 128; ADDR++){
	    uint32_t timeout = 10000;
		while((I2C1->SR2 & (1 << 1)) && --timeout);
		if(timeout == 0) {
			I2C1->CR1 |= (1 << 9); // STOP
			return 0;
		}

		I2C1->CR1 |= (1 << 8); // START-BIT
		timeout = 10000;
		while(!(I2C1->SR1 & (1 << 0)) && --timeout); // WAIT FOR START-BIT
		if(timeout == 0) {
			I2C1->CR1 |= (1 << 9); // STOP
			return 0;
		}

		I2C1->DR = (ADDR << 1);
		timeout = 10000;
		while(!(I2C1->SR1 & ((1 << 1)|(1 << 10))) && --timeout);
		if(timeout == 0) {
			I2C1->CR1 |= (1 << 9); // STOP
			return 0;
		}
		if(I2C1->SR1 & (1 << 1)){
			nos++;
			if(nos == 1) drone_sensor_addr.sensor1 = ADDR;
			else if(nos == 2) drone_sensor_addr.sensor2 = ADDR;
			else drone_sensor_addr.sensor3 = ADDR;

			(void)I2C1->SR1;
			(void)I2C1->SR2;

		}
        if(I2C1->SR1 & (1 << 10)){
            I2C1->SR1 &= ~(1 << 10);
        }

		I2C1->CR1 |= (1 << 9); // STOP
		if(nos == 3) break;
	}
	return (nos == 3) ? 1 : 0;
}
