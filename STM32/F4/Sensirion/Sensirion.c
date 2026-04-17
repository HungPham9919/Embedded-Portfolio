/*
 * Sensirion_Sensor.c
 *
 *  Created on: Mar 11, 2026
 *      Author: hung
 */
#include "Sensirion_Sensor.h"


void delay_ms(uint32_t delay){
	SysTick->LOAD = 15999;
	SysTick->CTRL |= (1 << 0)|(1 << 2);
	SysTick->VAL = 0;
	for(uint32_t i = 0;i < delay; i++){
		while(!(SysTick->CTRL & (1 << 16)));
	}
	SysTick->CTRL = 0;
}

void Config(void){
	  RCC->AHB1ENR |= (1 << 0)|(1 << 2); // Enable GPIOA, C
	  RCC->APB1ENR |= (1 << 23); // I2C3

	  GPIOA->MODER &= ~(3 << 16);
	  GPIOC->MODER &= ~(3 << 18);

	  GPIOA->MODER |= (2 << 16); // AF MODE
	  GPIOC->MODER |= (2 << 18);

	  GPIOA->OSPEEDR |= (2 << 16);
	  GPIOC->OSPEEDR |= (2 << 18);

	  GPIOA->OTYPER |= (1 << 8); // open drain
	  GPIOC->OTYPER |= (1 << 9);

	  GPIOA->PUPDR |= (1 << 16);
	  GPIOC->PUPDR |= (1 << 18);

	  GPIOA->AFR[1] |= (4 << 0);
	  GPIOC->AFR[1] |= (4 << 4);

	  I2C3->CR1 &= ~(1 << 0);
	  I2C3->CR2 = 16;
	  I2C3->CCR = 80;
	  I2C3->TRISE = 17;

	  I2C3->CR1 |= (1 << 0);
}

uint8_t Scan_Address(void){
	uint8_t addr;
	for(uint8_t i = 1; i < 128; i++){
		I2C3->CR1 |= (1 << 8); // START BIT
		while(!(I2C3->SR1 & (1 << 0)));

		I2C3->DR = (i << 1);

		while(!(I2C3->SR1 & ((1 << 1)|(1 << 10))));
		if(I2C3->SR1 & (1 << 1)){
			addr = i;
			(void)I2C3->SR1;
			(void)I2C3->SR2;
			I2C3->CR1 |= (1 << 9);
		}
		else {
			I2C3->SR1 &= ~(1 << 10);
			I2C3->CR1 |= (1 << 9);
		}

		for(int i = 0; i < 1000; i++);
	}
	return addr;
}

uint8_t CRC_cal(uint8_t *data, int len){
	uint8_t crc = 0xFF;
	for(int i = 0; i < len;i++){
		crc ^= data[i];
		for(int j = 0;j < 8;j++){
			if(crc & 0x80){
				crc = (crc << 1) ^ 0x31;
			}
			else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

void Read_Register(uint8_t slave_id ,uint16_t reg, uint8_t *data, int byte_length, uint32_t ms){
	I2C3->CR1 |= (1 << 8);
	while(!(I2C3->SR1 & (1 << 0)));

	I2C3->DR  = (slave_id << 1);
	while(!(I2C3->SR1 & (1 << 1))){
		if(I2C3->SR1 & (1 << 10)){
			I2C3->SR1 &= ~(1 << 10);
			I2C3->CR1 |= (1 << 9);
			return;
		}
	};
	(void)I2C3->SR1;
	(void)I2C3->SR2;

	I2C3->DR = (uint8_t)((reg >> 8) & 0xFF);
	while(!(I2C3->SR1 & (1 << 7)));
	I2C3->DR = (uint8_t)(reg & 0xFF);
	while(!(I2C3->SR1 & (1 << 2))); // BTF
	I2C3->CR1 |= (1 << 9);

	delay_ms(ms);
	//read function
	I2C3->CR1 |= (1 << 8);
	while(!(I2C3->SR1 & (1 << 0)));

	I2C3->DR = (slave_id << 1) | 1; // read mode
	while(!(I2C3->SR1 & ((1 << 1)|(1 << 10))));
	if(I2C3->SR1 & (1 << 10)){
	    I2C3->SR1 &= ~(1 << 10);
	    I2C3->CR1 |= (1 << 9);
	    return;
	}

	if(byte_length == 1){
		I2C3->CR1 &= ~(1 << 10);
		(void)I2C3->SR1;
		(void)I2C3->SR2;
		I2C3->CR1 |= (1 << 9); // stop
	}
	else {
		I2C3->CR1 |= (1 << 10);
		(void)I2C3->SR1;
		(void)I2C3->SR2;
	}

	for(int i = 0; i < byte_length; i++){
		if(i == byte_length - 1 && byte_length > 1){
			I2C3->CR1 &= ~(1 << 10);
			I2C3->CR1 |= (1 << 9);
		}
		while(!(I2C3->SR1 & (1 << 6)));
		data[i] = I2C3->DR;
	}
}


uint32_t See_Status_Of_Device(uint32_t ds){
	uint8_t status[6];
	Read_Register(SEN66_ADDR, DEVICE_STATUS, status, sizeof(status),20);
	return ds = ((uint32_t)status[0] << 24) |
	                  ((uint32_t)status[1] << 16) |
	                  ((uint32_t)status[3] << 8)  |
	                  ((uint32_t)status[4]);
	  // Value is 0x00 -> Normal operation
}

void Send_command_write(uint8_t slave_id, uint8_t *data, int length){
	I2C3->CR1 |= (1 << 8);
	while(!(I2C3->SR1 & (1 << 0)));

	I2C3->DR = (slave_id << 1);
	while(!(I2C3->SR1 & ((1 << 1)|(1 << 10))));

	if(I2C3->SR1 & (1 << 10)){
		I2C3->SR1 &= ~(1 << 10);
		I2C3->CR1 |= (1 << 9);
		return;
	}
	(void)I2C3->SR1;
	(void)I2C3->SR2;

	for(uint8_t i = 0; i < length; i++){
		while(!(I2C3->SR1 & (1 << 7))); // TXE
		I2C3->DR = data[i];
	}
	while(!(I2C3->SR1 & (1 << 2))); // BTF
	I2C3->CR1 |= (1 << 9);
}

/*
 * 9 cases
 * 0 -> crc = i + 2
 * 1 -> crc = i + 2*2
 * 2 -> crc = i + 2*3
 * 3 -> crc = i + 2*4
 *n -> crc = n + 2*(n+1);
 *
 *	byte data: 2 byte -> CRC 01,34,67,9 10,
 *
 *	i = 0: response[0] = buff[i], response[1] = buff[i+1]   -> crc -> compare buff[2]; PM1.0
 *	i = 1: response[0] = buff[i+2], response[1] = buff[i+3] -> crc -> compare buff[5]; PM2.5
 *	i = 2: response[0] = buff[i+4], response[1] = buff[i+5] -> crc -> compare buff[7]; PM4.0
 *
 *	i = n: response[0] = buff[i+(i*2)], response[1] = buff[i+(i*2 + 1)]
 *
*/

void Start_Or_Stop_Sensor(uint16_t cmd, uint32_t wait_ms){ // Start and stop measuring
	uint8_t frame[2];
	frame[0] = (uint8_t)((cmd >> 8) & 0xFF); // high byte
	frame[1] = (uint8_t)(cmd & 0xFF); // low byte

	Send_command_write(SEN66_ADDR, frame, sizeof(frame));
	delay_ms(wait_ms);
}

uint16_t combine_data(uint8_t data1,uint8_t data2){
	return (uint16_t)((data1 << 8)|(data2));
}

void Return_Value_Of_Sensor(Sensor_data *data){
	uint8_t buff[27]; // 27 byte
	uint8_t response[2];
	uint8_t crc_cal[9], crc_check[9]; // 9byte crc

	Read_Register(SEN66_ADDR, Read_Measure_Value, buff, sizeof(buff), 30);

	for(int i = 0; i < sizeof(crc_cal);i++){
		response[0] = buff[i + (i*2)];
		response[1] = buff[i + (i*2 + 1)];
		crc_cal[i] = CRC_cal(response, sizeof(response));

		crc_check[i] = buff[i + 2*(i+1)];

		if(crc_cal[i] != crc_check[i]){
			switch(i){
				case 0:
					data->pm1 = 0xFFFF;
					break;
				case 1:
					data->pm2_5 = 0xFFFF;
					break;
				case 2:
					data->pm4 = 0xFFFF;
					break;
				case 3:
					data->pm10 = 0xFFFF;
					break;
				case 4:
					data->humid = 0x7FFF;
					break;
				case 5:
					data->temp = 0x7FFF;
					break;
				case 6:
					data->voc = 0x7FFF;
					break;
				case 7:
					data->nox = 0x7FFF;
					break;
				case 8:
					data->co2 = 0xFFFF;
					break;
				default:
					break;
			};
		}
		else {
			switch(i){
				case 0: // PM1.0
					data->pm1 = (uint16_t)(combine_data(response[0], response[1])/10);
					break;
				case 1: // PM2.5
					data->pm2_5 = (uint16_t)(combine_data(response[0], response[1])/10);
					break;
				case 2: // PM4.0
					data->pm4 = (uint16_t)(combine_data(response[0], response[1])/10);
					break;
				case 3: // PM10.0
					data->pm10 = (uint16_t)(combine_data(response[0], response[1])/10);
					break;
				case 4: // Ambient Humidity
					data->humid = (int16_t)(combine_data(response[0], response[1])/100);
					break;
				case 5:	// Ambient Temperature
					data->temp = (int16_t)(combine_data(response[0], response[1])/200);
					break;
				case 6: // VOC index
					data->voc = (int16_t)(combine_data(response[0], response[1])/10);
					break;
				case 7: // NOx index
					data->nox = (int16_t)(combine_data(response[0], response[1])/10);
					break;
				case 8: //COx index -> In 5-6s it will be 0xFFFF
					data->co2 = (int16_t)(combine_data(response[0], response[1]));
					break;
				default:
					break;
			};
		}
	}
}
