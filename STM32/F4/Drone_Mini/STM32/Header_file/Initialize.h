/*
 * Initialize.h
 *
 *  Created on: Apr 5, 2026
 *      Author: hung
 */

#ifndef INC_INITIALIZE_H_
#define INC_INITIALIZE_H_

typedef struct {
	uint8_t sensor1,sensor2,sensor3,sensor4;
}Sensor_address;

extern volatile int adr;
extern int Drone_State;
void Init_The_Config_Of_Drone(void);

void I2C3_Initialized(void);
void I2C3_ClearBus(void);
void I2C3_RST_APB(void);
int Check_Address_I2C3(void);

void I2C1_Initialized(void);
void I2C1_ClearBus(void);
void I2C1_RST_APB(void);
int Check_Address_I2C1(void);

void delay_ms(uint32_t time_Delay);
void BUS_Init(void);
void DMA_I2C3_Stream(void);

void The_First_State(void);
void Stop_Motor(void);
int pwm(int duty);

#endif /* INC_INITIALIZE_H_ */
