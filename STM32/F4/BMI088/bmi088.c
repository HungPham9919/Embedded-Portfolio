/*
 * bmi088.c
 *
 *  Created on: Apr 7, 2026
 *      Author: hung
 */
#include "bmi088.h"
struct Final_data final;
Status_Of_BMI088 status;
Drone_Angle drone_angle;
os bmi088_offset;

struct bmi_parameters sens = {
	.acc_pwr = 0x04,
	.acc_pwr_cfg = 0x00,
	.acc_config = (uint8_t)((0x08 << 4) | 0x09), // OSR4 | 200  Hz
	.acc_range = 0x01, // 6g
	.acc_int1 = 0x0A, // push pull and active high -> 0x53
	.acc_io_map = 0x04, // 0x58

	.gyro_pwr = 0x00,
	.gyro_range = 0x01, // 1000 degree
	.gyro_bandwidth = 0x04, // 200Hz
	.gyro_io_map_cfg = 0x01,
	.gyro_int3 = 0x01,
};

void Write_data(uint8_t slave_id, uint8_t reg, uint8_t value){
	I2C3->CR1 |= (1 << 8); // START BIT
	while(!(I2C3->SR1 & (1 << 0))); // WAIT FOR SB

	I2C3->DR = (slave_id << 1);
	while(!(I2C3->SR1 & (1 << 1))){
		if(I2C3->SR1 & (1 << 10)){
			I2C3->SR1 &= ~(1 << 10);
			I2C3->CR1 |= (1 << 9); // stop
			return;
		}
	}
	(void)I2C3->SR1;
	(void)I2C3->SR2;

	I2C3->DR = reg;
	while(!(I2C3->SR1 & (1 << 7))); // TC
	I2C3->DR = value;
	while(!(I2C3->SR1 & (1 << 2))); // BTF

	I2C3->CR1 |= (1 << 9); // stop
}

void Read_Data(uint8_t slave_id ,uint8_t reg, uint8_t *data, int len){
	while(I2C3->SR2 & (1 << 1));
	I2C3->CR1 |= (1 << 8);
	while(!(I2C3->SR1 & (1 << 0)));

	I2C3->DR  = (slave_id << 1);
    while(!(I2C3->SR1 & (1 << 1))){
		if(I2C3->SR1 & (1 << 10)){
			I2C3->SR1 &= ~(1 << 10);
			I2C3->CR1 |= (1 << 9);
			return;
		}
    }; // ADDR

	(void)I2C3->SR1;
	(void)I2C3->SR2;

	I2C3->DR = reg;
	while(!(I2C3->SR1 & (1 << 7)));

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

    if (len == 1)
    {
        I2C3->CR1 &= ~(1 << 10); // ACK = 0
        (void)I2C3->SR1;
        (void)I2C3->SR2;
        I2C3->CR1 |= (1 << 9); // STOP
    }
    else
    {
        I2C3->CR1 |= (1 << 10); // ACK = 1
        (void)I2C3->SR1;
        (void)I2C3->SR2;
    }

	for(int i = 0; i < len; i++){
		if(i == len - 1 && len > 1){
			I2C3->CR1 &= ~(1 << 10);
			I2C3->CR1 |= (1 << 9);
		}

	    while (!(I2C3->SR1 & (1 << 6)));
	    data[i] = I2C3->DR;
	}
}

void Read_Status(uint8_t slave_id ,uint8_t reg, uint8_t *data){
	I2C3->CR1 |= (1 << 8);
	while(!(I2C3->SR1 & (1 << 0)));

	I2C3->DR  = (slave_id << 1);
	while(!(I2C3->SR1 & (1 << 1)));
	if(I2C3->SR1 & (1 << 10)){
		I2C3->SR1 &= ~(1 << 10);
		I2C3->CR1 |= (1 << 9);
		return;
	}

	(void)I2C3->SR1;
	(void)I2C3->SR2;

	I2C3->DR = reg;
	while(!(I2C3->SR1 & (1 << 7)));

	//read function
	I2C3->CR1 |= (1 << 8);
	while(!(I2C3->SR1 & (1 << 0)));
	I2C3->DR = (slave_id << 1) | 1; // read mode
	while(!(I2C3->SR1 & (1 << 1)));

	I2C3->CR1 &= ~(1 << 10);
	(void)I2C3->SR1;
	(void)I2C3->SR2;
	I2C3->CR1 |= (1 << 9);

	while(!(I2C3->SR1 & (1 << 6)));
	*data = I2C3->DR;
}

void GYRO_ACC_LSB(uint8_t acc_range, uint8_t gyro_range){
	switch (acc_range){
		case 0x00:
			sens.acc_lsb = ACC_LSB_3G;
			break;
		case 0x01:
			sens.acc_lsb = ACC_LSB_6G;
			break;
		case 0x02:
			sens.acc_lsb = ACC_LSB_12G;
			break;
		default:
			sens.acc_lsb = ACC_LSB_24G;
			break;
	}

	switch (gyro_range){
		case 0x00:
			sens.gyro_lsb = 16.384f;
			break;
		case 0x01:
			sens.gyro_lsb = 32.768f;
			break;
		case 0x02:
			sens.gyro_lsb = 65.536f;
			break;
		case 0x03:
			sens.gyro_lsb = 131.072f;
			break;
		case 0x04:
			sens.gyro_lsb = 262.144f;
			break;
	}
}

void Configuration_Of_BMI088(void){
	Write_data(ACC_ADDR, ACC_CONFIG,sens.acc_config);
	Write_data(ACC_ADDR,ACC_RANGE,sens.acc_range);
	Write_data(GYRO_ADDR, GYRO_BANDWIDTH, sens.gyro_bandwidth);
	Write_data(GYRO_ADDR, GYRO_RANGE, sens.gyro_range);
}

void BMI088_Calib(void){
	float sum_gx = 0 , sum_gy = 0,sum_gz = 0;
	int16_t gx = 0,gy = 0,gz = 0;
	uint8_t gyro[6] = {0};
	for(int i = 0; i < 500; i++){
		Read_Data(GYRO_ADDR, GYRO_Data,gyro, sizeof(gyro));
		gx = (gyro[1] << 8)|gyro[0];
		gy = (gyro[3] << 8)|gyro[2];
		gz = (gyro[5] << 8)|gyro[4];

		sum_gx += gx;
		sum_gy += gy;
		sum_gz += gz;

		delay_ms(5);
	}

	bmi088_offset.gyro_offset_x = (sum_gx/ 500);
	bmi088_offset.gyro_offset_y = (sum_gy/ 500);
	bmi088_offset.gyro_offset_z = (sum_gz/ 500);
}

volatile int16_t ACC_X = 0, ACC_Y = 0, ACC_Z = 0;
volatile int16_t GYRO_X = 0, GYRO_Y = 0, GYRO_Z = 0;

void BMI088_Data(uint8_t *acc_data, uint8_t *gyro_data){

	ACC_X = (acc_data[1] << 8) | acc_data[0];
	ACC_Y = (acc_data[3] << 8) | acc_data[2];
	ACC_Z = (acc_data[5] << 8) | acc_data[4];

	GYRO_X = (gyro_data[1] << 8)|gyro_data[0];
	GYRO_Y = (gyro_data[3] << 8)|gyro_data[2];
	GYRO_Z = (int16_t)((gyro_data[5] << 8)|gyro_data[4]);

	final.ax = ACC_X/sens.acc_lsb;
	final.ay = ACC_Y/sens.acc_lsb;
	final.az = ACC_Z/sens.acc_lsb;

	final.gx = (float)(GYRO_X - bmi088_offset.gyro_offset_x)/sens.gyro_lsb;
	final.gy = (float)(GYRO_Y - bmi088_offset.gyro_offset_y)/sens.gyro_lsb;
	final.gz = (float)((GYRO_Z - bmi088_offset.gyro_offset_z)/sens.gyro_lsb);

}

void Calculate_And_Filter_Angle(uint8_t *acc_data,uint8_t *gyro_data,float dt){
	BMI088_Data(acc_data, gyro_data);
	float acc_roll = atan2f(final.ay,final.az)*(180/M_PI);
	float acc_pitch = atan2f(-final.ax,sqrt(final.ax*final.ax + final.az*final.az))*(180/M_PI);

	// 0.02 acc + 0.98 gyro
	drone_angle.Roll_angle = alpha*(drone_angle.Roll_angle + final.gx*dt) + acc_roll*(1- alpha);
	drone_angle.Pitch_angle = alpha*(drone_angle.Pitch_angle + final.gy*dt) + acc_pitch*(1- alpha);
	drone_angle.Yaw_angle += final.gz * dt;

	if(drone_angle.Yaw_angle > 180) drone_angle.Yaw_angle -= 360;
	if(drone_angle.Yaw_angle < -180) drone_angle.Yaw_angle += 360;
}

void Check_Status(void){
	Read_Status(ACC_ADDR, ACC_CONFIG, &status.acc_cfg);
	Read_Status(ACC_ADDR, ACC_RANGE, &status.acc_range);
	Read_Status(ACC_ADDR,ACC_IO1_CFG,&status.acc_int1);
	Read_Status(ACC_ADDR, ACC_PWR_CFG, &status.acc_pwr_cfg);
	Read_Status(ACC_ADDR, ACC_PWR_CRTL, &status.acc_pwr_ctrl);
	// gyro
	Read_Status(GYRO_ADDR, GYRO_RANGE, &status.gyro_range);
	Read_Status(GYRO_ADDR,GYRO_BANDWIDTH,&status.gyro_bandwidth);
	Read_Status(GYRO_ADDR, GYRO_LPM1, &status.gyro_lpm1);
	Read_Status(GYRO_ADDR, GYRO_INT_CTRL, &status.gyro_int_ctrl); // check
	Read_Status(GYRO_ADDR, GYRO_INT34_IO_MAP, &status.gyro_io_map);
	Read_Status(GYRO_ADDR, GYRO_INT34_IO_CFG, &status.gyro_io_cfg);
}

void BMI088_Initialize(void){
	Write_data(ACC_ADDR, ACC_SOFT_RST, 0xB6);
	delay_ms(50);
	Write_data(ACC_ADDR,ACC_PWR_CFG,sens.acc_pwr_cfg); // Active 0x00
	delay_ms(10);
	Write_data(ACC_ADDR,ACC_PWR_CRTL,sens.acc_pwr); // normal mode 0x04
	delay_ms(50);

	Write_data(GYRO_ADDR, GYRO_SOFT_RST, 0xB6); // GYRO_SOFTRESET
	delay_ms(50);
	Write_data(GYRO_ADDR, GYRO_LPM1, sens.gyro_pwr); // normal mode
	delay_ms(50);

	// Check status
	// Read_Status(ACC_ADDR, ACC_CHIP_ID, &status.acc_id);
	// Read_Status(GYRO_ADDR, GYRO_CHIP_ID, &status.gyro_id);

	// INT mode
	Write_data(ACC_ADDR, ACC_IO_MAP, sens.acc_io_map); // 0x04 for 0x58
	Write_data(ACC_ADDR,ACC_IO1_CFG, sens.acc_int1); // 0x0A for 0x53
	Write_data(ACC_ADDR, ACC_IO2_CFG, 0x00); // Off int 2

	Write_data(GYRO_ADDR, GYRO_INT34_IO_CFG, sens.gyro_int3);
	Write_data(GYRO_ADDR, GYRO_INT_CTRL, 0x80);
	Write_data(GYRO_ADDR, GYRO_INT34_IO_MAP, sens.gyro_io_map_cfg);
	Configuration_Of_BMI088();

	// Check status of register
	// Check_Status();
}

