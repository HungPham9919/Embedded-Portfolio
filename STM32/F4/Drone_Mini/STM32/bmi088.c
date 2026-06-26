/*
 * bmi088.c
 *
 *  Created on: Apr 7, 2026
 *      Author: hung
 */
#include "bmi088.h"
#include "Initialize.h"
#include "cmsis_os.h"
#include "main.h"
#include "PWM_Control.h"

struct Final_data final;
Status_Of_BMI088 status;
Drone_Angle drone_angle;
os bmi088_offset;

extern osThreadId_t I2C3_Broken_Task_Handle;
extern osThreadId_t Error_Thread;

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

int Write_data(uint8_t slave_id, uint8_t reg, uint8_t value){
	volatile uint32_t timeout = 10000;
	while ((I2C3->SR2 & (1 << 1)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	timeout = 10000;
	I2C3->CR1 |= (1 << 8); // START BIT

	while(!(I2C3->SR1 & (1 << 0)) && --timeout); // WAIT FOR SB
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	I2C3->DR = (slave_id << 1);
	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 1)) && --timeout){
		if(I2C3->SR1 & (1 << 10)){
			I2C3->CR1 |= (1 << 9); // stop
			I2C3->SR1 &= ~(1 << 10);
			return 0;
		}
	}
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	(void)I2C3->SR1;
	(void)I2C3->SR2;

	I2C3->DR = reg;

	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 7)) && --timeout){}; // TC
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}
	I2C3->DR = value;

	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 2)) && --timeout){}; // BTF
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	I2C3->CR1 |= (1 << 9); // stop
	while(I2C3->CR1 & (1 << 9)); // wait to done

	while(I2C3->SR2 & (1 << 1));
	return 1;
}

void BMI088_Write_With_Retry(uint8_t slave_id, uint8_t reg, uint8_t value) {
    while(1) {
        if (Write_data(slave_id, reg, value) == 1) {
            break;
        }
        else {
            I2C3_Broken_Task_Handle = osThreadGetId();
            osThreadFlagsSet(Error_Thread, REQ_RESET_I2C3);

            osThreadFlagsWait(ERROR_FIXED, osFlagsWaitAny, osWaitForever);
            osDelay(5);
        }
    }
}

int Read_Data(uint8_t slave_id ,uint8_t reg, uint8_t *data, int len){
	volatile uint32_t timeout = 10000;
	while((I2C3->SR2 & (1 << 1)) && --timeout);

	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	I2C3->CR1 |= (1 << 8);
	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 0)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	I2C3->DR  = (slave_id << 1);
	timeout = 10000;
    while(!(I2C3->SR1 & (1 << 1)) && --timeout){
		if(I2C3->SR1 & (1 << 10)){
			I2C3->CR1 |= (1 << 9);
			I2C3->SR1 &= ~(1 << 10);
			return 0;
		}
    }; // ADDR

	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	(void)I2C3->SR1;
	(void)I2C3->SR2;

	I2C3->DR = reg;
	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 7)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 2)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	//read function
	timeout = 10000;
	I2C3->CR1 |= (1 << 8);
	while(!(I2C3->SR1 & (1 << 0)) && --timeout);

	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	I2C3->DR = (slave_id << 1) | 1; // read mode
	while(!(I2C3->SR1 & ((1 << 1)|(1 << 10))) && --timeout);

	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	if(I2C3->SR1 & (1 << 10)){
	    I2C3->CR1 |= (1 << 9);
	    I2C3->SR1 &= ~(1 << 10);
	    return 0;
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

		timeout = 10000;
	    while(!(I2C3->SR1 & (1 << 6)) && --timeout){};
		if(timeout == 0) {
			I2C3->CR1 |= (1 << 9);
			I2C3->CR1 &= ~(1 << 0); // off PE
			return 0;
		}
	    data[i] = I2C3->DR;
	}
	timeout = 10000;
	while((I2C3->SR2 & (1 << 1)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	return 1;
}

void BMI088_Read_Data_With_Retry(uint8_t slave_id, uint8_t reg, uint8_t *data, int len) {
    while(1) {
        if (Read_Data(slave_id, reg, data, len) == 1) {
            break;
        }
        else {
            I2C3_Broken_Task_Handle = osThreadGetId();
            osThreadFlagsSet(Error_Thread, REQ_RESET_I2C3);

            osThreadFlagsWait(ERROR_FIXED, osFlagsWaitAny, osWaitForever);
            osDelay(3);
        }
    }
}

int Read_Status(uint8_t slave_id ,uint8_t reg, uint8_t *data){
	volatile uint32_t timeout = 10000;
	while ((I2C3->SR2 & (1 << 1)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	I2C3->CR1 |= (1 << 8);
	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 0)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	I2C3->DR  = (slave_id << 1);
	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 1)) && --timeout){
		if(I2C3->SR1 & (1 << 10)){
			I2C3->CR1 |= (1 << 9);
			I2C3->SR1 &= ~(1 << 10);
			(void)I2C3->SR1;
			while (I2C3->SR2 & (1 << 1));
			return 0;
		}
	};

	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	(void)I2C3->SR1;
	(void)I2C3->SR2;

	I2C3->DR = reg;
	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 7)) && --timeout); 			// TXE
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 2)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

																	//read function
	I2C3->CR1 |= (1 << 8);
	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 0)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}
	I2C3->DR = (slave_id << 1) | 1; // read mode

	timeout = 10000;
	while(!(I2C3->SR1 & ((1 << 1) | (1 << 10))) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	if(I2C3->SR1 & (1 << 10)){ // Nếu dính NACK ở pha Đọc
	   I2C3->CR1 |= (1 << 9);   // Phát STOP trước
	   I2C3->SR1 &= ~(1 << 10); // Xóa AF sau
	   I2C3->CR1 &= ~(1 << 0); // add
	   return 0;
	 }

	I2C3->CR1 &= ~(1 << 10);

	(void)I2C3->SR1;
	(void)I2C3->SR2;

	I2C3->CR1 |= (1 << 9);

	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 6)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}
	*data = I2C3->DR;

	timeout = 10000;
	while((I2C3->CR1 & (1 << 9)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	timeout = 10000;
	while ((I2C3->SR2 & (1 << 1)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	return 1;
}

void BMI088_Read_Status_With_Retry(uint8_t slave_id, uint8_t reg, uint8_t *data) {
    while(1) {
        if (Read_Status(slave_id, reg, data) == 1) {
            break;
        }
        else {
            I2C3_Broken_Task_Handle = osThreadGetId();
            osThreadFlagsSet(Error_Thread, REQ_RESET_I2C3);

            osThreadFlagsWait(ERROR_FIXED, osFlagsWaitAny, osWaitForever);
            osDelay(5);
        }
    }
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
	BMI088_Write_With_Retry(ACC_ADDR, ACC_CONFIG,sens.acc_config);
	osDelay(2);
	BMI088_Write_With_Retry(ACC_ADDR,ACC_RANGE,sens.acc_range);
	osDelay(2);
	BMI088_Write_With_Retry(GYRO_ADDR, GYRO_BANDWIDTH, sens.gyro_bandwidth);
	osDelay(2);
	BMI088_Write_With_Retry(GYRO_ADDR, GYRO_RANGE, sens.gyro_range);
	osDelay(2);
}

void BMI088_Calib(void){
	float sum_gx = 0 , sum_gy = 0,sum_gz = 0;
	float sum_ax = 0, sum_ay = 0, sum_az = 0;

	int16_t gx = 0,gy = 0,gz = 0;
	int16_t ax = 0, ay = 0, az = 0;

	uint8_t gyro[6] = {0};
	uint8_t accel_data[6] = {0};
	for(int i = 0; i < 550; i++){
		BMI088_Read_Data_With_Retry(GYRO_ADDR, GYRO_Data, gyro, sizeof(gyro));
		BMI088_Read_Data_With_Retry(ACC_ADDR, ACC_Data, accel_data, sizeof(accel_data));
		if(i > 49){
			gx = (gyro[1] << 8)|gyro[0];
			gy = (gyro[3] << 8)|gyro[2];
			gz = (gyro[5] << 8)|gyro[4];

			sum_gx += gx;
			sum_gy += gy;
			sum_gz += gz;

			ax = (accel_data[1] << 8) | accel_data[0];
			ay = (accel_data[3] << 8) | accel_data[2];
			az = (accel_data[5] << 8) | accel_data[4];

			sum_ax += ax;
			sum_ay += ay;
			sum_az += az;
		}
		osDelay(5);
	}

	bmi088_offset.gyro_offset_x = (sum_gx/ 500);
	bmi088_offset.gyro_offset_y = (sum_gy/ 500);
	bmi088_offset.gyro_offset_z = (sum_gz/ 500);

	bmi088_offset.acc_offset_x = (sum_ax / 500.0f);
	bmi088_offset.acc_offset_y = (sum_ay / 500.0f);
	bmi088_offset.acc_offset_z = (sum_az / 500.0f) + ACC_LSB_6G;

//	roll_pid_rate.previous_measure  = 0.0f;
//	pitch_pid_rate.previous_measure = 0.0f;
//	yaw_pid_rate.previous_measure   = 0.0f;

//	roll_pid.previous_measure  = drone_angle.Roll_angle;
//	pitch_pid.previous_measure = drone_angle.Pitch_angle;
//	yaw_pid.previous_measure   = drone_angle.Yaw_angle;

}

volatile int16_t ACC_X = 0, ACC_Y = 0, ACC_Z = 0;
volatile int16_t GYRO_X = 0, GYRO_Y = 0, GYRO_Z = 0;

void BMI088_Data(uint8_t *acc_data, uint8_t *gyro_data){

	ACC_X = (int16_t)(acc_data[1] << 8) | acc_data[0];
	ACC_Y = (int16_t)(acc_data[3] << 8) | acc_data[2];
	ACC_Z = (int16_t)(acc_data[5] << 8) | acc_data[4];

	GYRO_X = (int16_t)(gyro_data[1] << 8)|gyro_data[0];
	GYRO_Y = (int16_t)(gyro_data[3] << 8)|gyro_data[2];
	GYRO_Z = (int16_t)((gyro_data[5] << 8)|gyro_data[4]);

	final.ax = (float)((ACC_X - bmi088_offset.acc_offset_x)/sens.acc_lsb);
	final.ay = -(float)((ACC_Y - bmi088_offset.acc_offset_y)/sens.acc_lsb);
	final.az = -(float)((ACC_Z - bmi088_offset.acc_offset_z)/sens.acc_lsb);

	// inverse

	final.gx = ((float)(GYRO_X - bmi088_offset.gyro_offset_x)/sens.gyro_lsb);
	final.gy = -((float)(GYRO_Y - bmi088_offset.gyro_offset_y)/sens.gyro_lsb);
	final.gz = -((float)((GYRO_Z - bmi088_offset.gyro_offset_z)/sens.gyro_lsb));

	if(fabs(final.gz) < 0.05f) final.gz = 0;
}

void Calculate_And_Filter_Angle(uint8_t *acc_data,uint8_t *gyro_data,float dt){
	BMI088_Data(acc_data, gyro_data);
	float acc_roll = atan2f(final.ay,final.az)*(180/M_PI); // inverse
	float acc_pitch = atan2f(-final.ax,sqrtf(final.ax*final.ax + final.az*final.az))*(180/M_PI);

	// 0.02 acc + 0.98 gyro
	drone_angle.Roll_angle = (alpha*(drone_angle.Roll_angle + final.gx*dt) + acc_roll*(1- alpha)) ;
	drone_angle.Pitch_angle = alpha*(drone_angle.Pitch_angle + final.gy*dt) + acc_pitch*(1- alpha);
	drone_angle.Yaw_angle += final.gz * dt;

	if(drone_angle.Yaw_angle > 180) drone_angle.Yaw_angle -= 360;
	if(drone_angle.Yaw_angle < -180) drone_angle.Yaw_angle += 360;
}

void Check_Status(void){
	BMI088_Read_Status_With_Retry(ACC_ADDR, ACC_CONFIG, &status.acc_cfg);
	osDelay(2);
	BMI088_Read_Status_With_Retry(ACC_ADDR, ACC_RANGE, &status.acc_range);
	osDelay(2);
	BMI088_Read_Status_With_Retry(ACC_ADDR,ACC_IO1_CFG,&status.acc_int1);
	osDelay(2);
	BMI088_Read_Status_With_Retry(ACC_ADDR, ACC_PWR_CFG, &status.acc_pwr_cfg);
	osDelay(2);
	BMI088_Read_Status_With_Retry(ACC_ADDR, ACC_PWR_CRTL, &status.acc_pwr_ctrl);
	osDelay(2);
	// gyro
	BMI088_Read_Status_With_Retry(GYRO_ADDR, GYRO_RANGE, &status.gyro_range);
	osDelay(2);
	BMI088_Read_Status_With_Retry(GYRO_ADDR,GYRO_BANDWIDTH,&status.gyro_bandwidth);
	osDelay(2);
	BMI088_Read_Status_With_Retry(GYRO_ADDR, GYRO_LPM1, &status.gyro_lpm1);
	osDelay(2);
	BMI088_Read_Status_With_Retry(GYRO_ADDR, GYRO_INT_CTRL, &status.gyro_int_ctrl); // check
	osDelay(2);
	BMI088_Read_Status_With_Retry(GYRO_ADDR, GYRO_INT34_IO_MAP, &status.gyro_io_map);
	osDelay(2);
	BMI088_Read_Status_With_Retry(GYRO_ADDR, GYRO_INT34_IO_CFG, &status.gyro_io_cfg);
	osDelay(2);
}

void BMI088_Initialize(void){
	BMI088_Write_With_Retry(ACC_ADDR, ACC_SOFT_RST, 0xB6);
	osDelay(50);
	BMI088_Write_With_Retry(ACC_ADDR,ACC_PWR_CFG,sens.acc_pwr_cfg); // Active 0x00
	osDelay(20);
	BMI088_Write_With_Retry(ACC_ADDR,ACC_PWR_CRTL,sens.acc_pwr); // normal mode 0x04
	osDelay(50);

	BMI088_Write_With_Retry(GYRO_ADDR, GYRO_SOFT_RST, 0xB6); // GYRO_SOFTRESET
	osDelay(50);
	BMI088_Write_With_Retry(GYRO_ADDR, GYRO_LPM1, sens.gyro_pwr); // normal mode
	osDelay(50);

	// Check status
	BMI088_Read_Status_With_Retry(ACC_ADDR, ACC_CHIP_ID, &status.acc_id);
	osDelay(2);
	BMI088_Read_Status_With_Retry(GYRO_ADDR, GYRO_CHIP_ID, &status.gyro_id);

	osDelay(20);

	// INT mode
	BMI088_Write_With_Retry(ACC_ADDR, ACC_IO_MAP, sens.acc_io_map); // 0x04 for 0x58
	osDelay(2);
	BMI088_Write_With_Retry(ACC_ADDR,ACC_IO1_CFG, sens.acc_int1); // 0x0A for 0x53
	osDelay(2);
	BMI088_Write_With_Retry(ACC_ADDR, ACC_IO2_CFG, 0x00); // Off int 2
	osDelay(2);

	BMI088_Write_With_Retry(GYRO_ADDR, GYRO_INT34_IO_CFG, sens.gyro_int3);
	osDelay(2);
	BMI088_Write_With_Retry(GYRO_ADDR, GYRO_INT_CTRL, 0x80);
	osDelay(2);
	BMI088_Write_With_Retry(GYRO_ADDR, GYRO_INT34_IO_MAP, sens.gyro_io_map_cfg);
	osDelay(2);

	Configuration_Of_BMI088();
//	 Check_Status();
}

// DMA buffer
volatile uint32_t dma_ndtr = 0;
int Read_Data_DMA(uint8_t slave_id ,uint8_t reg, uint8_t *data_dma, int len){
	volatile uint32_t timeout = 10000;
	while((I2C3->SR2 & (1 << 1)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	timeout = 10000;
	I2C3->CR1 |= (1 << 8);
	while(!(I2C3->SR1 & (1 << 0)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	I2C3->DR  = (slave_id << 1);
    while(!(I2C3->SR1 & (1 << 1)) && --timeout){ 	// ADDR - Thường crash ở đây
		if(I2C3->SR1 & (1 << 10)){
			I2C3->CR1 |= (1 << 9);
			I2C3->SR1 &= ~(1 << 10);
			return 0;
		}
    };

	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	(void)I2C3->SR1;
	(void)I2C3->SR2;

	I2C3->DR = reg;
	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 7)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	timeout = 10000;
	while(!(I2C3->SR1 & (1 << 2)) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

															//read function
	timeout = 10000;
	I2C3->CR1 |= (1 << 8);
	while(!(I2C3->SR1 & (1 << 0)) && --timeout);

	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	I2C3->DR = (slave_id << 1) | 1; // read mode
	timeout = 10000;
	while(!(I2C3->SR1 & ((1 << 1)|(1 << 10))) && --timeout);
	if(timeout == 0) {
		I2C3->CR1 |= (1 << 9);
		I2C3->CR1 &= ~(1 << 0); // off PE
		return 0;
	}

	if(I2C3->SR1 & (1 << 10)){
	    I2C3->CR1 |= (1 << 9);
	    I2C3->SR1 &= ~(1 << 10);
	    return 0;
	}
    // DMA
	DMA1_Stream2->CR &= ~(1 << 0); // off to config
	while(DMA1_Stream2->CR & (1 << 0));

	DMA1->LIFCR = (0x3D << 16);
	DMA1_Stream2->FCR = 0;

	DMA1_Stream2->NDTR = 0;
	DMA1_Stream2->PAR  = (uint32_t)&I2C3->DR;
	DMA1_Stream2->M0AR = (uint32_t)data_dma;

	DMA1_Stream2->NDTR = len;
	DMA1_Stream2->CR =
	      (3 << 25) |   // CHSEL=3
	      (3 << 16) |   // priority high
	      (1 << 10) |   // MINC
	      (1 << 4)	|	// TCIE
		  (1 << 0);     //Enable

	I2C3->CR2 |= (1 << 11)|(1 << 12); // DMA I2C enable and last byte
	dma_ndtr = DMA1_Stream2->NDTR;
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
	// CPU sleep
	osThreadFlagsWait(0x0002, osFlagsWaitAny, osWaitForever); // CPU ngủ để đợi xong

	if (len > 1) {
	   I2C3->CR1 |= (1 << 9);
	}
	I2C3->CR2 &= ~(1 << 11) &~(1 << 12);
	return 1;
}

void BMI088_Read_Data_DMA_With_Retry(uint8_t slave_id, uint8_t reg, uint8_t *data, int len) {
    while(1) {
        if (Read_Data_DMA(slave_id, reg, data, len) == 1) {
            break;
        }
        else {
            I2C3_Broken_Task_Handle = osThreadGetId();
            osThreadFlagsSet(Error_Thread, REQ_RESET_I2C3);

            osThreadFlagsWait(ERROR_FIXED, osFlagsWaitAny, osWaitForever);
            osDelay(5);
        }
    }
}

