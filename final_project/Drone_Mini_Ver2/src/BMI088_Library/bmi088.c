#include "bmi088.h"
#include "RTOS/RTOS_Init.h"
#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include "stdlib.h"
#include "I2C_Progress/I2C.h"
#include "zephyr/sys/printk.h"

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

int BMI088_Calib(void){
	float sum_gx = 0 , sum_gy = 0,sum_gz = 0;
	float sum_ax = 0, sum_ay = 0, sum_az = 0;

	int16_t gx = 0,gy = 0,gz = 0;
	int16_t ax = 0, ay = 0, az = 0;

	uint8_t gyro[6] = {0};
	uint8_t accel_data[7] = {0};
	for(int i = 0; i < 550; i++){
		
		if(i2c_dma_read_data(dev_i2c3,GYRO_ADDR, GYRO_Data, gyro, 6,&dma1_stream2_signal) != 0) goto ERR;
		if(i2c_dma_read_data(dev_i2c3,ACC_ADDR, ACC_Data, accel_data, 7, &dma1_stream2_signal) != 0) goto ERR;
		if(i > 49){
			gx = (gyro[1] << 8)|gyro[0];
			gy = (gyro[3] << 8)|gyro[2];
			gz = (gyro[5] << 8)|gyro[4];

			sum_gx += gx;
			sum_gy += gy;
			sum_gz += gz;

			ax = (accel_data[2] << 8) | accel_data[1];
			ay = (accel_data[4] << 8) | accel_data[3];
			az = (accel_data[6] << 8) | accel_data[5];

			sum_ax += ax;
			sum_ay += ay;
			sum_az += az;
		}
		k_msleep(5);
	}

	bmi088_offset.gyro_offset_x = (sum_gx/ 500);
	bmi088_offset.gyro_offset_y = (sum_gy/ 500);
	bmi088_offset.gyro_offset_z = (sum_gz/ 500);

	bmi088_offset.acc_offset_x = (sum_ax / 500.0f);
	bmi088_offset.acc_offset_y = (sum_ay / 500.0f);
	bmi088_offset.acc_offset_z = (sum_az / 500.0f) + ACC_LSB_6G;

	return 0;

ERR:
	k_work_submit(&i2c3_error_work);
	printk("Failed to Calib BMI088 \n");
	return -1;

}

volatile int16_t ACC_X = 0, ACC_Y = 0, ACC_Z = 0;
volatile int16_t GYRO_X = 0, GYRO_Y = 0, GYRO_Z = 0;

void BMI088_Data(uint8_t *acc_data, uint8_t *gyro_data){

	ACC_X = (int16_t)(acc_data[1] << 8) | acc_data[0];
	ACC_Y = (int16_t)(acc_data[3] << 8) | acc_data[2];
	ACC_Z = (int16_t)(acc_data[5] << 8) | acc_data[4];

	GYRO_X = (int16_t)((gyro_data[1] << 8)|gyro_data[0]);
	GYRO_Y = (int16_t)((gyro_data[3] << 8)|gyro_data[2]);
	GYRO_Z = (int16_t)((gyro_data[5] << 8)|gyro_data[4]);

	final.ax = (float)((ACC_X - bmi088_offset.acc_offset_x)/sens.acc_lsb);
	final.ay = -(float)((ACC_Y - bmi088_offset.acc_offset_y)/sens.acc_lsb);
	final.az = -(float)((ACC_Z - bmi088_offset.acc_offset_z)/sens.acc_lsb);

	// inverse

	final.gx = ((float)(GYRO_X - bmi088_offset.gyro_offset_x)/sens.gyro_lsb);
	final.gy = -((float)(GYRO_Y - bmi088_offset.gyro_offset_y)/sens.gyro_lsb);
	final.gz = -((float)((GYRO_Z - bmi088_offset.gyro_offset_z)/sens.gyro_lsb));
}

void Check_Status(void){
	if(i2c_dma_read_data(dev_i2c3,ACC_ADDR, ACC_CONFIG, &status.acc_cfg,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,ACC_ADDR, ACC_RANGE, &status.acc_range,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,ACC_ADDR,ACC_IO1_CFG,&status.acc_int1,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,ACC_ADDR, ACC_PWR_CFG, &status.acc_pwr_cfg,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,ACC_ADDR, ACC_PWR_CRTL, &status.acc_pwr_ctrl,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	// gyro
	if(i2c_dma_read_data(dev_i2c3,GYRO_ADDR, GYRO_RANGE, &status.gyro_range,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,GYRO_ADDR,GYRO_BANDWIDTH,&status.gyro_bandwidth,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,GYRO_ADDR, GYRO_LPM1, &status.gyro_lpm1,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,GYRO_ADDR, GYRO_INT_CTRL, &status.gyro_int_ctrl,1,&dma1_stream2_signal) != 0) goto ERR; // check
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,GYRO_ADDR, GYRO_INT34_IO_MAP, &status.gyro_io_map,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,GYRO_ADDR, GYRO_INT34_IO_CFG, &status.gyro_io_cfg,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);

ERR:
	k_work_submit(&i2c3_error_work);
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

int BMI088_Initialize(void){
	if(i2c_write_data(dev_i2c3, ACC_ADDR, ACC_SOFT_RST, 0xB6, 1,&dma1_stream4_signal) != 0) goto ERR;
	k_msleep(100);
	if(i2c_write_data(dev_i2c3,ACC_ADDR,ACC_PWR_CFG,sens.acc_pwr_cfg, 1,&dma1_stream4_signal) != 0) goto ERR; // Active 0x00
	k_msleep(20);
	if(i2c_write_data(dev_i2c3,ACC_ADDR,ACC_PWR_CRTL,sens.acc_pwr,1,&dma1_stream4_signal) != 0) goto ERR; // normal mode 0x04
	k_msleep(50);

	if(i2c_write_data(dev_i2c3,GYRO_ADDR, GYRO_SOFT_RST, 0xB6,1,&dma1_stream4_signal)!= 0) goto ERR; // GYRO_SOFTRESET
	k_msleep(100);
	if(i2c_write_data(dev_i2c3,GYRO_ADDR, GYRO_LPM1, sens.gyro_pwr,1,&dma1_stream4_signal) != 0) goto ERR; // normal mode
	k_msleep(50);

	// INT mode
	if(i2c_write_data(dev_i2c3,ACC_ADDR, ACC_IO_MAP, sens.acc_io_map,1,&dma1_stream4_signal) != 0) goto ERR; // 0x04 for 0x58
	k_msleep(2);
	if(i2c_write_data(dev_i2c3,ACC_ADDR,ACC_IO1_CFG, sens.acc_int1,1,&dma1_stream4_signal) != 0) goto ERR; // 0x0A for 0x53
	k_msleep(2);
	if(i2c_write_data(dev_i2c3,ACC_ADDR, ACC_IO2_CFG, 0x00,1,&dma1_stream4_signal) !=  0) goto ERR; // Off int 2
	k_msleep(2);

	if(i2c_write_data(dev_i2c3,GYRO_ADDR, GYRO_INT34_IO_CFG, sens.gyro_int3,1,&dma1_stream4_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_write_data(dev_i2c3,GYRO_ADDR, GYRO_INT_CTRL, 0x80,1,&dma1_stream4_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_write_data(dev_i2c3,GYRO_ADDR, GYRO_INT34_IO_MAP, sens.gyro_io_map_cfg,1,&dma1_stream4_signal) != 0) goto ERR;
	k_msleep(2);

	// Configuration
	if(i2c_write_data(dev_i2c3,ACC_ADDR, ACC_CONFIG,sens.acc_config,1,&dma1_stream4_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_write_data(dev_i2c3,ACC_ADDR,ACC_RANGE,sens.acc_range,1,&dma1_stream4_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_write_data(dev_i2c3,GYRO_ADDR, GYRO_BANDWIDTH, sens.gyro_bandwidth,1,&dma1_stream4_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_write_data(dev_i2c3,GYRO_ADDR, GYRO_RANGE, sens.gyro_range,1,&dma1_stream4_signal) != 0) goto ERR;
	k_msleep(2);

	// Check status

	if(i2c_dma_read_data(dev_i2c3,ACC_ADDR, ACC_CHIP_ID, &status.acc_id,1,&dma1_stream2_signal) != 0) goto ERR;
	k_msleep(2);
	if(i2c_dma_read_data(dev_i2c3,GYRO_ADDR, GYRO_CHIP_ID, &status.gyro_id, 1, &dma1_stream2_signal) != 0) goto ERR;
	k_msleep(20);

	Check_Status();
	return 0;
ERR:
	k_work_submit(&i2c3_error_work);
	printk("Failed to init BMI088 \n");
	return -1;

}
