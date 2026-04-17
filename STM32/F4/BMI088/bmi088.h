/*
 * bmi088.h
 *
 *  Created on: Apr 7, 2026
 *      Author: hung
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "bmi088.h"
#include "stdio.h"
#include  "stdint.h"
#include "Initialize.h"
#include "stm32f4xx.h"
#include "math.h"

// ACC has 2 power mode : normal and suspend 0x04 for normal: 0x00 for suspend (clear)

// Gyyro has 3 power mode: normal, suspend and deep-suspend
// To enter suspend mode -> write 0x80 in LPM1 register - it can be cleared by writting 0x00 or soft reset
// To enter deep-suspend mode -> write 0x20 in LPM1 register - it can be cleared by writting 0x00 or soft reset

#define ACC_CHIP_ID 0x00
#define ACC_ERROR_REG 0x02
#define ACC_STATUS 0x03
#define ACC_PWR_CFG 0x7C
#define ACC_PWR_CRTL 0x7D
#define ACC_Data 0x12
#define ACC_ADDR 0x18		// Note
#define ACC_CONFIG 0x40
#define ACC_RANGE 0x41
#define ACC_IO1_CFG 0x53
#define ACC_IO2_CFG 0x54
#define ACC_IO_MAP 0x58
#define ACC_SOFT_RST 0x7E

#define ACC_LSB_3G   10922.0f
#define ACC_LSB_6G   5461.0f
#define ACC_LSB_12G  2730.0f
#define ACC_LSB_24G  1365.0f

#define GYRO_ADDR 0x69		// Note
#define GYRO_CHIP_ID 0x0F
#define GYRO_Data 0x02
#define GYRO_BANDWIDTH 0x10
#define GYRO_RANGE 0x0F
#define GYRO_LPM1 0x11
#define GYRO_SOFT_RST 0x14
#define GYRO_INT_CTRL 0x15
#define GYRO_INT34_IO_CFG 0x16 // = (1 << 0) // bit3 clear
#define GYRO_INT34_IO_MAP 0x18 // 0x01

#define alpha 0.98f

struct bmi_parameters {
	uint8_t acc_pwr;
	uint8_t acc_pwr_cfg;
	uint8_t acc_config;
	uint8_t acc_range;
	uint8_t acc_int1;
	uint8_t acc_io_map;

	uint8_t gyro_pwr;
	uint8_t gyro_bandwidth;
	uint8_t gyro_range;
	uint8_t gyro_int3;
	uint8_t gyro_io_map_cfg;
	float acc_lsb,gyro_lsb;
};
extern struct bmi_parameters sens;

typedef struct bmi_offset {
	float gyro_offset_x;
	float gyro_offset_y;
	float gyro_offset_z;
}os;

extern os bmi088_offset;

struct Final_data {
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
};
extern struct Final_data final;

typedef struct {
	uint8_t acc_id,acc_err_code,acc_fatal_err;
	uint8_t acc_cfg, acc_range,acc_int1,acc_pwr_cfg, acc_pwr_ctrl;

	uint8_t gyro_id,gyro_range,gyro_bandwidth,gyro_lpm1,gyro_int_ctrl;
	uint8_t gyro_io_map,gyro_io_cfg;
} Status_Of_BMI088;
extern Status_Of_BMI088 status;

typedef struct {
	float Roll_angle,Pitch_angle,Yaw_angle;
}Drone_Angle;
extern Drone_Angle drone_angle;

void GYRO_ACC_LSB(uint8_t acc_range, uint8_t gyro_range);
void BMI088_Initialize(void);
void BMI088_Calib(void);
void Calculate_And_Filter_Angle(uint8_t *acc_data,uint8_t *gyro_data,float dt);
void BMI088_Data(uint8_t *acc_time_data, uint8_t *gyro_data);
void Read_Status(uint8_t slave_id ,uint8_t reg, uint8_t *data);
void Read_Data(uint8_t slave_id ,uint8_t reg, uint8_t *data, int len);

#endif /* INC_BMI088_H_ */
