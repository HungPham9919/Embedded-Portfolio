#ifndef BMI160_H
#define BMI160_H

#include "stdio.h"
#include "stdint.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "math.h"
#include "esp_timer.h"

#define BMI160_ADDR 0x69
extern i2c_master_dev_handle_t bmi160_handle;
extern TaskHandle_t bmi160_task;
#define GYRO_RANGE 0x43
#define GYRO_CONF 0x42

#define ACC_RANGE 0x41
#define ACC_CONF 0x40

#define CHIP_ID 0x00
#define ERR_REG 0x02
#define PMU_STATUS 0x03

#define BMI_DATA_ADDR 0x0C // 6byte LSB -> MSB
#define TIME_DATA 0x18

#define BMI_CMD 0x7E

typedef struct {
    int16_t ACC_X;
    int16_t ACC_Y;
    int16_t ACC_Z;

    int16_t GYRO_X;
    int16_t GYRO_Y;
    int16_t GYRO_Z;
}Raw_data;

struct BMI160_sensitivity
{
    uint8_t acc_cfg;
    uint8_t acc_range;
    uint8_t gyro_cfg;
    uint8_t gyro_range;
    
};

typedef struct {
    float ax,ay,az;
    float gx,gy,gz;
}final_data;

esp_err_t BMI160_Init(void);
esp_err_t BMI160_Setup(void);
void sensitivity(void);
void vBMI160_Sensor_Task(void *pvParameter);
#endif
