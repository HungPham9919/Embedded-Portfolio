#ifndef HMC5883_H
#define HMC5883_H

#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include "stdio.h"
#include "stdint.h"
#include <sys/_stdint.h>
#include "RTOS/RTOS_Init.h"
#include "math.h"

#define HMC5883_ADDR 0x1E
#define HMC5883_REG_A 0x00
#define HMC5883_REG_B 0x01
#define HMC5883_MODE 0x02
#define HMC5883_DATA 0x03
#define HMC5883_STATUS 0x09

int HMC5883_Initialized(void);
int HMC5883_Calibration(void);
void Cal_The_Direction_Of_Yaw(uint8_t *data);

typedef struct {
    float mag_x_os;
    float mag_y_os;
    float mag_z_os;
} hmc_data_os;

typedef struct {
    float mag_x, mag_y,mag_z;
    float heading;
} hmc_data;


#endif
