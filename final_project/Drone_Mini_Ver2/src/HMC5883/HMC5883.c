#include "HMC5883.h"
#include "I2C_Progress/I2C.h"
#include "RTOS_Init.h"
#include "bmi088.h"
#include "zephyr/kernel.h"
#include <math.h>
#include <stdint.h>
#include <sys/_stdint.h>


hmc_data magnetic_data;
hmc_data_os hmc_offset;

int HMC5883_Initialized(void){
    // Mode: Normal : FRE 75 : 8 sample -> 01111000 = 0x78
    if(i2c_write_data(dev_i2c1,HMC5883_ADDR, HMC5883_REG_A, 0x78,1) != 0) goto ERR;
    if(i2c_write_data(dev_i2c1,HMC5883_ADDR, HMC5883_REG_B, 0x20,1) != 0) goto ERR; // Default gain
    if(i2c_write_data(dev_i2c1,HMC5883_ADDR, HMC5883_MODE, 0x00,1) != 0) goto ERR; // Continuous mode
    ERR:
        k_work_submit(&i2c3_error_work);
        printk("Failed to init HMC5883 \n");
    return 0;
}

int HMC5883_Calibration(void){
    uint8_t hmc_calib[6] = {0};

    int16_t x_max = 0, x_min = 0;
    int16_t z_max = 0, z_min = 0;
    int16_t y_max = 0, y_min = 0;
    int16_t mag_x_raw, mag_z_raw, mag_y_raw;

    for(int i = 0;i < 500; i++){
        if(i2c_dma_read_data(dev_i2c3,HMC5883_ADDR, HMC5883_DATA, hmc_calib, sizeof(hmc_calib), &dma1_stream2_signal) != 0) goto ERR;

        mag_x_raw = (int16_t)(hmc_calib[0] << 8 | hmc_calib[1]);
        mag_z_raw = (int16_t)(hmc_calib[2] << 8 | hmc_calib[3]);
        mag_y_raw = (int16_t)(hmc_calib[4] << 8 | hmc_calib[5]);

        if(mag_x_raw > x_max) x_max = mag_x_raw;
        if(mag_x_raw < x_min) x_min = mag_x_raw;

        if(mag_z_raw > z_max) z_max = mag_z_raw;
        if(mag_z_raw < z_min) z_min = mag_z_raw;

        if(mag_y_raw > y_max) y_max = mag_y_raw;
        if(mag_y_raw < y_min) y_min = mag_y_raw;
        k_msleep(10);
    }

    hmc_offset.mag_x_os = (float)((x_max + x_min)/2.0f);
    hmc_offset.mag_z_os = (float)((z_max + z_min)/2.0f);
    hmc_offset.mag_y_os = (float)((y_max + y_min)/2.0f);

    ERR:
        k_work_submit(&i2c1_error_work);
        printk("Failed to cablib HMC5883 \n");
    return 0;
}

void Cal_The_Direction_Of_Yaw(uint8_t *data){
    magnetic_data.mag_x = ((int16_t)(data[0] << 8 | data[1])) - hmc_offset.mag_x_os;
    magnetic_data.mag_z = ((int16_t)(data[2] << 8 | data[3])) - hmc_offset.mag_z_os;
    magnetic_data.mag_y = ((int16_t)(data[4] << 8 | data[5])) - hmc_offset.mag_y_os;

    magnetic_data.heading = atan2f(magnetic_data.mag_y, magnetic_data.mag_x) * (180.0f / M_PI);
    if(magnetic_data.heading < 0) magnetic_data.heading += 360.0f;
}
