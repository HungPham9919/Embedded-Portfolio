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
    if(i2c_write_data(dev_i2c3,HMC5883_ADDR, HMC5883_REG_A, 0x78,1) != 0) goto ERR;
    if(i2c_write_data(dev_i2c3,HMC5883_ADDR, HMC5883_REG_B, 0x20,1) != 0) goto ERR; // Default gain
    if(i2c_write_data(dev_i2c3,HMC5883_ADDR, HMC5883_MODE, 0x00,1) != 0) goto ERR; // Continuous mode
    return 0;
    
ERR:
    return -1;

}

int HMC5883_Calibration(void){
    uint8_t hmc_calib[6] = {0};

    // BẮT BỘC: Khởi tạo cực đại/cực tiểu đúng cực trị int16_t
    int16_t x_max = -32768, x_min = 32767;
    int16_t y_max = -32768, y_min = 32767;
    int16_t z_max = -32768, z_min = 32767;
    int16_t mag_x_raw, mag_z_raw, mag_y_raw;

    printk("Bat dau Calib HMC5883! Hãy xoay drone hinh so 8 hoac quay 360 deg...\n");
    k_msleep(2000); // Delay 2s de bat dau xoay

    // 3000 lan * 10ms = 30 giay thu thap va quet quy dao
    for(int i = 0; i < 3000; i++){
        if(i2c_dma_read_data(dev_i2c3, HMC5883_ADDR, HMC5883_DATA, hmc_calib, sizeof(hmc_calib), &dma1_stream2_signal) != 0) goto ERR;

        mag_x_raw = (int16_t)(((uint16_t)hmc_calib[0] << 8) | hmc_calib[1]);
        mag_z_raw = (int16_t)(((uint16_t)hmc_calib[2] << 8) | hmc_calib[3]);
        mag_y_raw = (int16_t)(((uint16_t)hmc_calib[4] << 8) | hmc_calib[5]);

        // Cập nhật Min/Max
        if(mag_x_raw > x_max) x_max = mag_x_raw;
        if(mag_x_raw < x_min) x_min = mag_x_raw;

        if(mag_y_raw > y_max) y_max = mag_y_raw;
        if(mag_y_raw < y_min) y_min = mag_y_raw;

        if(mag_z_raw > z_max) z_max = mag_z_raw;
        if(mag_z_raw < z_min) z_min = mag_z_raw;

        // In Data de theo doi tiến trình qua Terminal
        if(i % 10 == 0) { // Moi 100ms in 1 lan cho khoi nghen Terminal
            printk("MAG_RAW: X=%d [%d,%d] | Y=%d [%d,%d] | Z=%d [%d,%d]\n", 
                    mag_x_raw, x_min, x_max, 
                    mag_y_raw, y_min, y_max, 
                    mag_z_raw, z_min, z_max);
        }

        k_msleep(10);
    }

    // Tinh Offset
    hmc_offset.mag_x_os = (float)((x_max + x_min) / 2.0f);
    hmc_offset.mag_y_os = (float)((y_max + y_min) / 2.0f);
    hmc_offset.mag_z_os = 0.0f;

printk("-> KET QUA CALIB: OS_X=%d | OS_Y=%d | OS_Z=%d\n", 
            (int)hmc_offset.mag_x_os, 
            (int)hmc_offset.mag_y_os, 
            (int)hmc_offset.mag_z_os);

    return 0;

ERR:
    return -1;
}

#define MAG_X_OFFSET    (-38.0f)
#define MAG_Y_OFFSET    (97.5f)
#define MAG_Z_OFFSET    (-194.0f)

#define MAG_X_SCALE     (1.0495f)
#define MAG_Y_SCALE     (0.9550f)
#define MAG_Z_SCALE     (1.0000f)

void Cal_The_Direction_Of_Yaw(uint8_t *data){
    magnetic_data.mag_x = (int16_t)((uint16_t)data[0] << 8 | data[1]) - hmc_offset.mag_x_os;
    magnetic_data.mag_z = (int16_t)((uint16_t)data[2] << 8 | data[3]) - hmc_offset.mag_z_os;
    magnetic_data.mag_y = (int16_t)((uint16_t)data[4] << 8 | data[5]) - hmc_offset.mag_y_os;

    magnetic_data.heading = atan2f(magnetic_data.mag_y, magnetic_data.mag_x) * (180.0f / M_PI);
    if(magnetic_data.heading < 0) magnetic_data.heading += 360.0f;
}

// void Cal_The_Direction_Of_Yaw(const uint8_t *data) {
//     // 1. Ghép 2 byte thành giá trị raw (int16_t)
//     int16_t raw_x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
//     int16_t raw_z = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
//     int16_t raw_y = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

//     // 2. Bù Hard-Iron (Offset) và Soft-Iron (Scale)
//     float cal_x = ((float)raw_x - MAG_X_OFFSET) * MAG_X_SCALE;
//     float cal_y = ((float)raw_y - MAG_Y_OFFSET) * MAG_Y_SCALE;
//     float cal_z = ((float)raw_z - MAG_Z_OFFSET) * MAG_Z_SCALE;

//     magnetic_data.mag_x = cal_x;
//     magnetic_data.mag_y = cal_y;
//     magnetic_data.mag_z = cal_z;

//     // 3. Tính góc Heading/Yaw
//     magnetic_data.heading = atan2f(cal_y, cal_x) * (180.0f / (float)M_PI);
//     if (magnetic_data.heading < 0.0f) {
//         magnetic_data.heading += 360.0f;
//     }
// }
