#include "BMP280.h"
#include "I2C_Progress/I2C.h"
#include "RTOS/RTOS_Init.h"
#include <sys/_stdint.h>
#include "math.h"
#include "zephyr/kernel.h"

bmp280_calib_data calib;
volatile uint8_t BMP_ID = 0;

int BMP280_Initialized(void){
    uint8_t bmpid = 0;
	if(i2c_write_data(dev_i2c1,BMP280_ADDR, BMP280_MEASURE, 0x27,1) != 0) goto ERR;
	k_msleep(5);
	if(i2c_write_data(dev_i2c1,BMP280_ADDR, BMP280_CONFIG, 0x00,1) != 0)goto ERR;
	k_msleep(5);
	if(i2c_dma_read_data(dev_i2c1,BMP280_ADDR, BMP280_ID, &bmpid, 1, &dma1_stream5_signal) != 0) goto ERR; // Read the ID: 58
    BMP_ID = bmpid;

	ERR:
		k_work_submit(&i2c1_error_work);
		printk("Failed to init BMP280 \n");
	return 0;
}
// Stream 5 for i2c1

int BMP280_Calibration(void){
	static uint8_t buffer[24];
	if(i2c_dma_read_data(dev_i2c1,BMP280_ADDR, BMP280_Calib,buffer, 24, &dma1_stream5_signal) != 0) goto ERR;
    
    calib.dig_T1 = (int16_t)(buffer[0] | (buffer[1] << 8));
	calib.dig_T2 = (int16_t)(buffer[2] | (buffer[3] << 8));
	calib.dig_T3 = (int16_t)(buffer[4] | (buffer[5] << 8));

	calib.dig_P1 = (int16_t)(buffer[6] | (buffer[7] << 8));
	calib.dig_P2 = (int16_t)(buffer[8] | (buffer[9] << 8));
	calib.dig_P3 = (int16_t)(buffer[10] | (buffer[11] << 8));
	calib.dig_P4 = (int16_t)(buffer[12] | (buffer[13] << 8));
	calib.dig_P5 = (int16_t)(buffer[14] | (buffer[15] << 8));
	calib.dig_P6 = (int16_t)(buffer[16] | (buffer[17] << 8));
	calib.dig_P7 = (int16_t)(buffer[18] | (buffer[19] << 8));
	calib.dig_P8 = (int16_t)(buffer[20] | (buffer[21] << 8));
	calib.dig_P9 = (int16_t)(buffer[22] | (buffer[23] << 8));

	ERR:
		k_work_submit(&i2c1_error_work);
		printk("Failed to calib BMP280 \n");
	return 0;
}

int32_t t_fine;

double compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    if (var1 == 0) return 0; // Tránh chia cho 0

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);

    return (double)p / 256.0; // Kết quả là Pascal
}

volatile float filtered_alt = 0;
void Calculate_Pressure(uint8_t *bmp_data){
	uint32_t Press_raw = (uint32_t)((bmp_data[0] << 12)|(bmp_data[1] << 4)|(bmp_data[2] >> 4));
	int32_t Temp_raw = (int32_t)((bmp_data[3] << 12)|(bmp_data[4] << 4)|(bmp_data[5] >> 4));

	int32_t var1, var2;
	var1 = ((((Temp_raw >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
	var2 = (((((Temp_raw >> 4) - ((int32_t)calib.dig_T1)) * ((Temp_raw >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
	t_fine = var1 + var2;

	double pressure_pa = compensate_pressure(Press_raw);

	double p0 = 101325.0; // Áp suất mực nước biển
	float altitude = 44330.0 * (1.0 - pow(pressure_pa / p0, 0.1903));

	    // 4. Lọc (Ví dụ: Lọc trung bình cộng đơn giản)
	filtered_alt = 0.95f * filtered_alt + 0.05f * altitude;
}
