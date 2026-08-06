#ifndef BMP280_H
#define BMP280_H

#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include "stdint.h"
#include "stdio.h"
#include <sys/_stdint.h>


#define BMP280_ADDR 0x76
#define BMP280_ID 0xD0
#define BMP280_MEASURE 0xF4
#define BMP280_CONFIG 0xF5
#define BMP280_PRESS 0xF7 // F7-F9
#define BMP280_Calib 0x88

/*
Measure : Control mode (2 bit) 11 - normal mode
osrs_p oversampling pressure 2-4 (3 bit) 101 ultra high precision
*/


typedef struct {
	uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_calib_data;
extern volatile uint8_t BMP_ID;

int BMP280_Initialized(void);
int BMP280_Calibration(void);
void Calculate_Pressure(uint8_t *bmp_data);
#endif