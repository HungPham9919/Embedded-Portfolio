#ifndef I2C3_PROCESS_DATA_H
#define I2C3_PROCESS_DATA_H

#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"

int I2C3_Write_Data(uint8_t slave_id,uint8_t reg, uint8_t value);
void I2C3_Write_Data_With_Retry(uint8_t slave_id, uint8_t reg, uint8_t value);
int I2C3_Read_Multiple_Byte(uint8_t slave_id, uint8_t reg, uint8_t *data, int length);
void I2C3_Read_Multiple_Byte_Safe(uint8_t slave_id, uint8_t reg, uint8_t *data,uint8_t length, struct k_work *signal);
int I2C3_Read_Data_DMA(uint8_t slave_id, uint8_t reg, uint8_t *data_dma, int length);
void I2C3_Read_Data_DMA_Safe(uint8_t slave_id,uint8_t reg, uint8_t *data_dma, int length, struct k_work *signal);

#endif