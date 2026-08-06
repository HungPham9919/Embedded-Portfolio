#ifndef I2C1_PROCESSING_DATA_H
#define I2C1_PROCESSING_DATA_H

#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include "stdint.h"
#include "stdio.h"

int I2C1_Write_Data(uint8_t slave_id, uint8_t reg, uint16_t value, int length);
void I2C1_Write_Data_Safe(uint8_t slave_id, uint8_t reg, uint16_t value, int length);
int I2C1_Read_Multiple_Byte(uint8_t slave_id, uint8_t reg, uint8_t *data, int length);
void I2C1_Read_Multiple_Byte_Safe(uint8_t slave_id, uint8_t reg, uint8_t *data,uint8_t length, struct k_work *signal);

int I2C1_Read_Data_DMA(uint8_t slave_id ,uint8_t reg, uint8_t *data_dma, int len);
void I2C1_Read_Data_DMA_With_Safe(uint8_t slave_id, uint8_t reg, uint8_t *data, int len, struct k_work *signal);
#endif