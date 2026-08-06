#include "Ranging_Sensor/VL53L1X.h"
#include "I2C3_Process_Data/I2C3_Processing.h"
#include <stdint.h>
#include "stdio.h"
#include "RTOS/RTOS_Init.h"
#include "vl53l1_ll_device.h"
#include "vl53l1_platform.h"
// I2C3 - 16bit 

int I2C3_Write_VL53(uint8_t slave_id, uint16_t reg, uint8_t *data,int length){
    volatile uint32_t timeout = 10000;
    while((I2C3->SR2 & (1 << 1)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->CR1 |= (1 << 8);
    timeout = 10000;
    while(!(I2C3->SR1 & (1 << 0)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->DR = (slave_id << 1);
    timeout = 10000;
    while(!(I2C3->SR1 & (1 << 1)) && --timeout);
    if(I2C3->SR1 & (1 << 10)){
        I2C3->SR1 &= ~(1 << 10); // ACK = 0
        I2C3->CR1 |= (1 << 9);
    }

    if(timeout == 0) goto OFF_I2C3;
    (void)I2C3->SR1;
    (void)I2C3->SR2;

    I2C3->DR = (uint8_t)(reg >> 8);
    timeout = 10000;
    while(!(I2C3->SR1 & (1 << 7)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->DR = (uint8_t)(reg & 0xFF);
    timeout = 10000;
    while(!(I2C3->SR1 & (1 << 7)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    for(int i = 0; i < length ;i++){
        I2C3->DR = data[i];
        timeout = 10000;
        while(!(I2C3->SR1 & (1 << 7)) && --timeout);
        if(timeout == 0) goto OFF_I2C3;
    }

    timeout = 10000;
    while (!(I2C3->SR1 & (1 << 2)) && --timeout); // BTF
    if (timeout == 0) goto OFF_I2C3;

    I2C3->CR1 |= (1 << 9);

    return 1;
OFF_I2C3:
    I2C3->CR1 |= (1 << 9); // stop i2c
    return 0;
}

void I2C3_Write_VL53_With_Retry(uint8_t slave_id, uint16_t reg, uint8_t *value, int length){
    while (1) {
        if(I2C3_Write_VL53(slave_id, reg, value, length) == 1){
            break;
        }
        else {
            k_work_submit(&i2c3_error_work);
        }
    }
}
