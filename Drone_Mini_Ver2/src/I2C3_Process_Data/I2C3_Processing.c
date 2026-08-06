#include "I2C3_Process_Data/I2C3_Processing.h"
#include "RTOS/RTOS_Init.h"
#include "zephyr/kernel.h"
#include "stdint.h"

int I2C3_Write_Data(uint8_t slave_id,uint8_t reg, uint8_t value){
    volatile uint32_t timeout = 10000;
    while((I2C3->SR2 & (1 << 1)) && --timeout); // BUSY
    if(timeout == 0) goto OFF_I2C3;

    I2C3->CR1 |= (1 << 8); // Start bit
    timeout = 10000;
    while(!(I2C3->SR1 & (1 << 0)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->DR = (slave_id << 1); // 8bit address
    timeout = 10000;
    while (!(I2C3->SR1 & (1 << 1)) && --timeout) {
        if(I2C3->SR1 & (1 << 10)){
            I2C3->CR1 |= (1 << 9);
            I2C3->SR1 &= ~(1 << 10);
            return 0;
        }
    }
    if(timeout == 0) goto OFF_I2C3;

    (void)I2C3->SR1;
    (void)I2C3->SR2;

    I2C3->DR = reg;
    timeout = 10000;
    while (!(I2C3->SR1 & (1 << 7)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->DR = value;
    timeout = 10000;
    while (!(I2C3->SR1 & (1 << 2)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->CR1 |= (1 << 9); // stop
    return 1;

OFF_I2C3:
    I2C3->CR1 |= (1 << 9); // STOP BIT
    return 0;
}

void I2C3_Write_Data_With_Retry(uint8_t slave_id, uint8_t reg, uint8_t value){
    while (1) {
        if(I2C3_Write_Data(slave_id, reg, value) == 1){
            break;
        }
        else {
            k_work_submit(&i2c3_error_work);
        }
    }
}

int I2C3_Read_Multiple_Byte(uint8_t slave_id, uint8_t reg, uint8_t *data, int length){
    volatile uint32_t timeout = 10000;
    while ((I2C3->SR2 & (1 << 1)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->CR1 |= (1 << 8);
    timeout = 10000;
    while (!(I2C3->SR1 & (1 << 0)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->DR = (slave_id << 1);
    timeout = 10000;
    while (!(I2C3->SR1 & (1 << 1)) && --timeout){
        if(I2C3->SR1 & (1 << 10)){
            I2C3->CR1 |= (1 << 9); 
            I2C3->SR1 &= ~(1 << 10);
            return 0;
        }
    };

    if(timeout == 0) goto OFF_I2C3;
    (void)I2C3->SR1;
    (void)I2C3->SR2;

    I2C3->DR = reg;
    timeout = 10000;
    while(!(I2C3->SR1 & (1 << 7)) && --timeout); // TC
    if(timeout == 0) goto OFF_I2C3;

    timeout = 10000;
    while(!(I2C3->SR1 & (1 << 2)) && --timeout); // BTF
    if(timeout == 0) goto OFF_I2C3;

    // Read Funtion

    timeout = 10000;
    I2C3->CR1 |= (1 << 8);
    timeout = 10000;
    while (!(I2C3->SR1 & (1 << 0)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->DR = (slave_id << 1)|1; // read mode
    timeout = 10000;
    while (!(I2C3->SR1 & ((1 << 1)|(1 << 10))) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    if(I2C3->SR1 & (1 << 10)){
        I2C3->SR1 &= ~(1 << 10); // AF
        I2C3->CR1 |= (1 << 9); 
        return 0;
    }

    if(length == 1){
        I2C3->CR1 &= ~(1 << 10); // ACK = 0
        (void)I2C3->SR1;
        (void)I2C3->SR2;
        I2C3->CR1 |= (1 << 9);
    }
    else {
        I2C3->CR1 |= (1 << 10); // ACK == 1
        (void)I2C3->SR1;
        (void)I2C3->SR2;
    }

    for (int i = 0; i < length; i++) {
        if (i == length - 2) {
            timeout = 10000;
            while(!(I2C3->SR1 & (1 << 6)) && --timeout); // Chờ RXNE của byte áp chót
            if(timeout == 0) goto OFF_I2C3;

            I2C3->CR1 &= ~(1 << 10); 
            I2C3->CR1 |= (1 << 9);   

            data[i] = I2C3->DR;
            continue;
        }
        
        timeout = 10000;
        while(!(I2C3->SR1 & (1 << 6)) && --timeout);
        if(timeout == 0) goto OFF_I2C3;
        data[i] = I2C3->DR;
    }
    return 1;

OFF_I2C3:
    I2C3->CR1 |= (1 << 9); // stop
    return 0;
}

void I2C3_Read_Multiple_Byte_Safe(uint8_t slave_id, uint8_t reg, uint8_t *data,uint8_t length, struct k_work *signal){
    while(1){
        if(I2C3_Read_Multiple_Byte(slave_id, reg, data, length) == 1) break;
        else {
            k_work_submit(signal);
            k_msleep(5);
        }
    }
}

// DMA buffer I2C3
volatile uint32_t dma_ndtr = 0;
int I2C3_Read_Data_DMA(uint8_t slave_id, uint8_t reg, uint8_t *data_dma, int length){
    volatile int timeout = 10000;
    while((I2C3->SR2 & (1 << 1)) && --timeout); // busy
    if(timeout == 0) goto OFF_I2C3;

    timeout = 10000;
    I2C3->CR1 |= (1 << 8);
    while(!(I2C3->SR1 & (1 << 0)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    timeout = 10000;
    I2C3->DR = (slave_id << 1); 
    while (!(I2C3->SR1 & (1 << 1)) && --timeout) {
        if(I2C3->SR1 & (1 << 10)){
            I2C3->SR1 &= ~(1 << 10);
            I2C3->CR1 |= (1 << 9); // stop i2c
            return 0;
        }
    };

    if(timeout == 0) goto OFF_I2C3;
    (void)I2C3->SR1;
    (void)I2C3->SR2;

    I2C3->DR = reg;
    timeout = 10000;
    while(!(I2C3->SR1 & (1 << 7)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;
    timeout = 10000;
    while(!(I2C3->SR1 & (1 << 2)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

                            // Read Function
    timeout = 10000;
    I2C3->CR1 |= (1 << 8);
    while(!(I2C3->SR1 & (1 << 0)) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    I2C3->DR = (slave_id << 1) | 1;
    while(!(I2C3->SR1 & ((1 << 1)|(1 << 10))) && --timeout);
    if(timeout == 0) goto OFF_I2C3;

    if(I2C3->SR1 & (1 << 10)){
        I2C3->SR1 &= ~(1 << 10); // AF
        I2C3->CR1 |= (1 << 9);
        return 0;
    }

    // DMA
    DMA1_Stream2->CR &= ~(1 << 0); // off to config
    while(DMA1_Stream2->CR & (1 << 0));

    DMA1->LIFCR = (0x3D << 16);
    DMA1_Stream2->FCR = 0;

    DMA1_Stream2->NDTR = 0;
    DMA1_Stream2->PAR = (uint32_t)&I2C3->DR;
    DMA1_Stream2->M0AR = (uint32_t)data_dma;

    DMA1_Stream2->NDTR = length;

	DMA1_Stream2->CR =
	      (3 << 25) |   // CHSEL=3
	      (3 << 16) |   // priority high
	      (1 << 10) |   // MINC
	      (1 << 4)	|	// TCIE
		  (1 << 0);     //Enable

    I2C3->CR2 |= (1 << 11)|(1 << 12); // DMA i2c enable and last byte

    dma_ndtr = DMA1_Stream2->NDTR;
    if(length == 1){
        I2C3->CR1 &= ~(1 << 10);
        (void)I2C3->SR1;
        (void)I2C3->SR2;
        I2C3->CR1 |= (1 << 9);
    }

    else {
        I2C3->CR1 |= (1 << 10); // ack = 1
        (void)I2C3->SR1;
        (void)I2C3->SR2;
    }

    // CPU sleeps here

    if(k_sem_take(&dma1_stream2_signal, K_FOREVER) == 0){
        if(length > 1){
            I2C3->CR1 |= (1 << 9);
        }
        I2C3->CR2 &= ~(1 << 11) &~(1 << 12);
        return 1;
    }

    return 0;

OFF_I2C3:
    I2C3->CR1 |= (1 << 9); // stop
    return 0;
}

void I2C3_Read_Data_DMA_Safe(uint8_t slave_id,uint8_t reg, uint8_t *data_dma, int length, struct k_work *signal){
    while (1) {
        if(I2C3_Read_Data_DMA(slave_id, reg, data_dma, length) == 1) break;
        else {
            k_work_submit(signal);
            k_msleep(5);
        }
    }
}