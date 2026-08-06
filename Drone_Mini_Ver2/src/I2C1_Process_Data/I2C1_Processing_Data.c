#include "I2C1_Process_Data/I2C1_Processing_Data.h"
#include "RTOS/RTOS_Init.h"
#include "stm32f405xx.h"
#include "zephyr/kernel.h"
#include <sys/_stdint.h>

int I2C1_Write_Data(uint8_t slave_id, uint8_t reg, uint16_t value, int length){
    volatile uint32_t timeout = 10000;
    while((I2C1->SR2 & (1 << 1)) && --timeout); // BUSY
    if(timeout == 0) goto OFF_I2C1;

    I2C1->CR1 |= (1 << 8); // Start bit
    timeout = 10000;
    while(!(I2C1->SR1 & (1 << 0)) && --timeout);
    if(timeout == 0) goto OFF_I2C1;

    I2C1->DR = (slave_id << 1); // 8bit address
    timeout = 10000;
    while (!(I2C1->SR1 & (1 << 1)) && --timeout) {
        if(I2C1->SR1 & (1 << 10)){
            I2C1->CR1 |= (1 << 9);
            I2C1->SR1 &= ~(1 << 10);
            return 0;
        }
    }
    if(timeout == 0) goto OFF_I2C1;

    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->DR = reg;
    timeout = 10000;
    while (!(I2C1->SR1 & (1 << 7)) && --timeout);
    if(timeout == 0) goto OFF_I2C1;

    if(length == 1){
        I2C1->DR = (uint8_t)(value & 0x0F);
        timeout = 10000;
        while (!(I2C1->SR1 & (1 << 2)) && --timeout);
        if(timeout == 0) goto OFF_I2C1;
    }
    else {
        I2C1->DR = (uint8_t)((value >> 8)); // High
        timeout = 10000;
        while (!(I2C1->SR1 & (1 << 7)) && --timeout); // TC
        if(timeout == 0) goto OFF_I2C1;

        I2C1->DR = (uint8_t)(value & 0xFF); // Low
        timeout = 10000;
        while (!(I2C1->SR1 & (1 << 2)) && --timeout); // BTF
        if(timeout == 0) goto OFF_I2C1;
    }

    I2C1->CR1 |= (1 << 9); // stop
    return 1;

OFF_I2C1:
    I2C1->CR1 |= (1 << 9); // STOP BIT
    return 0;
}

void I2C1_Write_Data_Safe(uint8_t slave_id, uint8_t reg, uint16_t value, int length){
    while (1) {
        if(I2C1_Write_Data(slave_id, reg, value, length) == 1) break;
        else k_work_submit(&i2c1_error_work);
    }
}

int I2C1_Read_Multiple_Byte(uint8_t slave_id, uint8_t reg, uint8_t *data, int length){
    volatile uint32_t timeout = 10000;
    while ((I2C1->SR2 & (1 << 1)) && --timeout);
    if(timeout == 0) goto OFF_I2C1;

    I2C1->CR1 |= (1 << 8);
    timeout = 10000;
    while (!(I2C1->SR1 & (1 << 0)) && --timeout);
    if(timeout == 0) goto OFF_I2C1;

    I2C1->DR = (slave_id << 1);
    timeout = 10000;
    while (!(I2C1->SR1 & (1 << 1)) && --timeout){
        if(I2C1->SR1 & (1 << 10)){
            I2C1->CR1 |= (1 << 9); 
            I2C1->SR1 &= ~(1 << 10);
            return 0;
        }
    };

    if(timeout == 0) goto OFF_I2C1;
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->DR = reg;
    timeout = 10000;
    while(!(I2C1->SR1 & (1 << 7)) && --timeout); // TC
    if(timeout == 0) goto OFF_I2C1;

    timeout = 10000;
    while(!(I2C1->SR1 & (1 << 2)) && --timeout); // BTF
    if(timeout == 0) goto OFF_I2C1;

    // Read Funtion

    timeout = 10000;
    I2C1->CR1 |= (1 << 8);
    timeout = 10000;
    while (!(I2C1->SR1 & (1 << 0)) && --timeout);
    if(timeout == 0) goto OFF_I2C1;

    I2C1->DR = (slave_id << 1)|1; // read mode
    timeout = 10000;
    while (!(I2C1->SR1 & ((1 << 1)|(1 << 10))) && --timeout);
    if(timeout == 0) goto OFF_I2C1;

    if(I2C1->SR1 & (1 << 10)){
        I2C1->SR1 &= ~(1 << 10); // AF
        I2C1->CR1 |= (1 << 9); 
        return 0;
    }

    if(length == 1){
        I2C1->CR1 &= ~(1 << 10); // ACK = 0
        (void)I2C1->SR1;
        (void)I2C1->SR2;
        I2C1->CR1 |= (1 << 9);
    }
    else {
        I2C1->CR1 |= (1 << 10); // ACK == 1
        (void)I2C1->SR1;
        (void)I2C1->SR2;
    }

    for (int i = 0; i < length; i++) {
        if (i == length - 2) {
            timeout = 10000;
            while(!(I2C1->SR1 & (1 << 6)) && --timeout); // Chờ RXNE của byte áp chót
            if(timeout == 0) goto OFF_I2C1;

            I2C1->CR1 &= ~(1 << 10); 
            I2C1->CR1 |= (1 << 9);   

            data[i] = I2C1->DR;
            continue;
        }
        
        timeout = 10000;
        while(!(I2C1->SR1 & (1 << 6)) && --timeout);
        if(timeout == 0) goto OFF_I2C1;
        data[i] = I2C1->DR;
    }
    return 1;

OFF_I2C1:
    I2C1->CR1 |= (1 << 9); // stop
    return 0;
}

void I2C1_Read_Multiple_Byte_Safe(uint8_t slave_id, uint8_t reg, uint8_t *data,uint8_t length, struct k_work *signal){
    while(1){
        if(I2C1_Read_Multiple_Byte(slave_id, reg, data, length) == 1) break;
        else {
            k_work_submit(signal);
            k_msleep(5);
        }
    }
}

volatile int dma_ndtr_i2c1 = 0;
int I2C1_Read_Data_DMA(uint8_t slave_id ,uint8_t reg, uint8_t *data_dma, int len){
	volatile uint32_t timeout = 10000;
	while((I2C1->SR2 & (1 << 1)) && --timeout);
	if(timeout == 0) goto OFF_I2C1;

	timeout = 10000;
	I2C1->CR1 |= (1 << 8);
	while(!(I2C1->SR1 & (1 << 0)) && --timeout);
	if(timeout == 0) goto OFF_I2C1;

	I2C1->DR  = (slave_id << 1);
    while(!(I2C1->SR1 & (1 << 1)) && --timeout){
		if(I2C1->SR1 & (1 << 10)){
			I2C1->CR1 |= (1 << 9);
			I2C1->SR1 &= ~(1 << 10);
			return 0;
		}
    };

	if(timeout == 0) goto OFF_I2C1;

	(void)I2C1->SR1;
	(void)I2C1->SR2;

	I2C1->DR = reg;
	timeout = 10000;
	while(!(I2C1->SR1 & (1 << 7)) && --timeout);
	if(timeout == 0) goto OFF_I2C1;

	timeout = 10000;
	while(!(I2C1->SR1 & (1 << 2)) && --timeout);
	if(timeout == 0) goto OFF_I2C1;

						//read function
	timeout = 10000;
	I2C1->CR1 |= (1 << 8);
	while(!(I2C1->SR1 & (1 << 0)) && --timeout);

	if(timeout == 0) goto OFF_I2C1;

	I2C1->DR = (slave_id << 1) | 1; // read mode
	timeout = 10000;
	while(!(I2C1->SR1 & ((1 << 1)|(1 << 10))) && --timeout);
	if(timeout == 0) goto OFF_I2C1;

	if(I2C1->SR1 & (1 << 10)){
	    I2C1->CR1 |= (1 << 9);
	    I2C1->SR1 &= ~(1 << 10);
	    return 0;
	}
    // DMA
	DMA1_Stream5->CR &= ~(1 << 0); // off to config
	while(DMA1_Stream5->CR & (1 << 0));

	DMA1->HIFCR = (0x3D << 6);
	DMA1_Stream5->FCR = 0;

	DMA1_Stream5->NDTR = 0;
	DMA1_Stream5->PAR  = (uint32_t)&I2C1->DR;
	DMA1_Stream5->M0AR = (uint32_t)data_dma;

	DMA1_Stream5->NDTR = len;
	DMA1_Stream5->CR =
	      (1 << 25) |   // CHSEL=1
	      (3 << 16) |   // priority high
	      (1 << 10) |   // MINC
	      (1 << 4)	|	// TCIE
		  (1 << 0);     //	Enable
          
	dma_ndtr_i2c1 = DMA1_Stream5->NDTR;
	I2C1->CR2 |= (1 << 11)|(1 << 12); // DMA I2C enable and last byte
    if (len == 1)
    {
        I2C1->CR1 &= ~(1 << 10); // ACK = 0
        (void)I2C1->SR1;
        (void)I2C1->SR2;
        I2C1->CR1 |= (1 << 9); // STOP
    }
    else
    {
        I2C1->CR1 |= (1 << 10); // ACK = 1
        (void)I2C1->SR1;
        (void)I2C1->SR2;
    }
	// CPU sleep - stream 5
    k_sem_take(&dma1_stream5_signal, K_FOREVER);

	if (len > 1) {
	   I2C1->CR1 |= (1 << 9);
	}
	I2C1->CR2 &= ~(1 << 11) &~(1 << 12);
	return 1;

OFF_I2C1:
    I2C1->CR1 |= (1 << 9);
	return 0;
}

void I2C1_Read_Data_DMA_With_Safe(uint8_t slave_id, uint8_t reg, uint8_t *data, int len, struct k_work *signal) {
    while(1) {
        if (I2C1_Read_Data_DMA(slave_id, reg, data, len) == 1) {
            break;
        }
        else {
            k_work_submit(signal);
            k_msleep(5);
        }
    }
}
