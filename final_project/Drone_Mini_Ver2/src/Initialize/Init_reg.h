#ifndef INIT_REG_H
#define INIT_REG_H

#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include "zephyr/device.h"

#include "zephyr/drivers/uart.h"
#include "zephyr/drivers/spi.h"
#include "zephyr/drivers/dma.h"
#include "zephyr/drivers/i2c.h"
#include "zephyr/drivers/gpio.h"

#include "stdint.h"
#include "soc.h"
#include "stdio.h"
#include "stdint.h"
#include "zephyr/irq.h"

void Init_The_Config_Of_Drone(void);
void BUS_Init(void);
void DMA_I2C3_Stream(void);
void DMA_I2C1_Stream(void);

void The_First_State(void);

#endif