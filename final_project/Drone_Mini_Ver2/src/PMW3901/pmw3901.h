#ifndef PMW3901_H
#define PMW3901_H

#include "zephyr/kernel.h"
#include "zephyr/devicetree.h"
#include "stdint.h"
#include "stdio.h"

#define PMW3901_PRODUCT_ID 0x00
#define PMW3901_REVISION_ID 0X01
#define PMW3901_INVERSE_PRODUCT_ID 0x5F
#define MOTION 0x02
#define DELTA_X_L 0x03

#define PMW3901_RST 0x3A
#define PMW3901_MOTION_BRUST 0x16
// #define DELTA_X_H 0x04
// #define DELTA_Y_L 0x05
// #define DELTA_Y_H 0X06


void dma1_stream3_irqhandler(const void *arg);
void dma1_stream4_irqhandler(const void *arg);

void pmw3901_init_registers(void);
volatile uint8_t product_id, revision_id,inverse_product;
void optical_flow_sensor(void);
#endif