#ifndef PMW3901_H
#define PMW3901_H

#include "stm32f405xx.h"
#include "zephyr/drivers/gpio.h"
#include "zephyr/drivers/pinctrl.h"
#include "zephyr/kernel.h"
#include "zephyr/devicetree.h"
#include "stdint.h"
#include "stdio.h"
#include <sys/_stdint.h>

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

typedef struct {
    int x_pos, y_pos, z_pos;
} Drone_Pos;

struct spi_dev_t {
    SPI_TypeDef *regs;
    uint32_t freq;
    struct gpio_dt_spec cs_gpio;
    const struct pinctrl_dev_config *pcfg;
    
    DMA_TypeDef *dma;
    DMA_Stream_TypeDef *dma_tx_stream;
    DMA_Stream_TypeDef *dma_rx_stream;

    uint32_t dma_tx_channel;
    uint32_t dma_rx_channel;
};

extern volatile int16_t delta_x;
extern volatile int16_t delta_y;
extern volatile int32_t pos_x; // Tích lũy tọa độ X
extern volatile int32_t pos_y; // Tích lũy tọa độ Y
extern volatile uint8_t squal;


extern Drone_Pos drone_pos;
extern volatile uint8_t product_id, revision_id,inverse_product;
extern const struct device *dev_spi2;
void dma1_stream3_irqhandler(const void *arg);
void dma1_stream4_irqhandler(const void *arg);
void optical_identify_id(const struct device *dev);
void pmw3901_init_registers(const struct device *dev);
int pmw3901_read_burst_dma(const struct device *dev, uint8_t *buffer);
#endif
