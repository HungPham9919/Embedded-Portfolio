#ifndef RADIO_COMMUNICATION_H
#define RADIO_COMMUNICATION_H

#include "stdio.h"
#include "stm32f405xx.h"
#include "string.h"
#include "stdlib.h"
#include "zephyr/devicetree.h"
#include "zephyr/drivers/pinctrl.h"
#include "zephyr/kernel.h"
#include <sys/_stdint.h>

extern const struct device *dev_usart6;

int PWM_Converted(char *buffer);
int usart_dma_tx(const struct device *dev, uint8_t *buffer, uint16_t length);
struct usart_dev_t {
    USART_TypeDef *regs;
    const struct pinctrl_dev_config *pcfg;
    uint32_t baudrate;
    DMA_TypeDef *dma;
    DMA_Stream_TypeDef *dma_tx_stream;
    DMA_Stream_TypeDef *dma_rx_stream;
    uint32_t dma_rx_channel;
    uint32_t dma_tx_channel;
};


#endif
