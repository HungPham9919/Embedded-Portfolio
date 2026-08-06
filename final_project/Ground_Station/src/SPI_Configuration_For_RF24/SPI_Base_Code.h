#ifndef SPI_BASE_CODE_H
#define SPI_BASE_CODE_H

#include "zephyr/kernel.h"
#include "zephyr/devicetree.h"

#include "zephyr/irq.h"
#include <stdint.h>
#include "stdio.h"

#define Choose_SPI1 1
#define Choose_SPI2 2
#define Choose_SPI3 3


void SPI_Config(int SPI_x, SPI_TypeDef *SPI, DMA_TypeDef DMA, DMA_Stream_TypeDef *RX_Stream,DMA_Stream_TypeDef *TX_Stream, uint32_t RX_Channel,uint32_t TX_Channel, int irq_pos,int irq_pri,void (*isr_rx_handler)(const void *),void (*isr_tx_handler)(const void *));
void DMA_SPI_RX_Config(SPI_TypeDef *SPI, DMA_TypeDef DMA, DMA_Stream_TypeDef *RX_Stream, uint32_t RX_Channel, int irq_pos,int irq_pri,void (*isr_handler)(const void *));
void DMA_SPI_TX_Config(SPI_TypeDef *SPI, DMA_TypeDef DMA, DMA_Stream_TypeDef *TX_Stream, uint32_t TX_Channel,int irq_pos,int irq_pri,void (*isr_handler)(const void *));


void exti0_handler(const void *arg);
void exti1_handler(const void *arg);
void exti2_handler(const void *arg);
void exti3_handler(const void *arg);
void exti4_handler(const void *arg);

void dma1_stream3_irqhandler(const void *arg);
void dma1_stream4_irqhandler(const void *arg);
void dma1_stream2_irqhandler(const void *arg);
void dma1_stream5_irqhandler(const void *arg);

void dma2_stream0_irqhandler(const void *arg);
void dma2_stream3_irqhandler(const void *arg);
#endif