#include "SPI_Base_Code.h"
#include "zephyr/irq.h"

void SPI_Config(int SPI_x, SPI_TypeDef *SPI, DMA_TypeDef DMA, DMA_Stream_TypeDef *RX_Stream,DMA_Stream_TypeDef *TX_Stream, uint32_t RX_Channel,uint32_t TX_Channel, int irq_pos,int irq_pri,void (*isr_rx_handler)(const void *),void (*isr_tx_handler)(const void *)){
    RCC->AHB1ENR |= (1 << 0)|(1 << 1)|(1 << 2); // GPIO A-B-C
    RCC->APB2ENR |= (1 << 14); // SYS
    switch (SPI_x) {
        case 1:
            // DMA2 || SPI1 - RX : Stream 0 || SPI1 - TX : Stream 3
            RCC->APB2ENR |= (1 << 12); // SPI1
            RCC->AHB1ENR |= (1 << 22); // DMA2 enable
            DMA_SPI_RX_Config(SPI, DMA, RX_Stream, RX_Channel, irq_pos,irq_pri,isr_rx_handler);
            DMA_SPI_TX_Config(SPI, DMA, TX_Stream, TX_Channel, irq_pos, irq_pri,isr_tx_handler);
            break;
        case 2:
            // DMA1 || SPI2 - RX: stream 3 || SPI2 - TX: Stream 4
            RCC->APB1ENR |= (1 << 14); // SPI2
            RCC->AHB1ENR |= (1 << 21); // DMA1 enable
            DMA_SPI_RX_Config(SPI, DMA, RX_Stream, RX_Channel, irq_pos,irq_pri,isr_rx_handler);
            DMA_SPI_TX_Config(SPI, DMA, TX_Stream, TX_Channel, irq_pos, irq_pri,isr_tx_handler);
            break;
        default:
            // DMA1 || SPI3 - RX : Stream 2 || SPI3 - TX : Stream 5
            RCC->APB1ENR |= (1 << 15); // SPI3
            RCC->AHB1ENR |= (1 << 21); // DMA1 enable
            DMA_SPI_RX_Config(SPI, DMA, RX_Stream, RX_Channel, irq_pos,irq_pri,isr_rx_handler);
            DMA_SPI_TX_Config(SPI, DMA, TX_Stream, TX_Channel, irq_pos, irq_pri,isr_tx_handler);
            break;
    }
    k_msleep(100);
    GPIOA->MODER &= ~(3 << 0) &~(3 << 2) &~(3 << 4) &~(3 << 6) &~(3 << 8); // clear
    GPIOA->MODER |= (1 << 0)|(1 << 2)|(1 << 4)|(1 << 6)|(1 << 8); // Output pp

    GPIOB->MODER &= ~(3 << 0) &~(3 << 2) &~(3 << 4) &~(3 << 6) &~(3 << 8);
    GPIOB->MODER |= (1 << 0)|(1 << 2)|(1 << 4)|(1 << 6)|(1 << 8); // OPP

    SYSCFG->EXTICR[0] |= (2 << 0)|(2 << 4)|(2 << 8)|(2 << 12); // exti 0-3 - PC
    SYSCFG->EXTICR[1] |= (2 << 0); // exti 4
    EXTI->RTSR |= (1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 4);
    EXTI->IMR |= (1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 4);
    EXTI->PR |= (1 << 0)|(1 << 1)|(1 << 2)|(1 << 3)|(1 << 4); // clear pending

    IRQ_CONNECT(0, 5, exti0_handler, NULL, 0);
    IRQ_CONNECT(1, 5, exti1_handler, NULL, 0);
    IRQ_CONNECT(2, 5, exti2_handler, NULL, 0);
    IRQ_CONNECT(3, 5, exti3_handler, NULL, 0);
    IRQ_CONNECT(4, 5, exti4_handler, NULL, 0);
}

// Doing something else

void DMA_SPI_RX_Config(SPI_TypeDef *SPI, DMA_TypeDef DMA, DMA_Stream_TypeDef *RX_Stream, uint32_t RX_Channel, int irq_pos,int irq_pri,void (*isr_handler)(const void *)){
    RX_Stream->CR &= ~(1 << 0);
    DMA.LIFCR = 0x0F7D0F7D; 
    DMA.HIFCR = 0x0F7D0F7D; 
    RX_Stream->PAR = (uint32_t)&(SPI->DR);

    uint32_t rx_config = 0;
    rx_config |= (RX_Channel << 25);
    rx_config |= (3 << 16); // High pri
    rx_config |= (1 << 10); // mem increment
    rx_config |= (1 << 4); // interrupt

    RX_Stream->CR = rx_config;
    RX_Stream->FCR = 0;

    SPI->CR2 |= (1 << 0);

    irq_connect_dynamic(irq_pos, irq_pri, isr_handler, NULL, 0);
    irq_enable(irq_pos);
}

void DMA_SPI_TX_Config(SPI_TypeDef *SPI, DMA_TypeDef DMA, DMA_Stream_TypeDef *TX_Stream, uint32_t TX_Channel,int irq_pos,int irq_pri,void (*isr_handler)(const void *)){
    TX_Stream->CR &= ~(1 << 0);
    DMA.LIFCR = 0x0F7D0F7D; 
    DMA.HIFCR = 0x0F7D0F7D; 
    TX_Stream->PAR = (uint32_t)&(SPI->DR);

    uint32_t tx_config = 0;
    tx_config |= (TX_Channel << 25);
    tx_config |= (3 << 16); // High pri
    tx_config |= (1 << 6); // mem to pe
    tx_config |= (1 << 4); // interrupt

    TX_Stream->CR = tx_config;
    TX_Stream->FCR = 0;
    SPI->CR2 |= (1 << 1);
    irq_connect_dynamic(irq_pos, irq_pri, isr_handler, NULL, 0);
    irq_enable(irq_pos);
}

void dma1_stream2_irqhandler(const void *arg){
    ARG_UNUSED(arg);

	if(DMA1->LISR & (1 << 21)){
	    DMA1->LIFCR = (0x3D << 16); // clear flag
        // Send signal
	}
}

void dma1_stream3_irqhandler(const void *arg){
    ARG_UNUSED(arg);

    if(DMA1->LISR & (1 << 27)){
	    DMA1->LIFCR = (0x3D << 22); // clear flag
        // Send signal
	}
}

void dma1_stream4_irqhandler(const void *arg){ // TX
    ARG_UNUSED(arg);
    if(DMA1->HISR & (1 << 5)){
	    DMA1->HIFCR = (0x3D << 0); // clear flag
        // Send signal
	}
}

void dma1_stream5_irqhandler(const void *arg){  // TX
    ARG_UNUSED(arg);

    if(DMA1->HISR & (1 << 11)){
	    DMA1->HIFCR = (0x3D << 6); // clear flag
        // Send signal
	}
}

void dma2_stream0_irqhandler(const void *arg){ // RX
    ARG_UNUSED(arg);
	if(DMA2->LISR & (1 << 5)){
	    DMA2->LIFCR = (0x3D << 0); // clear flag
        // Send signal
	}
}

void dma2_stream3_irqhandler(const void *arg){ // TX
    ARG_UNUSED(arg);
	if(DMA2->LISR & (1 << 27)){
	    DMA2->LIFCR = (0x3D << 16); // clear flag
        // Send signal
	}
}

void exti0_handler(const void *arg){
    ARG_UNUSED(arg);
    if(EXTI->PR & (1 << 0)){
        EXTI->PR = (1 << 0);
    }
}

void exti1_handler(const void *arg){
    ARG_UNUSED(arg);
    if(EXTI->PR & (1 << 1)){
        EXTI->PR = (1 << 1);
    }
}
void exti2_handler(const void *arg){
    ARG_UNUSED(arg);
    if(EXTI->PR & (1 << 2)){
        EXTI->PR = (1 << 2);
    }
}
void exti3_handler(const void *arg){
    ARG_UNUSED(arg);
    if(EXTI->PR & (1 << 3)){
        EXTI->PR = (1 << 3);
    }
}
void exti4_handler(const void *arg){
    ARG_UNUSED(arg);
    if(EXTI->PR & (1 << 4)){
        EXTI->PR = (1 << 4);
    }
}