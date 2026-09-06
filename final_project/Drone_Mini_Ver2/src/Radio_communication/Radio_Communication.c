#include "Radio_Communication.h"
#include "bmi088.h"
#include "pmw3901.h"
#include "INA226.h"
#include "math.h"
#include "stm32f405xx.h"
#include "zephyr/drivers/pinctrl.h"
#include "zephyr/irq.h"
#include "zephyr/kernel.h"
#include <stdio.h>
#include <string.h>
#include <sys/_stdint.h>
#include "zephyr/drivers/pinctrl.h"

char number[11] = {'0','1','2','3','4','5','6','7','8','9','.'};
Sensor_Status sensor_state;
void dma2_stream1_irqhandler(const void *arg);

#define DT_DRV_COMPAT vnd_write_usart
const struct device *dev_usart6 = DEVICE_DT_GET(DT_NODELABEL(usart6));

int drone_usart_init_hw(const struct device *dev){
	const struct usart_dev_t *cfg = dev->config;
	USART_TypeDef *usart = cfg->regs;

    if (cfg->pcfg) {
        int ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if(ret < 0) return ret;
    }
	if((uintptr_t)usart == USART6_BASE) {
		RCC->APB2ENR |= (1 << 5);
		(void)RCC->APB2ENR;
	}

	RCC->AHB1ENR |= (1 << 22); // DMA2 enable

	usart->CR1 &= ~(1 << 13);
	(void)RCC->AHB1ENR;
	uint32_t pclk = 84000000; // Tần số APB2 bus
    uint32_t usartdiv = (pclk + (cfg->baudrate / 2)) / cfg->baudrate; // Tương đương (84M / 115200) = 729 (0x2D9)
    
    usart->BRR = (uint16_t)usartdiv;
	usart->CR1 |= (1 << 2)|(1 << 3)|(1 << 13);

	IRQ_CONNECT(DMA2_Stream1_IRQn, 2, dma2_stream1_irqhandler, NULL, 0);
	irq_enable(DMA2_Stream1_IRQn);

	k_busy_wait(2000);
	return 0;
}

int usart_dma_tx(const struct device *dev, uint8_t *buffer, uint16_t length){
	if(!dev || !dev->config) return -EINVAL;
	const struct usart_dev_t *cfg = dev->config;
	USART_TypeDef *usart = cfg->regs;
	DMA_TypeDef *dma = cfg->dma;
	DMA_Stream_TypeDef *dma_tx_stream = cfg->dma_tx_stream;

	dma_tx_stream->CR &= ~(1 << 0);
	while(dma_tx_stream->CR & (1 << 0)){};

	(void)usart->SR;
    (void)usart->DR;
	usart->SR &= ~(1 << 6);
    dma->LIFCR = 0x0F7D0F7D;
    dma->HIFCR = 0x0F7D0F7D;
	dma_tx_stream->FCR = 0;

	dma_tx_stream->PAR = (uint32_t)&usart->DR;
	dma_tx_stream->M0AR = (uint32_t)buffer;
	dma_tx_stream->NDTR = length;

	dma_tx_stream->CR = ((cfg->dma_tx_channel & 0x07) << 25)|(1 << 10)|(1 << 6);
	usart->CR3 |= (1 << 7);
	dma_tx_stream->CR |= (1 << 0);

	return 0;
}

void usart_dma_wait_complete(const struct device *dev) {
    const struct usart_dev_t *cfg = dev->config;
    while (cfg->dma_tx_stream->CR & (1 << 0));
    while (!(cfg->regs->SR & (1 << 6)));
}

int usart_dma_rx(const struct device *dev, uint8_t *buffer, uint16_t length){
	if(!dev || !dev->config) return -EINVAL;
	const struct usart_dev_t *cfg = dev->config;
	USART_TypeDef *usart = cfg->regs;
	DMA_TypeDef *dma = cfg->dma;
	DMA_Stream_TypeDef *dma_rx_stream = cfg->dma_rx_stream;

	dma_rx_stream->CR &= ~(1 << 0);
	while(dma_rx_stream->CR & (1 << 0)){};

    dma->LIFCR = 0x0F7D0F7D;
    dma->HIFCR = 0x0F7D0F7D;
	dma_rx_stream->FCR = 0;

	dma_rx_stream->PAR = (uint32_t)&(usart->DR);
	dma_rx_stream->M0AR = (uint32_t)buffer;
	dma_rx_stream->NDTR = length;

	dma_rx_stream->CR = ((cfg->dma_rx_channel & 0x7) << 25)|(1 << 10)|(1 << 8)|(1 << 4);
	usart->CR3 |= (1 << 6); // rx enable
	dma_rx_stream->CR |= (1 << 0);

	return 0;
}

void dma2_stream1_irqhandler(const void *arg){ // USART_RX
	ARG_UNUSED(arg);
	if(DMA2->LISR & (1 << 5)){
		DMA2->LIFCR = (0x3D << 6);
		k_sem_give(&dma2_stream1_signal);
	}
}

volatile int duty = 0;
int PWM_Converted(char *buffer){
	char temp[4];
	int k = 0;
	for(int i = 0; i < strlen(buffer); i++){
		for(int j = 0; j < 10; j++){
			if(buffer[i] == number[j]){
				temp[k++] = buffer[i];
				break;
			}
			if(k >= 3) break;
		}
		if(buffer[i] == '\0') break;
    }
	int val_return;
	duty = atoi(temp);
	if(duty > 90){
		val_return = 3024; // 90% - pwm
	}
	else {
		val_return = (3559*duty)/100;
	}
	return val_return;
}

int Atoi_Converted(char *buffer){
	char temp[2]; // max = local + NULL
	int k = 0;
	for(int i = 0; i < strlen(buffer); i++){
		for(int j = 0; j < 10; j++){
			if(buffer[i] == number[j]){
				temp[k++] = buffer[i];
				break;
			}
			if(k > 2) break;
		}
		if(buffer[i] == '\0') break;
    }
	return atoi(temp);
}

Drone_data_transfer packet;
void Leader_Data_To_Followers(void){ // Integer to char
	packet.packet_id = PACKET_ID_TELE;
	packet.roll_tsf = (int16_t)(roundf(drone_angle.Roll_angle * 100.0f));
	packet.pitch_tsf = (int16_t)(roundf(drone_angle.Pitch_angle * 100.0f));
	packet.yaw_tsf = (int16_t)(roundf(drone_angle.Yaw_angle * 100.0f));
	packet.x_pos_tsf = drone_pos.x_pos;
	packet.y_pos_tsf = drone_pos.y_pos;
	packet.z_pos_tsf = drone_pos.z_pos;

	usart_dma_tx(dev_usart6, (uint8_t *)&packet, sizeof(Drone_data_transfer));
	usart_dma_wait_complete(dev_usart6);
}

void Follower_Data_From_Leader(void){ // Char to integer
	// RPY + XYZ + PIN + % PWM
	// IRQ
}


#define DRONE_USART_INIT(inst)                                                 \
    PINCTRL_DT_INST_DEFINE(inst);                                              \
    static const struct usart_dev_t usart_dev_config_##inst = {               \
        .regs = (USART_TypeDef *)DT_INST_REG_ADDR(inst),                       \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                          \
        .baudrate = DT_INST_PROP_OR(inst, current_speed, 115200),             \
        .dma = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas),                  \
                     ((DMA_TypeDef *)DT_REG_ADDR(DT_INST_DMAS_CTLR_BY_NAME(inst, rx))), \
                     (NULL)),                                                  \
        /* DMA RX Stream & Channel */                                          \
        .dma_rx_stream = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas),        \
                    ((DMA_Stream_TypeDef *)(DT_REG_ADDR(DT_INST_DMAS_CTLR_BY_NAME(inst, rx)) + \
                    0x10 + 0x18 * DT_INST_DMAS_CELL_BY_NAME(inst, rx, channel))), \
                    (NULL)),                                                  \
        .dma_rx_channel = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas),       \
                    (DT_INST_DMAS_CELL_BY_NAME(inst, rx, slot)),              \
                    (0)),                                                     \
        /* DMA TX Stream & Channel */                                          \
        .dma_tx_stream = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas),        \
                    ((DMA_Stream_TypeDef *)(DT_REG_ADDR(DT_INST_DMAS_CTLR_BY_NAME(inst, tx)) + \
                    0x10 + 0x18 * DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel))), \
                    (NULL)),                                                  \
        .dma_tx_channel = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas),       \
                    (DT_INST_DMAS_CELL_BY_NAME(inst, tx, slot)),              \
                    (0)),                                                     \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(inst, drone_usart_init_hw, NULL, NULL,               \
                          &usart_dev_config_##inst, POST_KERNEL,               \
                          50, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRONE_USART_INIT)
