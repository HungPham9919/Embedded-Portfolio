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
void usart_dma_wait_complete(const struct device *dev);
int usart_dma_rx(const struct device *dev, uint8_t *buffer, uint16_t length);
void Leader_Data_To_Followers(void);
void Follower_Data_From_Leader(void);

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

typedef enum {
    PACKET_ID_SENSOR = 0x01,
    PACKET_ID_TELE,
    COMMANDS,
} Packet_id;

typedef struct __attribute__((packed)){
    uint8_t packet_id;
	int16_t roll_tsf;
	int16_t pitch_tsf;
	int16_t yaw_tsf;
	int8_t x_pos_tsf;
	int8_t y_pos_tsf;
	int8_t z_pos_tsf;
    float press;
	int16_t pwm_tsf;
} Drone_data_transfer;

extern Drone_data_transfer packet;
typedef struct __attribute__((packed)){
    uint8_t packet_id;
    uint8_t bmi088_state : 1; // Using only 1 bit in mem
    uint8_t vl53_state : 1;
    uint8_t pmw3901_state : 1;
    uint8_t ina226_state : 1;
    uint8_t bmp280_state : 1;
    uint8_t hmc5883_state : 1;
} Sensor_Status;
extern Sensor_Status sensor_state;

#endif
