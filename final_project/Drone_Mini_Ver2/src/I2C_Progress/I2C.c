#include "I2C.h"

volatile DRONE_SENSOR drone_sensor_addr;
#define DT_DRV_COMPAT st_stm32_i2c_v1

// Lấy thẳng con trở từ device node
const struct device *dev_i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
const struct device *dev_i2c3 = DEVICE_DT_GET(DT_NODELABEL(i2c3));

int drone_i2c_init_hw(const struct device *dev){
    const struct i2c_dev_t *cfg = dev->config;
    I2C_TypeDef *i2c = cfg->regs;

    if (cfg->pcfg) {
        int ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if(ret < 0) return ret;
    }

    if ((uintptr_t)i2c == I2C1_BASE) {
        RCC->APB1ENR |= (1 << 21);
    } else if ((uintptr_t)i2c == I2C3_BASE) {
        RCC->APB1ENR |= (1 << 23);
    }

    (void)RCC->APB1ENR;

    i2c->CR1 |= (1 << 15); 
    k_busy_wait(1000);
    i2c->CR1 &= ~(1 << 15); 
    k_busy_wait(1000);

    i2c->CR1 &= ~(1 << 0);

    uint32_t pclk1_mhz = 42;
    i2c->CR2 = pclk1_mhz;

    if(cfg->speed_hz >= 400000){
        uint16_t ccr = pclk1_mhz * 1000000 / (3 * cfg->speed_hz);
        i2c->CCR = (1 << 15) | ccr; 
        i2c->TRISE = ((pclk1_mhz * 300) / 1000) + 1;
    } else {
        uint16_t ccr = pclk1_mhz * 1000000 / (2 * cfg->speed_hz);
        i2c->CCR = ccr;
        i2c->TRISE = pclk1_mhz + 1;
    }

    i2c->CR1 |= (1 << 0);
    k_busy_wait(2000);

    return 0;
}

int drone_i2c_check_address(const struct device *dev, uint8_t *sensor_out, int nos){
    if (!dev || !dev->config) return -EINVAL;
    const struct i2c_dev_t *cfg = dev->config;
    I2C_TypeDef *i2c = cfg->regs;
    int found = 0;

    for(int adr = 1; adr < 127; adr++){ 
        uint32_t timeout = 10000;

        while((i2c->SR2 & (1 << 1)) && --timeout);
        if(!timeout) {
            i2c->CR1 |= (1 << 9);
            return -EBUSY; 
        };
        
        timeout = 10000;
        i2c->CR1 |= (1 << 8); 
        while (!(i2c->SR1 & (1 << 0)) && --timeout);
        if(!timeout) goto ERROR_TASK;

        i2c->DR = (adr << 1); 
        
        timeout = 10000;
        while (!(i2c->SR1 & ((1 << 1) | (1 << 10))) && --timeout);
        if(!timeout) goto ERROR_TASK;

        if(i2c->SR1 & (1 << 1)){
            if (found < nos) {
                sensor_out[found++] = adr;
            }

            (void)i2c->SR1;
            (void)i2c->SR2;
        }

        if(i2c->SR1 & (1 << 10)){
            i2c->SR1 &= ~(1 << 10);
        }

        i2c->CR1 |= (1 << 9);
        k_busy_wait(100);
    }

    return (found == nos) ? 0 : -ENODEV;

ERROR_TASK:
    i2c->CR1 |= (1 << 9); 
    return -ETIMEDOUT;
}

int drone_i2c_clearbus(const struct device *dev){
    if (!dev || !dev->config) return -EINVAL;
    const struct i2c_dev_t *cfg = dev->config;
    I2C_TypeDef *i2c = cfg->regs;

    i2c->CR1 &= ~(1 << 0);

    if(!gpio_is_ready_dt(&cfg->scl_gpio) || !gpio_is_ready_dt(&cfg->sda_gpio)) {
        i2c->CR1 |= (1 << 0);
        return -ENODEV;
    }

    gpio_pin_configure_dt(&cfg->scl_gpio, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
    gpio_pin_configure_dt(&cfg->sda_gpio, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
    k_busy_wait(2000);

    for (int i = 0; i < 9; i++) {
        if (gpio_pin_get_dt(&cfg->sda_gpio) > 0) {
            break; 
        }
        gpio_pin_set_dt(&cfg->scl_gpio, 0);
        k_busy_wait(2000);
        gpio_pin_set_dt(&cfg->scl_gpio, 1);
        k_busy_wait(2000);
    }

    gpio_pin_set_dt(&cfg->scl_gpio, 0); 
	k_busy_wait(2000);
    gpio_pin_set_dt(&cfg->sda_gpio, 0); 
	k_busy_wait(2000);
    gpio_pin_set_dt(&cfg->scl_gpio, 1); 
	k_busy_wait(2000);
    gpio_pin_set_dt(&cfg->sda_gpio, 1); 
	k_busy_wait(2000);

    if (cfg->pcfg) {
        pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    }

    i2c->CR1 |= (1 << 0); 
    k_busy_wait(2000);
    return 0;
}

int i2c_write_data(const struct device *dev,uint8_t slave_id, uint8_t reg, uint16_t value, uint8_t len){
	if(!dev || !(dev->config)) return -EINVAL;
	const struct i2c_dev_t *cfg = dev->config;
    I2C_TypeDef *i2c = cfg->regs;

    uint32_t timeout = 10000;

    while ((i2c->SR2 & (1 << 1)) && --timeout);
    if (!timeout) goto WRITE_ERROR;

    timeout = 10000;
    i2c->CR1 |= (1 << 8);
    while(!(i2c->SR1 & (1 << 0)) && --timeout);
    if(!timeout) goto WRITE_ERROR;

    i2c->DR = (slave_id << 1);
    timeout = 10000;
    while(!(i2c->SR1 & (1 << 1)) && --timeout){
        if(i2c->SR1 & (1 << 10)){
            i2c->CR1 |= (1 << 9);
            i2c->SR1 &= ~(1 << 10);
            return -EIO;
        };
    };

    if(!timeout) goto WRITE_ERROR;
    (void)i2c->SR1;
    (void)i2c->SR2;

    timeout = 10000;
    i2c->DR = reg;
    while(!(i2c->SR1 & (1 << 7)) && --timeout); // TXE
    if(!timeout) goto WRITE_ERROR;

    if(len == 1){
        timeout = 10000;
        i2c->DR = (uint8_t)(value & 0xFF);
        while(!(i2c->SR1 & (1 << 2)) && --timeout);
        if(!timeout) goto WRITE_ERROR;
    }
    else {
        timeout = 10000;
        i2c->DR = (uint8_t)(value >> 8);
        while(!(i2c->SR1 & (1 << 7)) && --timeout);
        if(!timeout) goto WRITE_ERROR;

        timeout = 10000;
        i2c->DR = (uint8_t)(value & 0xFF);
        while(!(i2c->SR1 & (1 << 2)) && --timeout);
        if(!timeout) goto WRITE_ERROR;
    }

    i2c->CR1 |= (1 << 9);
    return 0;

WRITE_ERROR:
    i2c->CR1 |= (1 << 9);
    return -ETIMEDOUT;
}

int i2c_dma_read_data(const struct device *dev, uint8_t slave_id, uint8_t reg,uint8_t *value, uint8_t len, struct k_sem *dma_irq_signal){
    const struct i2c_dev_t *cfg = dev->config;
    I2C_TypeDef *i2c = cfg->regs;
    DMA_TypeDef *dma = cfg->dma;
    DMA_Stream_TypeDef *dma_stream = cfg->dma_stream;

    if(!dev || !dev->config) return -EINVAL;
    uint32_t timeout = 10000;
    while (i2c->SR2 & (1 << 1) && --timeout); // BUSY
    if(!timeout) goto ERR;

    timeout = 10000;
    i2c->CR1 |= (1 << 8);
    while (!(i2c->SR1 & (1 << 0)) && --timeout);
    if(!timeout) goto ERR;

    timeout = 10000;
    i2c->DR = (slave_id << 1);
    while(!(i2c->SR1 & (1 << 1)) && --timeout);
    if(i2c->SR1 & (1 << 10)){
        i2c->SR1 &=~(1 << 10);
        goto ERR;
    };

    if(!timeout) goto ERR;
    (void)i2c->SR1;
    (void)i2c->SR2;

    timeout = 10000;
    i2c->DR = reg;
    while(!(i2c->SR1 & (1 << 7)) && --timeout);
    if(!timeout) goto ERR;

    while(!(i2c->SR1 & (1 << 2)) && --timeout);
    if(!timeout) goto ERR;

    timeout = 10000;
    i2c->CR1 |= (1 << 8);
    while(!(i2c->SR1 & (1 << 0)) && --timeout);
    if(!timeout) goto ERR;

    timeout = 10000;
    i2c->DR = (slave_id << 1) | 1; // read
    while (!(i2c->SR1 & (1 << 1)) && --timeout);
    if(i2c->SR1 & (1 << 10)) {
        i2c->SR1 &= ~(1 << 10);
        goto ERR;
    }
    if(!timeout) goto ERR;
    // dma

    dma_stream->CR &= ~(1 << 0); // off to config
    while (dma_stream->CR & (1 << 0));

    dma->LIFCR = 0x0F7D0F7D;
    dma->HIFCR = 0x0F7D0F7D; // clear

    dma_stream->FCR = 0;

    dma_stream->NDTR = 0;
    dma_stream->PAR = (uint32_t)&i2c->DR;
    dma_stream->M0AR = (uint32_t)value;
    dma_stream->NDTR = len;

    dma_stream->CR = (cfg->dma_rx_channel << 25)|(2 << 16)|(1 << 10)|(1 << 4)|(1 << 0);
    i2c->CR2 |= (1 << 11)|(1 << 12);

    if(len == 1){
        i2c->CR2 |= (1 << 11) | (1 << 12); // DMAEN + LAST
    }
    else {
        i2c->CR2 |= (1 << 11);
    }

    (void)i2c->SR1;
    (void)i2c->SR2;

    if(k_sem_take(dma_irq_signal,K_FOREVER) == 0){
        i2c->CR1 |= (1 << 9);
        i2c->CR2 &= ~(1 << 11) &~(1 << 12);
        return 0;
    }

ERR:
    i2c->CR1 |= (1 << 9);
    i2c->CR2 &= ~((1 << 11) | (1 << 12));
    return -ETIMEDOUT;
}

#define DRONE_I2C_INIT(inst)                                                   \
    PINCTRL_DT_INST_DEFINE(inst);                                              \
    static const struct i2c_dev_t i2c_dev_config_##inst = {                    \
        .regs = (I2C_TypeDef *)DT_INST_REG_ADDR(inst),                         \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                          \
        .speed_hz = DT_INST_PROP(inst, clock_frequency),                       \
        .scl_gpio = GPIO_DT_SPEC_INST_GET(inst, scl_gpios),                    \
        .sda_gpio = GPIO_DT_SPEC_INST_GET(inst, sda_gpios),                    \
        .dma = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas),                  \
                     ((DMA_TypeDef *)DT_REG_ADDR(DT_INST_DMAS_CTLR_BY_NAME(inst, rx))), \
                     (NULL)),                                                  \
        .dma_stream = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas),           \
                    ((DMA_Stream_TypeDef *)(DT_REG_ADDR(DT_INST_DMAS_CTLR_BY_NAME(inst, rx)) + \
                    0x10 + 0x18 * DT_INST_DMAS_CELL_BY_NAME(inst, rx, channel))), \
                    (NULL)),                                                  \
        .dma_rx_channel = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas),       \
                    (DT_INST_DMAS_CELL_BY_NAME(inst, rx, slot)),              \
                    (0)),                                                     \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(inst, drone_i2c_init_hw, NULL, NULL,                 \
                          &i2c_dev_config_##inst, POST_KERNEL,                 \
                          50, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRONE_I2C_INIT)



