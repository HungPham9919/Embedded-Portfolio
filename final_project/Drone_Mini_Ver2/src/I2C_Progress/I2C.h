#ifndef CHECK_ADDRESS_I2C_H
#define CHECK_ADDRESS_I2C_H

#include "stm32f405xx.h"
#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include "stdio.h"

#include <sys/_stdint.h>
#include "zephyr/drivers/pinctrl.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include "zephyr/drivers/pinctrl.h"
#include "zephyr/dt-bindings/gpio/gpio.h"
#include "zephyr/drivers/gpio.h"
#include "zephyr/devicetree/clocks.h"
#include <sys/_stdint.h>
#include <sys/errno.h>
#include "zephyr/logging/log.h"

typedef struct {
    uint8_t sensor1,sensor2,sensor3,sensor4,sensor5,sensor6,sensor7;
} DRONE_SENSOR;

extern volatile DRONE_SENSOR drone_sensor_addr;

struct i2c_dev_t {
    I2C_TypeDef *regs;
    const struct pinctrl_dev_config *pcfg;
    struct stm32_pclken pclken;
    uint32_t speed_hz;

    struct gpio_dt_spec scl_gpio;
    struct gpio_dt_spec sda_gpio;

    DMA_Stream_TypeDef *dma_tx_stream;
    DMA_Stream_TypeDef *dma_rx_stream;
    DMA_TypeDef *dma;
    uint32_t dma_rx_channel;
    uint32_t dma_tx_channel;
};

extern const struct device *dev_i2c1;
extern const struct device *dev_i2c3;

int drone_i2c_init_hw(const struct device *dev);
int drone_i2c_clearbus(const struct device *dev);
int drone_i2c_check_address(const struct device *dev, uint8_t *sensor_out, int nos);
int i2c_write_data(const struct device *dev, uint8_t slave_id, uint8_t reg, uint16_t value, uint8_t len, struct k_sem *dma_tx_irg_signal);
int i2c_dma_read_data(const struct device *dev, uint8_t slave_id, uint8_t reg,uint8_t *value, uint8_t len, struct k_sem *dma_irq_signal);

#endif
