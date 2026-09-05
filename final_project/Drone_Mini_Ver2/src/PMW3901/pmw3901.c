#include "pmw3901.h"
#include "RTOS_Init.h"
#include "stm32f405xx.h"
#include "zephyr/device.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <sys/_stdint.h>
#include <sys/errno.h>

volatile int16_t delta_x = 0;
volatile int16_t delta_y = 0;
volatile int32_t pos_x = 0;
volatile int32_t pos_y = 0;
volatile uint8_t squal = 0;

Drone_Pos drone_pos;
const struct device *dev_spi2 = DEVICE_DT_GET(DT_NODELABEL(spi2));

static const uint8_t pmw3901_init_registers_table[][2] = {
    {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00},
    {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14},
    {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06},
    {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08},
    {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
    {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77},
    {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50},
    {0x7F, 0x0A}, {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11},
    {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F}, {0x4A, 0x78},
    {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
    {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x60}, {0x66, 0x08},
    {0x63, 0x78}, {0x7F, 0x15}, {0x48, 0x58}, {0x7F, 0x07},
    {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F},
    {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02},
    {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00}
};

static const uint8_t pmw3901_bitcraze_added[][2] = {
    {0x32, 0x44},{0x7F, 0x07},{0x40, 0x40},{0x7F, 0x06},
    {0x62, 0xF0},{0x63, 0x00},{0x7F, 0x0D},{0x48, 0xC0},
    {0x6F, 0xD5},{0x7F, 0x00},{0x5B, 0xA0},{0x4E, 0xA8},
    {0x5A, 0x50},{0x40, 0x80},
};

// static const uint8_t dummy_tx = 0x00;

static uint32_t spi_calc_br_bits(uint32_t pclk_hz, uint32_t target_hz) {
    uint32_t div = pclk_hz / target_hz;
    
    if (div <= 2)   return 0; /* /2   */
    if (div <= 4)   return 1; /* /4   */
    if (div <= 8)   return 2; /* /8   */
    if (div <= 16)  return 3; /* /16  */
    if (div <= 32)  return 4; /* /32  */
    if (div <= 64)  return 5; /* /64  */
    if (div <= 128) return 6; /* /128 */
    return 7;                 /* /256 */
}

int spi_init_hw(const struct device *dev){
    const struct spi_dev_t *cfg = dev->config;
    SPI_TypeDef *spi = cfg->regs;

    if(cfg->pcfg){
        int ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
        if(ret < 0) return ret;
    }

    if((uintptr_t)spi == SPI2_BASE){
        RCC->APB1ENR |= (1 << 14); // Bật clock SPI2
    }
    (void)RCC->APB1ENR;

    if (!gpio_is_ready_dt(&cfg->cs_gpio)) return -ENODEV;
    
    gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_INACTIVE);

    spi->CR1 &= ~(1 << 6);
    
    uint32_t pclk1_hz = 42000000;
    uint32_t br_bit = spi_calc_br_bits(pclk1_hz, cfg->freq);
    uint32_t spi_cr_val = 0;

    spi_cr_val |= (1 << 2);              // Master mode
    spi_cr_val |= (1 << 8) | (1 << 9);   // SSI = 1, SSM = 1
    spi_cr_val |= (br_bit << 3);          // Baudrate
    spi_cr_val |= (1 << 0) | (1 << 1);   // CPOL = 1, CPHA = 1 (SPI Mode 3)
    spi_cr_val &= ~(1 << 7);             // MSB first

    spi->CR1 = spi_cr_val;
    spi->CR1 |= (1 << 6);                // Enable SPI

    irq_connect_dynamic(14, 2, dma1_stream3_irqhandler, dev, 0);
    irq_enable(14);
    return 0;
}

int SPI_Transfer_Safe(SPI_TypeDef *spi, uint8_t data_out, uint8_t *data_in) {
    uint32_t timeout = 10000; 

    if (spi->SR & (1 << 0)) {
        volatile uint8_t dummy = *(volatile uint8_t *)&spi->DR;
        (void)dummy;
    }

    /* Wait TXE */
    while (!(spi->SR & (1 << 1))) {
        if (--timeout == 0) return -1; 
    }
    
    *(volatile uint8_t *)&spi->DR = data_out;

    timeout = 10000;
    while (!(spi->SR & (1 << 0))) {
        if (--timeout == 0) return -1; 
    }

    uint8_t rx_data = *(volatile uint8_t *)&spi->DR;
    if (data_in) {
        *data_in = rx_data;
    }

    /* Wait BSY */
    timeout = 10000;
    while (spi->SR & (1 << 7)) {
        if (--timeout == 0) return -1;
    }

    return 0; 
}

uint8_t pmw3901_read_reg(const struct device *dev, uint8_t reg_addr) {
    const struct spi_dev_t *cfg = dev->config;
    SPI_TypeDef *spi = cfg->regs;
    uint8_t val = 0;

    reg_addr &= 0x7F; // Bit 7 = 0 cho READ

    /* Chuẩn Zephyr ACTIVE LOW: set(1) -> CS LOW (0V) */
    gpio_pin_set_dt(&cfg->cs_gpio, 1); 

    SPI_Transfer_Safe(spi, reg_addr, NULL);
    k_usleep(50);
    SPI_Transfer_Safe(spi, 0x00, &val);

    /* set(0) -> CS HIGH (3.3V) */
    gpio_pin_set_dt(&cfg->cs_gpio, 0); 
    k_usleep(200);

    return val;
}

int pmw3901_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t data) {
    const struct spi_dev_t *cfg = dev->config;
    SPI_TypeDef *spi = cfg->regs;

    reg_addr |= 0x80; // Bit 7 = 1 cho WRITE

    /* CS ACTIVE (LOW) */
    gpio_pin_set_dt(&cfg->cs_gpio, 1);
    k_usleep(10);

    if (SPI_Transfer_Safe(spi, reg_addr, NULL) != 0) {
        gpio_pin_set_dt(&cfg->cs_gpio, 0); // CS INACTIVE (HIGH)
        return -1;
    }

    k_usleep(20); 

    if (SPI_Transfer_Safe(spi, data, NULL) != 0) {
        gpio_pin_set_dt(&cfg->cs_gpio, 0); // CS INACTIVE (HIGH)
        return -1;
    }

    k_usleep(10);
    gpio_pin_set_dt(&cfg->cs_gpio, 0); // CS INACTIVE (HIGH)
    k_usleep(100);        

    return 0;
}

volatile uint8_t product_id = 0, revision_id = 1, inverse_product = 0;

void optical_identify_id(const struct device *dev) {
    const struct spi_dev_t *cfg = dev->config;

    gpio_pin_set_dt(&cfg->cs_gpio, 0); // INACTIVE (HIGH 3.3V)
    k_usleep(50);
    gpio_pin_set_dt(&cfg->cs_gpio, 1); // ACTIVE (LOW 0V)
    k_usleep(50);
    gpio_pin_set_dt(&cfg->cs_gpio, 0); // INACTIVE (HIGH 3.3V)
    k_msleep(10);

    product_id = pmw3901_read_reg(dev, PMW3901_PRODUCT_ID);
    revision_id = pmw3901_read_reg(dev, PMW3901_REVISION_ID);
    inverse_product = pmw3901_read_reg(dev, PMW3901_INVERSE_PRODUCT_ID);
}

void pmw3901_init_registers(const struct device *dev) {
    pmw3901_write_reg(dev, PMW3901_RST, 0x5A);
    k_msleep(50);

    uint8_t size = sizeof(pmw3901_init_registers_table) / sizeof(pmw3901_init_registers_table[0]);
    for (uint8_t i = 0; i < size; i++) {
        pmw3901_write_reg(dev, pmw3901_init_registers_table[i][0], pmw3901_init_registers_table[i][1]);
    }

    k_msleep(10);

    uint8_t bit_size = sizeof(pmw3901_bitcraze_added) / sizeof(pmw3901_bitcraze_added[0]);
    for (uint8_t i = 0; i < bit_size; i++) {
        pmw3901_write_reg(dev, pmw3901_bitcraze_added[i][0], pmw3901_bitcraze_added[i][1]);
    }
    k_msleep(100);
}

volatile int dma1_stream3_count = 0;
void dma1_stream3_irqhandler(const void *arg) {
    const struct device *dev = (const struct device *)arg;
    if (dev == NULL) return;
    const struct spi_dev_t *cfg = dev->config;
    SPI_TypeDef *spi = cfg->regs;

    if (DMA1->LISR & (1 << 27)) {
        DMA1->LIFCR = (0x3D << 22);
        spi->CR2 &= ~((1 << 0) | (1 << 1));
        gpio_pin_set_dt(&cfg->cs_gpio, 0); // CS HIGH (INACTIVE)
        dma1_stream3_count++;
        k_sem_give(&dma1_stream3_signal);
    }
}

static const uint8_t dummy_tx = 0x00;

int pmw3901_read_burst_dma(const struct device *dev, uint8_t *buffer) {
    const struct spi_dev_t *cfg = dev->config;
    SPI_TypeDef *spi = cfg->regs;
    DMA_TypeDef *dma = cfg->dma;
    DMA_Stream_TypeDef *rx = cfg->dma_rx_stream;
    DMA_Stream_TypeDef *tx = cfg->dma_tx_stream;

    gpio_pin_set_dt(&cfg->cs_gpio, 1);

    if (SPI_Transfer_Safe(spi, 0x16, NULL) != 0) {
        gpio_pin_set_dt(&cfg->cs_gpio, 0); // CS INACTIVE (HIGH 3.3V)
        return -1;
    }

    k_busy_wait(45);

    uint32_t timeout = 10000;
    while (spi->SR & (1 << 7)) { // BSY
        if (--timeout == 0) {
            gpio_pin_set_dt(&cfg->cs_gpio, 0);
            return -ETIMEDOUT;
        }
    }

    if (spi->SR & (1 << 0)) {
        volatile uint8_t dummy = *(volatile uint8_t *)&spi->DR;
        (void)dummy;
    }

    rx->CR &= ~(1 << 0);
    tx->CR &= ~(1 << 0);
    while ((rx->CR & (1 << 0)) | (tx->CR & (1 << 0)));

    dma->LIFCR = 0x0F7D0F7D;
    dma->HIFCR = 0x0F7D0F7D;

    rx->PAR  = (uint32_t)&(spi->DR);
    rx->M0AR = (uint32_t)buffer;
    rx->NDTR = 12; // PMW3901 Burst Read = 12 Bytes
    rx->CR   = (cfg->dma_rx_channel << 25) | (2 << 16) | (1 << 10) | (1 << 4); // MINC=1, TCIE=1

    tx->PAR  = (uint32_t)&(spi->DR);
    tx->M0AR = (uint32_t)&dummy_tx;
    tx->NDTR = 12;
    tx->CR   = (cfg->dma_tx_channel << 25) | (1 << 16) | (1 << 6) | (1 << 4); // MINC=0, DIR=1 (Mem2Periph)

    spi->CR2 |= (1 << 0) | (1 << 1); // RXDMAEN, TXDMAEN
    rx->CR |= (1 << 0);
    tx->CR |= (1 << 0);

    if (k_sem_take(&dma1_stream3_signal, K_MSEC(10)) != 0) {
        spi->CR2 &= ~((1 << 0) | (1 << 1));
        gpio_pin_set_dt(&cfg->cs_gpio, 0); // CS INACTIVE (HIGH 3.3V)
        return -ETIMEDOUT;
    }

    return 0;
}

#define DT_DRV_COMPAT vnd_spi_write

#define DRONE_SPI_INIT(inst) \
    PINCTRL_DT_INST_DEFINE(inst); \
    static const struct spi_dev_t spi_dev_config_##inst = { \
        .regs = (SPI_TypeDef *)DT_INST_REG_ADDR(inst), \
        .freq = DT_INST_PROP_OR(inst, clock_frequency, 2000000), \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), \
        .cs_gpio = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, cs_gpios, 0), \
        .dma = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas), \
                ((DMA_TypeDef *)(DT_REG_ADDR(DT_INST_DMAS_CTLR_BY_NAME(inst, rx)))), \
                (NULL)), \
        .dma_rx_stream = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas), \
                ((DMA_Stream_TypeDef *)(DT_REG_ADDR(DT_INST_DMAS_CTLR_BY_NAME(inst, rx)) + \
                0x10 + 0x18 * DT_INST_DMAS_CELL_BY_NAME(inst, rx, channel))), \
                (NULL)), \
        .dma_rx_channel = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas), \
                (DT_INST_DMAS_CELL_BY_NAME(inst, rx, slot)), \
                (0)), \
        .dma_tx_stream = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas), \
                ((DMA_Stream_TypeDef *)(DT_REG_ADDR(DT_INST_DMAS_CTLR_BY_NAME(inst, tx)) + \
                0x10 + 0x18 * DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel))), \
                (NULL)), \
        .dma_tx_channel = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, dmas), \
                (DT_INST_DMAS_CELL_BY_NAME(inst, tx, slot)), \
                (0)), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, spi_init_hw, NULL, NULL, \
                          &spi_dev_config_##inst, POST_KERNEL, \
                          50, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRONE_SPI_INIT)
