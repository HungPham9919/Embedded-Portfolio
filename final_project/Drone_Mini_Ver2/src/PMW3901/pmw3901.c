#include "pmw3901.h"
#include "RTOS_Init.h"
#include "stm32f405xx.h"
#include "zephyr/irq.h"
#include "zephyr/kernel.h"
#include <stdlib.h>
#include <sys/_stdint.h>

volatile int16_t delta_x = 0;
volatile int16_t delta_y = 0;
volatile int32_t pos_x = 0; // Tích lũy tọa độ X
volatile int32_t pos_y = 0; // Tích lũy tọa độ Y
volatile uint8_t squal = 0;
volatile uint8_t motion_flag = 0;
volatile uint8_t raw_sum = 0, raw_max = 0, raw_min = 0;
uint8_t burst_data[12];

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

void Optical_Flow_Init(void) {
    RCC->APB1ENR |= (1 << 14); // SPI2
    // 2. PB13 SCK || PB14 - MISO || PB15-MOSI
    GPIOB->MODER &= ~(3 << 26) &~(3 << 28) &~(3 << 30);
    GPIOB->MODER |=  (2 << 26)|(2 << 28)|(2 << 30);

    GPIOB->OSPEEDR |= (3 << 26)|(3 << 28)|(3 << 30); // High speed
    // AF5
    GPIOB->AFR[1] &= ~(0x0F << 20) &~(0x0F << 24) &~(0x0F << 28);
    GPIOB->AFR[1] |= (5 << 20)|(5 << 24)|(5 << 28);

    // 3. Cấu hình PB12 (CS) -> Output Push-Pull
    GPIOB->MODER &= ~(3 << 24);
    GPIOB->MODER |=  (1 << 24);
    GPIOB->ODR |=  (1 << 12); // CS High (De-assert)

    // 5. Cấu hình SPI2 Controller
    // Master mode, Baudrate Prescaler = /32 hoặc /16 (Clock SPI < 2MHz cho an toàn lúc init)
    SPI2->CR1 = 0; // Clear
    SPI2->CR1 |= (1 << 2)|(1 << 8)|(1 << 9);    // Master mode, Software CS
    SPI2->CR1 |= (1 << 0)|(1 << 1);             // CPOL = 1, CPHA = 1 - MODE 3
    SPI2->CR1 |= (0x04 << 3);                   // fPCLK/32
    SPI2->CR1 |= (1 << 6); // Enable SPI

    irq_connect_dynamic(15, 2, dma1_stream3_irqhandler, NULL, 0);
    irq_enable(15);
}

static const uint8_t dummy_tx = 0x00;

void SPI2_DMA_Transfer(uint8_t *rx_buf, uint16_t size){
    DMA1_Stream3->CR &= ~(1 << 0);
    DMA1_Stream4->CR &= ~(1 << 0);
    while ((DMA1_Stream3->CR & (1 << 0)) || (DMA1_Stream4->CR & (1 << 0)));

    DMA1->LIFCR = (0x3D << 22);
    DMA1->HIFCR = (0x3D << 0);

    DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
    DMA1_Stream3->M0AR = (uint32_t)rx_buf;
    DMA1_Stream3->NDTR = size;

    DMA1_Stream3->CR = (0 << 25)|(2 << 16)|(1 << 10)|(1 << 4);

    // TX
    DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
    DMA1_Stream4->M0AR = (uint32_t)&dummy_tx;
    DMA1_Stream4->NDTR = size;

    DMA1_Stream4->CR = (0 << 25)|(1 << 16)|(1 << 6)|(1 << 4); // off minc

    // CS Low - choose slave
    GPIOB->ODR &= ~(1 << 12);

    SPI2->CR2 |= (1 << 0)|(1 << 1); // RXDMA, TXDMA

    DMA1_Stream3->CR |= (1 << 0); // DMA Stream Enable
    DMA1_Stream4->CR |= (1 << 0);

    if(k_sem_take(&dma1_stream3_signal, K_MSEC(10)) != 0){
        GPIOB->ODR |= (1 << 12);
    }
}

void dma1_stream3_irqhandler(const void *arg){
    ARG_UNUSED(arg);
    if(DMA1->LISR & (1 << 27)){
        DMA1->LIFCR = (0x3D << 22);
        // Send signal to dma
        GPIOB->ODR |= (1 << 12); // CS High
        k_sem_give(&dma1_stream3_signal);
    }
}

uint8_t SPI_Transfer(uint8_t data){
    while (!(SPI2->SR & (1 << 1)));
    *(volatile uint8_t *)&SPI2->DR = data;
    while (!(SPI2->SR & (1 << 0)));
    return *(volatile uint8_t *)&SPI2->DR;
}

// Hàm đọc 1 thanh ghi từ PMW3901
uint8_t pmw3901_read_reg(uint8_t reg_addr) {
    uint8_t val = 0;
    reg_addr &= 0x7F; // bit 7 = 0 - Read
    GPIOB->BSRR = (1 << 28);
    SPI_Transfer(reg_addr);
    k_usleep(50);
    val = SPI_Transfer(0);
    GPIOB->BSRR = (1 << 12);
    k_usleep(200);
    return val;
}

// Hàm kiểm tra sensor
volatile uint8_t product_id = 0, revision_id = 1,inverse_product = 0;
void optical_flow_sensor(void) {
    // Reset SPI bus state bằng cách nhấp nháy CS
    GPIOB->BSRR = (1 << 28);
    k_usleep(50);
    GPIOB->BSRR = (1 << 12);
    k_usleep(1000);

    product_id = pmw3901_read_reg(PMW3901_PRODUCT_ID);
    revision_id = pmw3901_read_reg(PMW3901_REVISION_ID);
    inverse_product = pmw3901_read_reg(PMW3901_INVERSE_PRODUCT_ID);

    GPIOB->BSRR = (1 << 28);
    k_usleep(10);
    GPIOB->BSRR = (1 << 12);
    k_usleep(10);
}

void pmw3901_write_reg(uint8_t reg_addr, uint8_t data) {
    reg_addr |= 0x80; // Bit 7 = 1 cho thao tác WRITE
    GPIOB->BSRR = (1 << 28); // CS Low
    SPI_Transfer(reg_addr);
    SPI_Transfer(data);
    GPIOB->BSRR = (1 << 12); // CS High
    k_usleep(20);
}

void pmw3901_init_registers(void) {
    pmw3901_write_reg(PMW3901_RST, 0x5A);
    k_msleep(5);

    uint8_t size = sizeof(pmw3901_init_registers_table) / sizeof(pmw3901_init_registers_table[0]);
    for (uint8_t i = 0; i < size; i++) {
        pmw3901_write_reg(pmw3901_init_registers_table[i][0], pmw3901_init_registers_table[i][1]);
    }

    k_msleep(100);

    uint8_t bit_size = sizeof(pmw3901_bitcraze_added)/sizeof(pmw3901_bitcraze_added[0]);
    for(uint8_t i = 0; i < bit_size;i++){
        pmw3901_write_reg(pmw3901_bitcraze_added[i][0], pmw3901_bitcraze_added[i][1]);
    }
}

void pmw3901_read_motion_burst(uint8_t *buffer) {
    GPIOB->BSRR = (1 << 28); // CS Low
    k_usleep(20);

    SPI_Transfer(PMW3901_MOTION_BRUST);
    k_usleep(150);
    for (int i = 0; i < 12; i++) {
        buffer[i] = SPI_Transfer(0x00);
    }
    k_usleep(50);
    GPIOB->BSRR = (1 << 12); // CS High
    k_usleep(200);
}

        // // 1. Đọc burst trực tiếp mỗi chu kỳ mà không cần chờ PC2 == 0
        // pmw3901_read_motion_burst(burst_data);

        // // 2. Lấy các giá trị ra
        // motion_flag = burst_data[0];
        // squal       = burst_data[6];

        // // 3. Tạm thời BỎ ĐIỀU KIỆN (squal > 20) để debug xem sensor trả về squal bao nhiêu
        // // Nếu có bit Motion (motion_flag & 0x80) hoặc đơn giản là có delta khác 0
        // delta_x = (int16_t)((burst_data[3] << 8) | burst_data[2]);
        // delta_y = (int16_t)((burst_data[5] << 8) | burst_data[4]);

        // if (delta_x != 0 || delta_y != 0) {
        //     pos_x += delta_x;
        //     pos_y += delta_y;
        // }

        // HAL_Delay(10); // Đọc với chu kỳ ~100Hz
