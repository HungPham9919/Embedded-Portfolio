#include "pmw3901.h"

volatile int16_t delta_x = 0;
volatile int16_t delta_y = 0;
volatile int32_t pos_x = 0; // Tích lũy tọa độ X
volatile int32_t pos_y = 0; // Tích lũy tọa độ Y
volatile uint8_t squal = 0;
volatile uint8_t squal_Reg = 0;
volatile uint8_t motion_flag = 0;
volatile uint8_t raw_sum = 0, raw_max = 0, raw_min = 0;
uint8_t burst_data[12];


const uint8_t pmw3901_init_registers_table[][2] = {
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

void optical_flow_sensor_init(void) {
    // 1. Bật clock cho GPIOC và SPI3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

    // 2. Cấu hình PC10 (SCK), PC11 (MISO), PC12 (MOSI) -> Alternate Function (AF6 cho SPI3)
    GPIOC->MODER &= ~((3U << (10 * 2)) | (3U << (11 * 2)) | (3U << (12 * 2)));
    GPIOC->MODER |=  ((2U << (10 * 2)) | (2U << (11 * 2)) | (2U << (12 * 2)));

    GPIOC->OSPEEDR |= ((3U << (10 * 2)) | (3U << (11 * 2)) | (3U << (12 * 2))); // High speed

    // Gán AF6 (SPI3) cho PC10, PC11, PC12
    GPIOC->AFR[1] &= ~((0xFU << ((10 - 8) * 4)) | (0xFU << ((11 - 8) * 4)) | (0xFU << ((12 - 8) * 4)));
    GPIOC->AFR[1] |=  ((0x6U << ((10 - 8) * 4)) | (0x6U << ((11 - 8) * 4)) | (0x6U << ((12 - 8) * 4)));

    // 3. Cấu hình PC3 (CS) -> Output Push-Pull
    GPIOC->MODER &= ~(3U << (3 * 2));
    GPIOC->MODER |=  (1U << (3 * 2));
    GPIOC->ODR   |=  (1U << 3); // CS High (De-assert)

    // 4. Cấu hình PC2 (EXTI / MOT) -> Input Pull-up
    GPIOC->MODER &= ~(3U << (2 * 2));
    GPIOC->PUPDR &= ~(3U << (2 * 2));
    GPIOC->PUPDR |=  (1U << (2 * 2)); // Pull-up vì Active LOW

    // 5. Cấu hình SPI3 Controller
    // Mode 3 (CPOL=1, CPHA=1) là chuẩn chạy ổn định nhất cho PMW3901
    // Master mode, Baudrate Prescaler = /32 hoặc /16 (Clock SPI < 2MHz cho an toàn lúc init)
    SPI3->CR1 = 0; // Clear
    SPI3->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM; // Master mode, Software CS
    SPI3->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;               // SPI Mode 3
    SPI3->CR1 |= (0x4U << SPI_CR1_BR_Pos);

    SPI3->CR1 |= SPI_CR1_SPE; // Enable SPI3
}

// Hàm Transmit/Receive 1 byte qua SPI
uint8_t spi3_transfer(uint8_t data) {
    while (!(SPI3->SR & SPI_SR_TXE)); // Chờ TX buffer trống
    *(volatile uint8_t *)&SPI3->DR = data;

    while (!(SPI3->SR & SPI_SR_RXNE)); // Chờ RX buffer có data
    return *(volatile uint8_t *)&SPI3->DR;
}

// Hàm Delay ngắn cỡ vài chục microsecond
static void delay_us(uint32_t us) {
    uint32_t count = us * (SystemCoreClock / 1000000U) / 5;
    while (count--) {
        __NOP();
    }
}

// Hàm đọc 1 thanh ghi từ PMW3901
uint8_t pmw3901_read_reg(uint8_t reg_addr) {
    uint8_t val = 0;
    // Bit 7 = 0 đại diện cho thao tác READ
    reg_addr &= 0x7F;
    // Kéo CS LOW
    GPIOC->BSRR = GPIO_BSRR_BR3;
    // Gửi địa chỉ thanh ghi
    spi3_transfer(reg_addr);
    // BẮT BUỘC: Delay t_HOLD (~50us) để PMW3901 chuẩn bị dữ liệu ra bus
    delay_us(50);
    // Gửi byte bù để clock ra data
    val = spi3_transfer(0x00);
    GPIOC->BSRR = GPIO_BSRR_BS3;
    delay_us(200);
    return val;
}

// Hàm kiểm tra sensor
volatile uint8_t product_id = 0, revision_id = 1,reverse_product = 0;
void optical_flow_sensor(void) {
    optical_flow_sensor_init();

    // Reset SPI bus state bằng cách nhấp nháy CS
    GPIOC->BSRR = GPIO_BSRR_BR3;
    delay_us(10);
    GPIOC->BSRR = GPIO_BSRR_BS3;
    delay_us(1000);

    product_id = pmw3901_read_reg(0x00);
    revision_id = pmw3901_read_reg(0x01);
    reverse_product = pmw3901_read_reg(0x5F);
}

void pmw3901_write_reg(uint8_t reg_addr, uint8_t data) {
    reg_addr |= 0x80; // Bit 7 = 1 cho thao tác WRITE

    GPIOC->BSRR = GPIO_BSRR_BR3; // CS Low
    spi3_transfer(reg_addr);
    spi3_transfer(data);
    GPIOC->BSRR = GPIO_BSRR_BS3; // CS High

    delay_us(20); // t_SWW / t_SWR delay
}

void pmw3901_init_registers(void) {
    // 1. Reset chip
    pmw3901_write_reg(0x3A, 0x5A);
    HAL_Delay(5);

    // 2. Nạp bảng thông số tối ưu hiệu năng
    uint8_t size = sizeof(pmw3901_init_registers_table) / sizeof(pmw3901_init_registers_table[0]);
    for (uint8_t i = 0; i < size; i++) {
        pmw3901_write_reg(pmw3901_init_registers_table[i][0], pmw3901_init_registers_table[i][1]);
    }

    HAL_Delay(100);

    // 3. Chuỗi thiết lập bổ sung theo thư viện Bitcraze
    pmw3901_write_reg(0x32, 0x44);
    pmw3901_write_reg(0x7F, 0x07);
    pmw3901_write_reg(0x40, 0x40);
    pmw3901_write_reg(0x7F, 0x06);
    pmw3901_write_reg(0x62, 0xF0);
    pmw3901_write_reg(0x63, 0x00);
    pmw3901_write_reg(0x7F, 0x0D);
    pmw3901_write_reg(0x48, 0xC0);
    pmw3901_write_reg(0x6F, 0xD5);
    pmw3901_write_reg(0x7F, 0x00);
    pmw3901_write_reg(0x5B, 0xA0);
    pmw3901_write_reg(0x4E, 0xA8);
    pmw3901_write_reg(0x5A, 0x50);
    pmw3901_write_reg(0x40, 0x80);
}

void pmw3901_read_motion_burst(uint8_t *buffer) {
    GPIOC->BSRR = GPIO_BSRR_BR3; // CS Low
    delay_us(50);

    spi3_transfer(0x16); // Thanh ghi Motion_Burst
    delay_us(150);       // t_SRAD delay

    for (int i = 0; i < 12; i++) {
        buffer[i] = spi3_transfer(0x00);
    }

    delay_us(50);
    GPIOC->BSRR = GPIO_BSRR_BS3; // CS High
    delay_us(200);
}


//   optical_flow_sensor();
//   GPIOC->BSRR = GPIO_BSRR_BR3;
//   delay_us(10);
//   GPIOC->BSRR = GPIO_BSRR_BS3;
//   HAL_Delay(10);

//   product_id      = pmw3901_read_reg(0x00); // Mong muốn: 0x49
//   revision_id     = pmw3901_read_reg(0x01);
//   reverse_product = pmw3901_read_reg(0x5F); // Mong muốn: 0xB6

//   pmw3901_init_registers();

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