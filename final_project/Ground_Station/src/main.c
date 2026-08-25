#include <zephyr/kernel.h>
#include <soc.h> // CMSIS Definitions cho STM32F4

struct i2c_dev_t {
    I2C_TypeDef *regs;
    uint32_t speed_hz;
};

static const struct i2c_dev_t i2c3_dev_obj = {
    .regs = I2C3,
    .speed_hz = 100000,
};

const struct i2c_dev_t *dev_i2c3 = &i2c3_dev_obj;

int drone_i2c_init_hw(const struct i2c_dev_t *dev) {
    if (!dev || !dev->regs) return -EINVAL;

    // 1. Enable Clock GPIOA, GPIOC, I2C3
    RCC->AHB1ENR |= (1 << 0) | (1 << 2);
    (void)RCC->AHB1ENR;
    RCC->APB1ENR |= (1 << 23);
    (void)RCC->APB1ENR;

    // 2. Cấu hình GPIO PA8 (SCL) & PC9 (SDA)
    GPIOA->MODER &= ~(3U << (8 * 2));
    GPIOA->MODER |=  (2U << (8 * 2)); // Alternate Function
    GPIOC->MODER &= ~(3U << (9 * 2));
    GPIOC->MODER |=  (2U << (9 * 2)); // Alternate Function

    GPIOA->OTYPER |= (1U << 8); // Open-Drain
    GPIOC->OTYPER |= (1U << 9); // Open-Drain
    
    GPIOA->OSPEEDR &= ~(3U << (8 * 2));
    GPIOA->OSPEEDR |=  (1U << (8 * 2)); // Medium Speed

    GPIOC->OSPEEDR &= ~(3U << (9 * 2));
    GPIOC->OSPEEDR |=  (1U << (9 * 2)); // Medium Speed

    GPIOA->PUPDR &= ~(3U << (8 * 2));                  
    GPIOA->PUPDR |=  (1U << (8 * 2)); // Pull-Up
    GPIOC->PUPDR &= ~(3U << (9 * 2));                  
    GPIOC->PUPDR |=  (1U << (9 * 2)); // Pull-Up

    GPIOA->AFR[1] &= ~(0xFU << ((8 - 8) * 4));
    GPIOA->AFR[1] |=  (4U << ((8 - 8) * 4)); // AF4 (I2C3)
    GPIOC->AFR[1] &= ~(0xFU << ((9 - 8) * 4));
    GPIOC->AFR[1] |=  (4U << ((9 - 8) * 4)); // AF4 (I2C3)

    // 3. Khởi tạo I2C3
    dev->regs->CR1 &= ~(1 << 0); // OFF I2C
    dev->regs->CR2 = 50;         // APB1 = 50MHz
    dev->regs->CCR = 250;        // 100kHz
    dev->regs->TRISE = 51;
    dev->regs->CR1 |= (1 << 0);  // ON I2C

    return 0;
}

volatile uint8_t current_scan_adr = 0;
int drone_i2c_check_address(const struct i2c_dev_t *dev, uint8_t *sensor_out, int nos) {
    if (!dev || !dev->regs) return -EINVAL;
    I2C_TypeDef *i2c = dev->regs;
    int found = 0;

    for (int adr = 1; adr < 127; adr++) { 
        current_scan_adr = adr;
        
        // 1. Nếu Bus bị Busy, ép phát lệnh STOP hoặc Software Reset để giải phóng Bus
        if (i2c->SR2 & (1 << 1)) { // BUSY bit
            i2c->CR1 |= (1 << 15); // SWRST = 1
            k_busy_wait(10);
            i2c->CR1 &= ~(1 << 15); // SWRST = 0
            
            // Re-init I2C3
            i2c->CR1 &= ~(1 << 0);
            i2c->CR2 = 50;  // Giả định APB1 = 50MHz
            i2c->CCR = 250; // 100kHz
            i2c->TRISE = 51;
            i2c->CR1 |= (1 << 0);
        }
        
        // 2. Gửi tín hiệu START
        uint32_t timeout = 2000;
        i2c->CR1 |= (1 << 8); // START = 1
        while (!(i2c->SR1 & (1 << 0)) && --timeout); // Chờ cờ SB (Start Bit)
        if (!timeout) {
            i2c->CR1 |= (1 << 9); // Bắt buộc STOP nếu timeout
            continue;
        }

        // 3. Gửi địa chỉ (Addr + Write)
        i2c->DR = (adr << 1); 
        
        // 4. Chờ cờ ADDR (ACK) hoặc AF (NACK - Acknowledge Failure)
        timeout = 2000;
        while (!(i2c->SR1 & ((1 << 1) | (1 << 10))) && --timeout);

        if (!timeout) {
            i2c->CR1 |= (1 << 9); // Bắt buộc STOP nếu timeout
            continue;
        }

        // 5. Trường hợp NACK (Không có thiết bị tại địa chỉ này)
        if (i2c->SR1 & (1 << 10)) { 
            i2c->SR1 &= ~(1 << 10); // Clear cờ AF bằng cách ghi 0
            i2c->CR1 |= (1 << 9);   // Bắt buộc gửi STOP giải phóng bus
            k_busy_wait(50);
            continue;
        }

        // 6. Trường hợp ACK (Tìm thấy thiết bị)
        if (i2c->SR1 & (1 << 1)) { 
            if (found < nos) {
                sensor_out[found++] = adr;
            }
            // Đọc SR1 theo sau bởi SR2 là quy trình BẮT BUỘC để clear cờ ADDR trong STM32 HW
            volatile uint32_t temp;
            temp = i2c->SR1;
            temp = i2c->SR2;
            (void)temp;

            i2c->CR1 |= (1 << 9); // Gửi STOP kết thúc Frame
        }

        k_msleep(2);
    }

    return (found == nos) ? 0 : -ENODEV;
}

volatile uint8_t addr_found = 0;

int main(void) {
    uint8_t sensor_addr = 0;

    drone_i2c_init_hw(dev_i2c3);
    drone_i2c_check_address(dev_i2c3, &sensor_addr, 1);
    
    addr_found = sensor_addr;

    while (1) {
        k_sleep(K_FOREVER);
    }
    return 0;
}
