#include "vl53l1_platform.h"
#include <zephyr/kernel.h>
#include "I2C.h"

extern const struct device *dev_i2c3;
int i2c_write_multi(const struct device *dev, uint8_t slave_id, uint16_t reg, uint8_t *pdata, uint32_t count) {
    if (!dev || !dev->config) return -EINVAL;
    const struct i2c_dev_t *cfg = dev->config;
    I2C_TypeDef *i2c = cfg->regs;
    volatile uint32_t timeout = 10000;

    // 1. Wait BUSY
    while ((i2c->SR2 & (1 << 1)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    // 2. START
    i2c->CR1 |= (1 << 8); 
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 0)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    // 3. Send Slave Address (Write)
    i2c->DR = (slave_id << 1); 
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 1)) && --timeout) {
        if (i2c->SR1 & (1 << 10)) { 
            i2c->CR1 |= (1 << 9);   
            i2c->SR1 &= ~(1 << 10);
            return -EIO;
        }
    }
    if (timeout == 0) goto OFF_I2C;
    (void)i2c->SR1; 
    (void)i2c->SR2;

    // 4. Send 16-bit Register Address (High Byte trước, Low Byte sau)
    i2c->DR = (uint8_t)(reg >> 8);
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 7)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    i2c->DR = (uint8_t)(reg & 0xFF);
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 7)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    // 5. Transmit Payload Data
    for (uint32_t i = 0; i < count; i++) {
        i2c->DR = pdata[i];
        timeout = 10000;
        while (!(i2c->SR1 & (1 << 7)) && --timeout);
        if (timeout == 0) goto OFF_I2C;
    }

    // 6. Wait BTF & STOP
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 2)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    i2c->CR1 |= (1 << 9); 
    return 0;

OFF_I2C:
    i2c->CR1 |= (1 << 9);
    i2c->CR1 |= (1 << 15);  // SWRST = 1
    k_busy_wait(10);
    i2c->CR1 &= ~(1 << 15); // SWRST = 0
    return -ETIMEDOUT;
}

int i2c_read_multi(const struct device *dev, uint8_t slave_id, uint16_t reg, uint8_t *pdata, uint32_t count) {
    if (!dev || !dev->config) return -EINVAL;
    const struct i2c_dev_t *cfg = dev->config;
    I2C_TypeDef *i2c = cfg->regs;
    volatile uint32_t timeout = 10000;

    // --- Phase 1: Write Reg Address ---
    while ((i2c->SR2 & (1 << 1)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    i2c->CR1 |= (1 << 8); // START
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 0)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    i2c->DR = (slave_id << 1); // Write mode
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 1)) && --timeout);
    if (timeout == 0) goto OFF_I2C;
    (void)i2c->SR1; (void)i2c->SR2;

    // Send 16-bit Reg Address
    i2c->DR = (uint8_t)(reg >> 8);
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 7)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    i2c->DR = (uint8_t)(reg & 0xFF);
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 7)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    // --- Phase 2: Re-START & Read ---
    i2c->CR1 |= (1 << 8); // Re-START
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 0)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    i2c->DR = (slave_id << 1) | 0x01; // Read mode
    timeout = 10000;
    while (!(i2c->SR1 & (1 << 1)) && --timeout);
    if (timeout == 0) goto OFF_I2C;

    i2c->CR1 |= (1 << 10); // Enable ACK
    (void)i2c->SR1; 
    (void)i2c->SR2;

    for (uint32_t i = 0; i < count; i++) {
        if (i == count - 1) {
            i2c->CR1 &= ~(1 << 10); // NACK cho byte cuối
            i2c->CR1 |= (1 << 9);   // STOP
        }
        timeout = 10000;
        while (!(i2c->SR1 & (1 << 6)) && --timeout); // Wait RxNE
        if (timeout == 0) goto OFF_I2C;
        pdata[i] = i2c->DR;
    }

    return 0;

OFF_I2C:
    i2c->CR1 |= (1 << 9);
    return -ETIMEDOUT;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int status = i2c_write_multi(dev_i2c3, Dev->devAddr, index, pdata, count);
    return (status == 0) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    int status = i2c_read_multi(dev_i2c3, Dev->devAddr, index, pdata, count);
    return (status == 0) ? VL53L1_ERROR_NONE : VL53L1_ERROR_CONTROL_INTERFACE;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *pdata) {
    uint8_t buffer[2];
    
    // Gọi hàm ReadMulti để đọc 2 byte dữ liệu từ thanh ghi 16-bit 'index'
    VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buffer, 2);
    
    if (status == VL53L1_ERROR_NONE) {
        // Ghép 2 byte đọc được thành 16-bit word (Big-Endian)
        *pdata = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    }
    
    return status;
}

// 2. Byte / Word wrappers

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms) {
    if (ptick_count_ms == NULL) {
        return VL53L1_ERROR_INVALID_PARAMS;
    }
    *ptick_count_ms = (uint32_t)k_uptime_get_32();
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(Dev, index, &data, 1);
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {
    uint8_t buffer[2] = {(uint8_t)(data >> 8), (uint8_t)(data & 0xFF)};
    return VL53L1_WriteMulti(Dev, index, buffer, 2);
}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata) {
    return VL53L1_ReadMulti(Dev, index, pdata, 1);
}
// 3. Delay & Time Helpers
VL53L1_Error VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms) {
    k_msleep(wait_ms);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us) {
    k_busy_wait(wait_us);
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_DEV Dev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms) {
    uint8_t reg_val = 0;
    uint32_t start_time = k_uptime_get_32();

    while ((k_uptime_get_32() - start_time) < timeout_ms) {
        if (VL53L1_RdByte(Dev, index, &reg_val) == VL53L1_ERROR_NONE) {
            if ((reg_val & mask) == value) {
                return VL53L1_ERROR_NONE;
            }
        }
        k_msleep(poll_delay_ms);
    }
    return VL53L1_ERROR_TIME_OUT;
}
