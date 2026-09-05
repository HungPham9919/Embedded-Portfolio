#include "INA226.h"
#include "I2C_Progress/I2C.h"
#include "RTOS_Init.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <sys/_stdint.h>

volatile uint16_t INA226_ID = 0;
volatile uint16_t Danger_Voltage = 0;
volatile float Current_voltage = 0;
int INA226_Initialized(void){
    uint8_t ID[2] = {0};

    if(i2c_write_data(dev_i2c1,INA226_ADDR, INA226_Config_Reg, 0x4127, 2) != 0) goto ERR;
    // first state 3.9V/0.00125 = 3120 || 3.6 2880 || 3.4 2720
    Danger_Voltage = 0x0C30; // 3.9V
    // Danger_Voltage = 0x0B40; // 3.6V
    // Danger_Voltage = 0x0AA0; // 3.4V
    if(i2c_write_data(dev_i2c1,INA226_ADDR, INA226_Alert_limit, Danger_Voltage, 2) != 0) goto ERR;;
    // Mask enable BUL bit 12 : Bus voltage under limit
    if(i2c_write_data(dev_i2c1,INA226_ADDR, INA226_Mask_Enable, 0x1001, 2) != 0) goto ERR; // Alert on

    k_msleep(5);
    if(i2c_dma_read_data(dev_i2c1,INA226_ADDR, INA226_DIE_ID, ID, sizeof(ID), &dma1_stream5_signal) != 0) goto ERR;
    INA226_ID = (uint16_t)((ID[0] << 8) | ID[1]);
    k_msleep(5);
    return 0;

ERR:
    printk("INA226 Initialized Error \n");
    return -1;
}
