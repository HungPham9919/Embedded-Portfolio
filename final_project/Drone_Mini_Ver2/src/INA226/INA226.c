#include "INA226.h"
#include "I2C1_Processing_Data.h"
#include "RTOS_Init.h"
#include "zephyr/kernel.h"
#include <sys/_stdint.h>

volatile uint16_t INA226_ID = 0;
volatile uint16_t Danger_Voltage = 0;
volatile float Current_voltage = 0;
int INA226_Initialized(void){
    uint8_t ID[2] = {0};

    I2C1_Write_Data_Safe(INA226_ADDR, INA226_Config_Reg, 0x4127, 2);
    // first state 3.9V/0.00125 = 3120 || 3.6 2880 || 3.4 2720
    Danger_Voltage = 0x0C30; // 3120 DEC
    I2C1_Write_Data_Safe(INA226_ADDR, INA226_Alert_limit, Danger_Voltage, 2);
    // Mask enable BUL bit 12
    I2C1_Write_Data_Safe(INA226_ADDR, INA226_Mask_Enable, 0x1001, 2); // Alert on

    k_msleep(5);
    I2C1_Read_Multiple_Byte_Safe(INA226_ADDR, INA226_DIE_ID, ID, sizeof(ID), &i2c1_error_work);
    INA226_ID = (uint16_t)((ID[0] << 8) | ID[1]);
    k_msleep(5);
    return 1;
}

// ALERT - PC11