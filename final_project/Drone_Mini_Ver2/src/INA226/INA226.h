#ifndef INA226_H
#define INA226_H

#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include "stdio.h"
#include "stdint.h"
#include <sys/_stdint.h>

#define INA226_ADDR 0x40
#define INA226_Config_Reg 0x00
#define INA226_Shunt_Voltage 0x01

#define INA226_Bus_Voltage 0x02
#define INA226_Calibration 0x05
#define INA226_Mask_Enable 0x06
#define INA226_Alert_limit 0x07
#define INA226_DIE_ID 0xFF

extern volatile uint16_t INA226_ID;
extern volatile uint16_t Danger_Voltage;
extern volatile float Current_voltage;
int INA226_Initialized(void);

#endif
