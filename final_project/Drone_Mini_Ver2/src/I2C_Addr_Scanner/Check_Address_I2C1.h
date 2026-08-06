#include "zephyr/devicetree.h"
#include "zephyr/kernel.h"
#include "stdio.h"

typedef struct {
    uint8_t sensor1,sensor2,sensor3,sensor4,sensor5,sensor6,sensor7;
}DRONE_SENSOR;

extern volatile DRONE_SENSOR drone_sensor_addr;

void I2C1_Initialized(void);
void I2C1_RST_APB(void);
void I2C1_ClearBus(void);

void I2C3_Initialized(void);
void I2C3_RST_APB(void);
void I2C3_ClearBus(void);

int Check_Address_I2C3(void);
int Check_Address_I2C1(void);