#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include <stdint.h>
#include <driver/i2c_master.h>
#include <esp_heap_caps.h>
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <esp_log.h>
#include <string.h>

#define I2C_MASTER_SCL_IO 4
#define I2C_MASTER_SDA_IO 5
#define I2C_MASTER_PORT 0

#define I2C1_MASTER_SCL_IO 14
#define I2C1_MASTER_SDA_IO 15
#define I2C_MASTER_FREQ_HZ 400000
#define I2C1_MASTER_PORT 0

extern i2c_master_bus_handle_t i2c1_bus_handle;

esp_err_t i2c1_master_init(void);
void check_address(void);
void i2c1_check_address(void);
#endif