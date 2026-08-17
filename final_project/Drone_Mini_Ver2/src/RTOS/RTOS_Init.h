#ifndef RTOS_INIT_H
#define RTOS_INIT_H

#include <zephyr/devicetree.h>
#include "zephyr/kernel.h"

#define Default_Thread_Stack_Size 1024

#define BMI088_Thread_Stack_Size 1024
#define HMC5883_Thread_Stack_Size 512
#define VL53_Thread_Stack_Size 1024

#define PMW3901_Thread_Stack_Size 1024
#define BMP280_Thread_Stack_Size 512
#define INA226_Thread_Stack_Size 512

#define BMI088_Priority 3
#define PMW3901_Priority 3
#define HMC5883_Priority 4
#define INA226_Priority 3
#define BMP280_Priority 5 
#define Default_Priority 6

#define BMI088_Ready (1 << 0)
#define BMI088_Failed (1 << 1)

#define HMC5883_Ready (1 << 2)
#define HMC5883_Failed (1 << 3)

#define BMP280_Ready (1 << 4)
#define BMP280_Failed (1 << 5)

#define INA226_Ready (1 << 6)
#define INA226_Failed (1 << 7)

#define AT24_Flag (1 << 8)

#define PMW3901_Ready (1 << 9)
#define PMW3901_Failed (1 << 10)

#define Motor_Flag (1 << 11)
#define Radio_Flag (1 << 12)

extern struct k_mutex i2c1_mutex;
extern struct k_mutex i2c3_mutex;

extern struct k_sem bmi088_signal;
extern struct k_sem HMC5883_signal;
extern struct k_sem pmw3901_signal;
extern struct k_sem radio_signal;
extern struct k_sem bmp280_signal;
extern struct k_sem ina226_signal;

extern struct k_sem dma1_stream2_signal;
extern struct k_sem dma1_stream3_signal;
extern struct k_sem dma1_stream4_signal;
extern struct k_sem dma1_stream5_signal;
extern struct k_sem dma1_stream6_signal;

extern struct k_work i2c3_error_work;
extern struct k_work i2c1_error_work;
extern struct k_work bmi088_work;
extern struct k_work ina226_work;
extern struct k_work bmp280_work;
extern struct k_work hmc5883_work;
extern struct k_work pmw3901_work;

// tính stack size

// #include <zephyr/kernel.h>

// void print_my_stack_usage(void) {
//     size_t unused_space;
    
//     // Lấy dung lượng stack chưa dùng đến của Thread hiện tại
//     k_thread_stack_space_get(k_current_get(), &unused_space);
    
//     printk("Unused stack space: %size_t bytes\n", unused_space);
// }

#endif
