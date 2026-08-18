#include "RTOS_Init.h"
#include "BMP280.h"
#include "HMC5883/HMC5883.h"
#include "I2C_Progress/I2C.h"
#include "INA226/INA226.h"
#include "Initialize/Init_reg.h"
#include "BMI088_Library/bmi088.h"
#include "PMW3901/pmw3901.h"
#include "stm32f405xx.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <stdbool.h>
#include <sys/_stdint.h>

volatile int i2c1_error_count = 0, i2c3_error_count = 0; 
volatile int bmi_error_init = 0, hmc_error_init = 0, pmw3901_error_init = 0,bmp_error_init = 0, ina_error_init = 0;
K_EVENT_DEFINE(Initial_State_events);

struct k_work i2c1_error_work;
void i2c1_error_work_handler(struct k_work *work){
    drone_i2c_clearbus(dev_i2c1);
    i2c1_error_count++;
}

struct k_work i2c3_error_work;
void i2c3_error_work_handler(struct k_work *work){
    drone_i2c_clearbus(dev_i2c3);
    i2c3_error_count++;
}

void Start_Default_Task(void *p1, void *p2, void *p3){
    uint8_t sensors_addr[7] = {0};
    while(1){

        if(drone_i2c_check_address(dev_i2c1, sensors_addr, 3) == 0) break;
        else {
            k_work_submit(&i2c1_error_work);
            if(i2c1_error_count > 50) break;
        }
        k_msleep(10);
    }

    drone_sensor_addr.sensor1 = sensors_addr[0];
    drone_sensor_addr.sensor2 = sensors_addr[1];
    drone_sensor_addr.sensor3 = sensors_addr[2];

    while(1){

        if(drone_i2c_check_address(dev_i2c3, &sensors_addr[3], 3) == 0) break;
        else {
            k_work_submit(&i2c3_error_work);
            if(i2c3_error_count > 50) break;
        }
        k_msleep(10);
    }

    drone_sensor_addr.sensor4 = sensors_addr[3];
    drone_sensor_addr.sensor5 = sensors_addr[4];
    drone_sensor_addr.sensor6 = sensors_addr[5];

    printk("Found all Sensor \n");

    // Initialization of sensor 

    // BMI088
    while (1) {
        if(bmi_error_init > 10){
            // REST MCU - call radio
            break;
        }
        if(BMI088_Initialize() == 0 && BMI088_Calib() == 0) break;
        k_work_submit(&bmi088_work);
        k_msleep(10);
    }
    printk("BMI088 OK \n");
    k_event_post(&Initial_State_events,BMI088_Ready); // BMI088
    // HMC5883
    while (1) {
        if(hmc_error_init > 10){
            break;
        }
        if(HMC5883_Initialized() == 0 && HMC5883_Calibration() == 0) break;

        k_work_submit(&hmc5883_work);
        k_msleep(5);
    }
    k_event_post(&Initial_State_events, HMC5883_Ready);
    
    // Vl53L1X

    
    // PMW3901
    // while (1) {
    //     if(pmw3901_error_init > 10){
    //         // call status
    //         break;
    //     }
    //     optical_flow_sensor();
    //     if(product_id != 0x49 || revision_id != 0x00 || inverse_product != 0xB6){
    //         k_work_submit(&pmw3901_work);
    //     }
    //     k_msleep(5);
    // }

    // pmw3901_init_registers();
    // k_event_post(&Initial_State_events, PMW3901_Ready);

    // INA226
    while(1){
        if(ina_error_init > 10) {
            break;
        }
        if(INA226_Initialized() == 0) break;
        k_work_submit(&ina226_work);
        k_msleep(5);
    }
    k_event_post(&Initial_State_events, INA226_Ready);

    // BMP280
    while (1) {
        if(bmp_error_init > 10){
            break;
        }
        if(BMP280_Initialized() == 0 && BMP280_Calibration() == 0) break;
        k_work_submit(&bmp280_work);
        k_msleep(5);
    }

    k_event_post(&Initial_State_events, BMP280_Ready);
    EXTI->IMR |= (1 << 10)|(1 << 13)|(1 << 14); // Enable interrupt
    GPIOC->BSRR = (1 << 1); // on led
    // AT24LC
    k_thread_abort(k_current_get());
}

struct k_work bmi088_work;
void bmi088_work_handler(struct k_work *work){
    drone_i2c_clearbus(dev_i2c3);
    bmi_error_init++;
}

void BMI088_Task(void *p1, void *p2, void *p3){     // 200Hz
    uint8_t acc_data[6] = {0};
    uint8_t gyro_data[6] = {0};
    float dt = 0.005f;
    GYRO_ACC_LSB(sens.acc_range, sens.gyro_range);
    // wait
    k_event_wait(&Initial_State_events,BMI088_Ready,false,K_FOREVER);

    while(1){
        k_sem_take(&bmi088_signal,K_FOREVER);
        if(k_mutex_lock(&i2c3_mutex,K_MSEC(1)) == 0){
            i2c_dma_read_data(dev_i2c3,ACC_ADDR,ACC_Data,acc_data,6, &dma1_stream2_signal);
            i2c_dma_read_data(dev_i2c3,GYRO_ADDR,GYRO_Data,gyro_data,6,&dma1_stream2_signal);
        }
        k_mutex_unlock(&i2c3_mutex);
        Calculate_And_Filter_Angle(acc_data,gyro_data,dt);

        // cal PID
    }
}

struct k_work hmc5883_work;
void hmc5883_work_handler(struct k_work *work){
    drone_i2c_clearbus(dev_i2c3);
    hmc_error_init++;
}

void HMC5883_Task(void *p1, void *p2, void *p3){
    uint8_t hmc_data[6] = {0};
    while (1) {
        k_sem_take(&HMC5883_signal, K_FOREVER);
        if(k_mutex_lock(&i2c3_mutex, K_MSEC(1))){
            i2c_dma_read_data(dev_i2c3,HMC5883_ADDR, HMC5883_DATA, hmc_data, sizeof(hmc_data), &dma1_stream2_signal);
        }
        k_mutex_unlock(&i2c3_mutex);
        Cal_The_Direction_Of_Yaw(hmc_data);
    }
}

struct k_work pmw3901_work;
void pmw3901_work_handler(struct k_work *work){
    pmw3901_error_init++;
    SPI2->CR1 &= ~(1 << 0);
    k_msleep(10);
    SPI2->CR1 |= (1 << 0);
}

void PMW3901_Task(void *p1, void *p2, void *p3){ // SPI
    uint8_t pmw3901_buffer[12] = {0};
    k_event_wait(&Initial_State_events, PMW3901_Ready, false, K_FOREVER);

    while(1){
        k_sem_take(&pmw3901_signal,K_FOREVER);
        SPI2_DMA_Transfer(pmw3901_buffer, sizeof(pmw3901_buffer));
    }

    // cal PID
}

struct k_work bmp280_work;

void bmp280_work_handler(struct k_work *work){
    drone_i2c_clearbus(dev_i2c1);
    bmp_error_init++;
}

volatile int bmptest = 0;
void BMP280_Task(void *p1, void *p2, void *p3){
    k_event_wait(&Initial_State_events, BMP280_Ready, false, K_FOREVER);
    while (1)
    {
        k_msleep(100);
    }
}

struct k_work ina226_work;
void ina226_work_handler(struct k_work *work){
    drone_i2c_clearbus(dev_i2c1);
    ina_error_init++;
}

volatile int16_t check_limit = 0;
void INA226_Task(void *p1, void *p2, void *p3){
    k_event_wait(&Initial_State_events, INA226_Ready, false, K_FOREVER);
    uint8_t limit_check[2] = {0};
    uint8_t bus_volatage[2] = {0};
    while(1){
        k_sem_take(&ina226_signal, K_FOREVER);
        i2c_dma_read_data(dev_i2c1,INA226_ADDR, INA226_Alert_limit, limit_check, 2, &dma1_stream5_signal);
        i2c_dma_read_data(dev_i2c1,INA226_ADDR, INA226_Bus_Voltage, bus_volatage, sizeof(bus_volatage), &dma1_stream5_signal);
        int16_t raw_volatge = (int16_t)(bus_volatage[0] << 8 | bus_volatage[1]);
        check_limit = (limit_check[0] << 8 | limit_check[1]);
        Current_voltage = raw_volatge*0.00125f;
        // if(Current_voltage < Danger_Voltage){
        //     // Radio signal
        // }
        k_msleep(20);
    }
}

void Radio_Communication(void *p1,void *p2, void *p3){
    while(1){
        k_msleep(10);
    }
}


K_THREAD_DEFINE(start_default_id,Default_Thread_Stack_Size,Start_Default_Task,NULL,NULL,NULL,Default_Priority,0,0);
K_THREAD_DEFINE(bmi088_id,BMI088_Thread_Stack_Size,BMI088_Task,NULL,NULL,NULL,BMI088_Priority,0,0);
K_THREAD_DEFINE(pmw3901_id,PMW3901_Thread_Stack_Size,PMW3901_Task,NULL,NULL,NULL,PMW3901_Priority,0,0);
K_THREAD_DEFINE(bmp280_id,BMP280_Thread_Stack_Size,BMP280_Task,NULL,NULL,NULL,BMP280_Priority,0,0);
K_THREAD_DEFINE(hmc5883_id, HMC5883_Thread_Stack_Size, HMC5883_Task, NULL, NULL, NULL, HMC5883_Priority, 0, 0);
K_THREAD_DEFINE(ina226_id, INA226_Thread_Stack_Size, INA226_Task, NULL, NULL, NULL, INA226_Priority, 0,0);
// Mutex

K_MUTEX_DEFINE(i2c3_mutex);
K_MUTEX_DEFINE(i2c1_mutex);

// Semaphore

K_SEM_DEFINE(bmi088_signal,0,1);
K_SEM_DEFINE(HMC5883_signal,0,1);
K_SEM_DEFINE(pmw3901_signal,0,1);
K_SEM_DEFINE(radio_signal,0,1);
K_SEM_DEFINE(bmp280_signal,0,1);
K_SEM_DEFINE(ina226_signal,0,1);


K_SEM_DEFINE(dma1_stream2_signal,0,1);
K_SEM_DEFINE(dma1_stream3_signal,0,1);
K_SEM_DEFINE(dma1_stream4_signal,0,1);
K_SEM_DEFINE(dma1_stream5_signal,0,1);
K_SEM_DEFINE(dma1_stream6_signal,0,1);
// k_work
K_WORK_DEFINE(i2c1_error_work, i2c1_error_work_handler);
K_WORK_DEFINE(i2c3_error_work, i2c3_error_work_handler);
K_WORK_DEFINE(bmi088_work, bmi088_work_handler);
K_WORK_DEFINE(ina226_work, ina226_work_handler);
K_WORK_DEFINE(bmp280_work,bmp280_work_handler);
K_WORK_DEFINE(hmc5883_work, hmc5883_work_handler);
K_WORK_DEFINE(pmw3901_work, pmw3901_work_handler);
