#include "RTOS_Init.h"
#include "BMP280.h"
#include "HMC5883/HMC5883.h"
#include "I2C_Progress/I2C.h"
#include "INA226/INA226.h"
#include "Initialize/Init_reg.h"
#include "BMI088_Library/bmi088.h"
#include "PMW3901/pmw3901.h"
#include "VL53L1X.h"

#include "stm32f405xx.h"
#include "vl53l1_api.h"
#include "vl53l1_core.h"
#include "vl53l1_def.h"
#include "vl53l1_error_codes.h"
#include "vl53l1_platform.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/printk.h"
#include <stdbool.h>
#include <sys/_stdint.h>

volatile int i2c1_error_count = 0, i2c3_error_count = 0; 
volatile int bmi_error_init = 0, hmc_error_init = 0, pmw3901_error_init = 0,bmp_error_init = 0, ina_error_init = 0, vl53_error_init = 0;
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
volatile uint32_t pa9_state;
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

        if(drone_i2c_check_address(dev_i2c3, &sensors_addr[3], 4) == 0) break;
        else {
            k_work_submit(&i2c3_error_work);
            if(i2c3_error_count > 50) break;
        }
        k_msleep(10);
    }

    drone_sensor_addr.sensor4 = sensors_addr[3];
    drone_sensor_addr.sensor5 = sensors_addr[4];
    drone_sensor_addr.sensor6 = sensors_addr[5];
    drone_sensor_addr.sensor7 = sensors_addr[6];

    printk("Found all Sensor \n");

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
    k_event_post(&Initial_State_events,BMI088_Ready);

    // HMC5883 has been calibed
    while (1) {
        if(hmc_error_init > 10){
            break;
        }
        if(HMC5883_Initialized() == 0) break;

        k_work_submit(&hmc5883_work);
        k_msleep(10);
    }

    k_event_post(&Initial_State_events, HMC5883_Ready);
    if(hmc_error_init < 10) printk("HMC5883 OK \n");
    else printk("HMC5883 Failed \n");

    // Vl53L1X
    while(1){
        if(vl53_error_init > 10) {
            break;
        }
        if(VL53L1_Init(vl53_dev) == VL53L1_ERROR_NONE){
            VL53L1_StartMeasurement(vl53_dev);
            VL53L1_WrByte(vl53_dev, 0x0046, 0x04);
            break;
        }
        k_work_submit(&vl53_work);
        k_msleep(10);
    }

    if (vl53_error_init < 10) {
        k_sem_reset(&vl53_signal);
        k_event_post(&Initial_State_events, VL53_Ready);
        printk("VL53L1X OK\n");
    }

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

    if(ina_error_init < 10) printk("INA226 OK \n");
    else printk("INA226 Failed \n");

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


    if(bmp_error_init < 10) printk("BMP280 OK \n");
    else printk("BMP280 Failed \n");

        // PMW3901
    while (1) {
        if(pmw3901_error_init > 10){
            // call status
            break;
        }
        optical_identify_id(dev_spi2);
        if(product_id == 0x49 && inverse_product == 0xB6) break;
        k_work_submit(&pmw3901_work);
        k_msleep(15);
    }
    pmw3901_init_registers(dev_spi2);
    if(pmw3901_error_init < 10) printk("PMW3901 OK \n");
    else printk("PMW3901 Failed \n");
    k_event_post(&Initial_State_events, PMW3901_Ready);


    EXTI->IMR |= (1 << 11)|(1 << 10)|(1 << 13)|(1 << 14)|(1 << 9)|(1 << 8)|(1 << 0); // Enable interrupt
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
    k_event_wait(&Initial_State_events,BMI088_Ready,false,K_FOREVER);

    while(1){
        k_sem_take(&bmi088_signal,K_FOREVER);
        if(k_mutex_lock(&i2c3_mutex,K_MSEC(2)) == 0){
            i2c_dma_read_data(dev_i2c3,ACC_ADDR,ACC_Data,acc_data,6, &dma1_stream2_signal);
            i2c_dma_read_data(dev_i2c3,GYRO_ADDR,GYRO_Data,gyro_data,6,&dma1_stream2_signal);
            k_mutex_unlock(&i2c3_mutex);
        }
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
        k_sem_take(&HMC5883_signal, K_FOREVER); // exti 0
        if(k_mutex_lock(&i2c3_mutex, K_MSEC(5)) == 0){
            i2c_dma_read_data(dev_i2c3,HMC5883_ADDR, HMC5883_DATA, hmc_data, sizeof(hmc_data), &dma1_stream2_signal);
            k_mutex_unlock(&i2c3_mutex);
        }
        Cal_The_Direction_Of_Yaw(hmc_data);
    }
}

struct k_work vl53_work;
void vl53_work_handler(struct k_work *work){
    drone_i2c_clearbus(dev_i2c3);
    vl53_error_init++;
}
volatile uint16_t distance_mm = 0;

void VL53_Task(void *p1, void *p2, void *p3){
    VL53L1_RangingMeasurementData_t RangingData;
    k_event_wait(&Initial_State_events, VL53_Ready, false, K_FOREVER);

    while(1){
        k_sem_take(&vl53_signal, K_FOREVER);
        if(k_mutex_lock(&i2c3_mutex, K_FOREVER) == 0){
            if (VL53L1_GetRangingMeasurementData(vl53_dev, &RangingData) == VL53L1_ERROR_NONE) {
                if (RangingData.RangeStatus == 0) {
                    distance_mm = RangingData.RangeMilliMeter;
                }
            }
            VL53L1_WrByte(vl53_dev, 0x0086, 0x01);
            k_mutex_unlock(&i2c3_mutex);
        }
    }
}

struct k_work pmw3901_work;
void pmw3901_work_handler(struct k_work *work){
    pmw3901_error_init++;
    SPI2->CR1 &= ~(1 << 0);
    k_msleep(10);
    SPI2->CR1 |= (1 << 0);
}

void PMW3901_Task(void *p1, void *p2, void *p3) {
    uint8_t pmw3901_buffer[12] = {0};
    k_event_wait(&Initial_State_events, PMW3901_Ready, false, K_FOREVER);

    while(1) {
        k_sem_take(&pmw3901_signal, K_FOREVER);

        if (pmw3901_read_burst_dma(dev_spi2, pmw3901_buffer) == 0) {
            squal = pmw3901_buffer[6];
            delta_x = (int16_t)((pmw3901_buffer[3] << 8) | pmw3901_buffer[2]);
            delta_y = (int16_t)((pmw3901_buffer[5] << 8) | pmw3901_buffer[4]);

            if (delta_x != 0 || delta_y != 0) {
                pos_x += delta_x;
                pos_y += delta_y;
            }
            // Calculate PID position control...
        } else {
            pmw3901_read_reg(dev_spi2, 0x02); 
            k_work_submit(&pmw3901_work);
        }
    }
}

struct k_work bmp280_work;
void bmp280_work_handler(struct k_work *work){
    drone_i2c_clearbus(dev_i2c1);
    bmp_error_init++;
}

volatile float bmp_height = 0;
void BMP280_Task(void *p1, void *p2, void *p3){
    uint8_t bmp_data[6];
    k_event_wait(&Initial_State_events, BMP280_Ready, false, K_FOREVER);
    while (1)
    {
        if(k_mutex_lock(&i2c1_mutex, K_FOREVER) == 0){
            i2c_dma_read_data(dev_i2c1, BMP280_ADDR, BMP280_PRESS, bmp_data, sizeof(bmp_data), &dma1_stream5_signal);
            bmp_height = Calculate_Pressure(bmp_data);
            k_mutex_unlock(&i2c1_mutex);
        }
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
    uint8_t limit_check[2] = {0};
    uint8_t bus_volatage[2] = {0};
    k_event_wait(&Initial_State_events, INA226_Ready, false, K_FOREVER);
    while(1){
        k_sem_take(&ina226_signal, K_FOREVER);
        if(k_mutex_lock(&i2c1_mutex, K_MSEC(10)) == 0){
            i2c_dma_read_data(dev_i2c1,INA226_ADDR, INA226_Alert_limit, limit_check, 2, &dma1_stream5_signal);
            i2c_dma_read_data(dev_i2c1,INA226_ADDR, INA226_Bus_Voltage, bus_volatage, sizeof(bus_volatage), &dma1_stream5_signal);
        }
        k_mutex_unlock(&i2c1_mutex);

        int16_t raw_volatge = (int16_t)(bus_volatage[0] << 8 | bus_volatage[1]);
        check_limit = (limit_check[0] << 8 | limit_check[1]);
        Current_voltage = raw_volatge*0.00125f;
        k_msleep(20);
    }
}

void Radio_Communication(void *p1,void *p2, void *p3){
    while(1){
        k_msleep(10);
    }
}

void Comms_Task(void *p1, void *p2, void *p3){
    while (1) {
        k_msleep(50); // 20 Hz
    }
}


K_THREAD_DEFINE(start_default_id,Default_Thread_Stack_Size,Start_Default_Task,NULL,NULL,NULL,Default_Priority,0,0);
K_THREAD_DEFINE(bmi088_id,BMI088_Thread_Stack_Size,BMI088_Task,NULL,NULL,NULL,BMI088_Priority,0,0);
K_THREAD_DEFINE(pmw3901_id,PMW3901_Thread_Stack_Size,PMW3901_Task,NULL,NULL,NULL,PMW3901_Priority,0,0);
K_THREAD_DEFINE(bmp280_id,BMP280_Thread_Stack_Size,BMP280_Task,NULL,NULL,NULL,BMP280_Priority,0,0);
K_THREAD_DEFINE(hmc5883_id, HMC5883_Thread_Stack_Size, HMC5883_Task, NULL, NULL, NULL, HMC5883_Priority, 0, 0);
K_THREAD_DEFINE(ina226_id, INA226_Thread_Stack_Size, INA226_Task, NULL, NULL, NULL, INA226_Priority, 0,0);
K_THREAD_DEFINE(comms_id, Comms_Thread_Stack_Size, Comms_Task, NULL,NULL,NULL,Comms_Priority,0,0);
K_THREAD_DEFINE(vl53_id, VL53_Thread_Stack_Size,VL53_Task,NULL,NULL,NULL,VL53_Priority,0,0);
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
K_SEM_DEFINE(vl53_signal,0,1);


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
K_WORK_DEFINE(vl53_work, vl53_work_handler);
