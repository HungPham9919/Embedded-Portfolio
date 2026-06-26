/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Initialize.h"
#include "bmi088.h"
#include "PWM_Control.h"
#include "event_groups.h"
#include "string.h"
#include "Radio_Communication.h"
#include "battery_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

osThreadId_t Error_Thread;
const osThreadAttr_t ErrorTask_attributes = {
		.name = "Error_Task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityHigh2,
};

osThreadId_t bmi088_thread;
const osThreadAttr_t bmi088task_attributes = {
		.name = "bmi088_task",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityRealtime1,
};

osMutexId_t bmi088_mutex;
const osMutexAttr_t bmi088mutex_attributes = {
		.name = "bmi088_Mutex",
		.attr_bits = osMutexRecursive,
};

osThreadId_t Motor_Thread;
const osThreadAttr_t motor_attributes = {
	.name = "Motor",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityHigh1,
};

osThreadId_t Radio_Thread;
const osThreadAttr_t Radio_attributes = {
		.name = "Radio Task",
		.stack_size = 256 * 4,
		.priority = (osPriority_t)osPriorityHigh,
};

osThreadId_t BatteryReadTaskHandle;
const osThreadAttr_t BatteryReadTask_attributes = {
  .name = "BatteryReadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osEventFlagsId_t xStartupEventFlags;

osMessageQueueId_t radioQueueHandle;

void START_BMI088_TASK(void *argument);
void RADIO_TASK(void *argument);
void MOTOR_STATE_TASK(void *argument);
void ERROR_TASK(void *argument);
void BATTERY_READ(void *argument);

volatile osThreadId_t I2C1_Broken_Task_Handle = NULL;
volatile osThreadId_t I2C3_Broken_Task_Handle = NULL;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

  /* add mutexes, ... */
	bmi088_mutex = osMutexNew(&bmi088mutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	radioQueueHandle = osMessageQueueNew(RADIO_QUEUE_SIZE, RADIO_MSG_LEN, NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  Error_Thread = osThreadNew(ERROR_TASK, NULL, &ErrorTask_attributes);

  bmi088_thread = osThreadNew(START_BMI088_TASK, NULL, &bmi088task_attributes);

  Motor_Thread = osThreadNew(MOTOR_STATE_TASK, NULL, &motor_attributes);

  Radio_Thread = osThreadNew(RADIO_TASK, NULL, &Radio_attributes);

  BatteryReadTaskHandle = osThreadNew(BATTERY_READ, NULL, &BatteryReadTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  xStartupEventFlags = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
volatile int count = 0, count1 = 0,error1 = 0;
volatile uint8_t is_i2c1_available = 1;
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	osDelay(100);
	while(1) {
	  if(Check_Address_I2C3() == 1) {
	     break;
	  }
	  else {
	     I2C3_Broken_Task_Handle = osThreadGetId();
	     count1++;
	     osThreadFlagsSet(Error_Thread, REQ_RESET_I2C3);
	     osThreadFlagsWait(ERROR_FIXED, osFlagsWaitAny, osWaitForever);
	     osDelay(10);
	     if(count1 > 50) break;
	  }
	}

	while(1) {
	  if(Check_Address_I2C1() == 1) {
		  is_i2c1_available = 1;
	     break;
	  }
	  else {
	     I2C1_Broken_Task_Handle = osThreadGetId();
	     count++;
	     osThreadFlagsSet(Error_Thread, REQ_RESET_I2C1);
	     osThreadFlagsWait(ERROR_FIXED, osFlagsWaitAny, 20); // error
		  is_i2c1_available = 0;

	     if(count > 50) {
	    	 osThreadFlagsSet(BatteryReadTaskHandle, BATTERY_ERROR);
	    	 break;
	     }
	     osDelay(10);
	  }
	}

	EXTI->IMR &= ~(1 << 13) &~(1 << 14); // off external interrupt

	osEventFlagsWait(xStartupEventFlags,BMI_BEGIN,osFlagsWaitAny, osWaitForever);

	BMI088_Initialize();
	osDelay(100);

	BMI088_Calib();
	osDelay(50);

	error1++;
	osEventFlagsSet(xStartupEventFlags, IMU_INITIALIZED_BIT);

	if (is_i2c1_available == 1) {
		Bat_initialized();
	}

	osEventFlagsWait(xStartupEventFlags, ALL_READY_BIT, osFlagsWaitAll, osWaitForever);

	EXTI->IMR |= (1 << 13)|(1 << 14);

  /* Infinite loop */
  for(;;)
  {
	 osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
volatile int i2c1_error = 0, i2c3_error = 0;
void ERROR_TASK(void *argument){
	for(;;){
		uint32_t error_flags = osThreadFlagsWait(REQ_RESET_I2C1 | REQ_RESET_I2C3, osFlagsWaitAny, osWaitForever);
		if(error_flags & (REQ_RESET_I2C1)){
			I2C1_ClearBus();
			I2C1_RST_APB();
			I2C1_Initialized();
			i2c1_error++;
			if(I2C1_Broken_Task_Handle != NULL){
			   osThreadFlagsSet(I2C1_Broken_Task_Handle, ERROR_FIXED);
			}
		}

		if(error_flags & (REQ_RESET_I2C3)){
			I2C3_ClearBus();
			I2C3_RST_APB();
			I2C3_Initialized();
			i2c3_error++;
			if(I2C3_Broken_Task_Handle != NULL){
			   osThreadFlagsSet(I2C3_Broken_Task_Handle, ERROR_FIXED);
			}
		}
	}
}

volatile int bmi_count = 0;
float desire_roll = 0, desire_pitch = 0, desire_yaw = 0;

void START_BMI088_TASK(void *argument){
	osEventFlagsClear(xStartupEventFlags, ALL_READY_BIT); // clear all bit -> event
	osThreadFlagsWait(0x0001, osFlagsWaitAny, 0);
	float dt = 0.005f;
	uint8_t acc_data[6] = {0};
	uint8_t gyro_data[6] = {0};
	GYRO_ACC_LSB(sens.acc_range, sens.gyro_range);
	Setup_PID();

	osEventFlagsWait(xStartupEventFlags, IMU_INITIALIZED_BIT, osFlagsWaitAny, osWaitForever);

	if(status.acc_id == 0x1E){
		osEventFlagsSet(xStartupEventFlags, IMU_BIT);
		bmi_count = 1; // Node
	};

	drone_angle.Roll_angle = 0.0f;
	drone_angle.Pitch_angle = 0.0f;
	drone_angle.Yaw_angle = 0.0f;

	GPIOC->BSRR = (1 << 16); // oke

	osThreadFlagsWait(0x0001, osFlagsWaitAny, 0);
	EXTI->PR = (1 << 13) | (1 << 14);

	for(;;){
		uint32_t flag = osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		if(flag == 0x0001){
			if(osMutexAcquire(bmi088_mutex, 2) == osOK){
				BMI088_Read_Data_DMA_With_Retry(GYRO_ADDR, GYRO_Data, gyro_data, 6);
				BMI088_Read_Data_DMA_With_Retry(ACC_ADDR, ACC_Data, acc_data, 6);
				osMutexRelease(bmi088_mutex);
			}
			Calculate_And_Filter_Angle(acc_data, gyro_data, dt);
			Control_PWM(desire_roll,desire_pitch,desire_yaw); // change value -> desired
		}
	}
}

volatile int Radio_count = 0;
extern float base_pwm; // MOTOR POWER
extern int drone_state;
volatile int Landing_Takeoff_State = 0;
float base_compare = 0;

void RADIO_TASK(void *argument){
	char local_radio_data[RADIO_MSG_LEN];
	for(;;){
		osStatus_t status = osMessageQueueGet(radioQueueHandle, local_radio_data, NULL, osWaitForever);
		if(status == osOK){
			if(strstr(local_radio_data, "START") != NULL){
				USART6_Send_String("OK");
				osEventFlagsSet(xStartupEventFlags, BMI_BEGIN); // start to config
				Radio_count++;
				The_First_State(); // MOTOR
			}
			else if(strstr(local_radio_data, "TAKEOFF") != NULL){ 			// Bay len
				osEventFlagsSet(xStartupEventFlags, RADIO_START_BIT); 	// RUN AT THIS TIME
				USART6_Send_String("OK");

				drone_state = 1; // on motor

				Landing_Takeoff_State = 2; // TAKEOFF
				TIM5->CR1 |= (1 << 0); // on timer
				TIM5->CNT = 0;
			}

			else if(strstr(local_radio_data, "BASE") != NULL){
				base_compare = PWM_Converted(local_radio_data);
				if(base_compare > base_pwm){
					Landing_Takeoff_State = 2; // TAKEOFF
					TIM5->CR1 |= (1 << 0); // on timer
					TIM5->CNT = 0;
				}

				USART6_Send_String("OK");
			}

			else if(strstr(local_radio_data, "LANDING") != NULL){ 		// Hạ Cánh
				Landing_Takeoff_State = 1; // LANDING
				TIM5->CR1 |= (1 << 0); // on timer
				TIM5->CNT = 0;
				USART6_Send_String("OK");
			}

			else if(strstr(local_radio_data, "STOP") != NULL){ 		// Pin
				osThreadFlagsSet(Motor_Thread, MOTOR_FLAG_EMERGENCY); // stop
				drone_state = 2;
				USART6_Send_String("OK");
			}

			else if(strstr(local_radio_data, "RPY") != NULL){ // roll pitch yaw - 6 numbers
				desire_roll = Atoi_Converted(local_radio_data)/10000; // 2 so dau
				desire_pitch = (Atoi_Converted(local_radio_data)/100) % 100; // 2 so giua
				desire_yaw = Atoi_Converted(local_radio_data)%100; // 2 so cuoi
				USART6_Send_String("OK");
			}

			else if(strstr(local_radio_data, "PIN") != NULL){ 		// Pin
				osThreadFlagsSet(BatteryReadTaskHandle, 1);
				USART6_Send_String("OK");
			}
		    memset(local_radio_data, 0, sizeof(local_radio_data));
		}
	}
}
volatile int motor_count = 0;

void MOTOR_STATE_TASK(void *argument){
	osThreadFlagsWait(MOTOR_ALL_STATUS, osFlagsWaitAny, 0); // clear flag
	for(;;){
		uint32_t Motor_Flag = osThreadFlagsWait(MOTOR_ALL_STATUS,osFlagsWaitAny, osWaitForever);

		switch (Motor_Flag) {
			case MOTOR_FLAG_STARTUP:
				osEventFlagsSet(xStartupEventFlags, MOTOR_BIT);
				motor_count++;
				break;
			case MOTOR_FLAG_EMERGENCY:
				drone_state = 2; // off motor
				TIM5->CR1 &= ~(1 << 0);
				TIM5->CNT = 0;
				Stop_Motor();
				base_compare = 0;
				break;
			case LANDING:
				if(base_pwm > 200){
					base_pwm -= 8; // 1s -> 125 times
				}
				else {
					// change state of drone
					drone_state = 2; // off
					base_compare = 0; // RESET
					TIM5->CR1 &= ~(1 << 0);
					TIM5->CNT = 0;

					Stop_Motor(); // Đã hạ cánh xong
				}
				break;
			case FLY_FROM_BASE:
				if(base_pwm < 840 || base_pwm < base_compare){
					base_pwm += 25;
				}

				else {
					TIM5->CR1 &= ~(1 << 0);
					TIM5->CNT = 0;

					if (base_pwm >= base_compare && base_compare > 840) {
					     base_pwm = base_compare;
					 }
					else {
					   base_pwm = 840;
					}
				}
				break;
			default:
				break;
		};
	}
}

volatile int bat_count = 0;
volatile float battery_voltage = 0.0f;
void BATTERY_READ(void *argument){
    uint8_t bus_buffer[2] = {0, 0};
    for(;;){
    	uint32_t pin_flag = osThreadFlagsWait((BATTERY|BATTERY_ERROR), osFlagsWaitAny, osWaitForever);
    	if(pin_flag == BATTERY_ERROR){
    		break;
    	}
    	if(pin_flag == BATTERY){
            Bat_Read_Data_With_Retry(INA226_ADDR, INA226_Bus_Voltage, bus_buffer, 2);
            int16_t bus_raw = (int16_t)((bus_buffer[0] << 8) | bus_buffer[1]);
            battery_voltage = (float)bus_raw * 0.00125f;

//            if(battery_voltage < 3.4) {
//            	osThreadFlagsSet(Motor_Thread, LANDING); // LANDING
//            	break;
//            }
    	}
    }
    osThreadTerminate(osThreadGetId()); // Tự xóa chính mình
}
/* USER CODE END Application */

