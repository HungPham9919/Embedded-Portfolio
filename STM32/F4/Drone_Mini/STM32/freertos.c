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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_BIT (1 << 0)
#define RADIO_BIT (1 << 1)
#define MOTOR_BIT (1 << 2)
#define ALL_READY_BIT ((1 << 0)|(1 << 1)|(1 << 2))

#define MOTOR_FLAG_STARTUP (1 << 0)
#define MOTOR_FLAG_EMERGENCY (1 << 1)
#define MOTOR_ALL_STATUS ((1 << 0)|(1 << 1))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef struct {
	uint8_t Channels[8];
}Radio_cmd_t;

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
osThreadId_t bmi088_thread;
const osThreadAttr_t bmi088task_attributes = {
		.name = "bmi088_task",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityHigh5,
};

osMutexId_t bmi088_mutex;
const osMutexAttr_t bmi088mutex_attributes = {
		.name = "bmi088_Mutex",
		.attr_bits = osMutexRecursive,
};

osMessageQueueId_t Radio_queue;
const osMessageQueueAttr_t Radio_Queue_attributes = {
		.name = "Radio Communication",
};

osThreadId_t Motor_Thread;
const osThreadAttr_t motor_attributes = {
	.name = "Motor",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t Radio_Thread;
const osThreadAttr_t Radio_attributes = {
		.name = "Radio Task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t)osPriorityAboveNormal2,
};

osEventFlagsId_t xStartupEventFlags;
void START_BMI088_TASK(void *argument);
void RADIO_TASK(void *argument);
void MOTOR_STATE_TASK(void *argument);
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
	Radio_queue = osMessageQueueNew(10, sizeof(Radio_cmd_t),&Radio_Queue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  bmi088_thread = osThreadNew(START_BMI088_TASK, NULL, &bmi088task_attributes);
  Motor_Thread = osThreadNew(MOTOR_STATE_TASK, NULL, &motor_attributes);
  Radio_Thread = osThreadNew(RADIO_TASK, NULL, &Radio_attributes);
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

	uint32_t flags = osEventFlagsWait(xStartupEventFlags, ALL_READY_BIT, osFlagsWaitAll, osWaitForever);
	if(flags == ALL_READY_BIT){
		EXTI->IMR &= ~(1 << 13) &~(1 << 14);

		drone_angle.Roll_angle = 0.0f;
		drone_angle.Pitch_angle = 0.0f;
		drone_angle.Yaw_angle = 0.0f;
		GYRO_ACC_LSB(sens.acc_range, sens.gyro_range);
		Setup_PID();

		EXTI->IMR |= (1 << 13)|(1 << 14);
	}
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
volatile int bmi_count = 0;
void START_BMI088_TASK(void *argument){

	EXTI->IMR &= ~(1 << 13) &~(1 << 14); // off external interrupt
	BMI088_Initialize();
	osDelay(100);
	BMI088_Calib();
	osDelay(50);

	float dt = 0.005f;
	uint8_t acc_data[6] = {0};
	uint8_t gyro_data[6] = {0};

	if(status.acc_id == 0x1E){
		osEventFlagsSet(xStartupEventFlags, IMU_BIT);
		bmi_count = 1;
	};

	for(;;){
		uint32_t flag = osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		if(flag == 0x0001){
			if(osMutexAcquire(bmi088_mutex, 2) == osOK){
				Read_Data_DMA(GYRO_ADDR, GYRO_Data, gyro_data, 6);
				while(I2C3->SR2 & (1 << 1));
				Read_Data_DMA(ACC_ADDR, ACC_Data, acc_data, 6);
				osMutexRelease(bmi088_mutex);
			}
			Calculate_And_Filter_Angle(acc_data, gyro_data, dt);
			Control_PWM();
		}
	}
}

volatile int Radio_count = 0;
void RADIO_TASK(void *argument){
	osDelay(1000);
	osEventFlagsSet(xStartupEventFlags, RADIO_BIT);
	Radio_count = 1;
	for(;;){
		uint32_t Radio_Flag = osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		if(Radio_Flag == 0x0001){
			osEventFlagsSet(xStartupEventFlags, RADIO_BIT);
		}
	}
}

volatile int motor_count = 0;

void MOTOR_STATE_TASK(void *argument){
	The_First_State();
	for(;;){
		uint32_t Motor_Flag = osThreadFlagsWait(MOTOR_ALL_STATUS,osFlagsWaitAny, osWaitForever);
		switch (Motor_Flag) {
			case MOTOR_FLAG_STARTUP:
				osEventFlagsSet(xStartupEventFlags, MOTOR_BIT);
				motor_count = 1;
				break;
			case MOTOR_FLAG_EMERGENCY:
				Stop_Motor();
				motor_count = 2;
			default:
				break;
		};
	}
}
/* USER CODE END Application */

