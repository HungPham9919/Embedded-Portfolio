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

#define IMU_INITIALIZED_BIT 1

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

osThreadId_t Error_Thread;
const osThreadAttr_t ErrorTask_attributes = {
		.name = "Error_Task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityRealtime1,
};

osThreadId_t bmi088_thread;
const osThreadAttr_t bmi088task_attributes = {
		.name = "bmi088_task",
		.stack_size = 1024 * 4,
		.priority = (osPriority_t) osPriorityRealtime,
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
	.priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t Radio_Thread;
const osThreadAttr_t Radio_attributes = {
		.name = "Radio Task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t)osPriorityNormal,
};

osEventFlagsId_t xStartupEventFlags;
void START_BMI088_TASK(void *argument);
void RADIO_TASK(void *argument);
void MOTOR_STATE_TASK(void *argument);
void ERROR_TASK(void *argument);

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
volatile int count = 0, count1 = 0;
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	osDelay(50);
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
	  }
	}

	while(1) {
	  if(Check_Address_I2C1() == 1) {
	     break;
	  }
	  else {
	     I2C1_Broken_Task_Handle = osThreadGetId();
	     count++;
	     osThreadFlagsSet(Error_Thread, REQ_RESET_I2C1);
	     osThreadFlagsWait(ERROR_FIXED, osFlagsWaitAny, osWaitForever);
	     osDelay(10);
	     if(count > 100) break;
	  }
	}

	EXTI->IMR &= ~(1 << 13) &~(1 << 14); // off external interrupt
	BMI088_Initialize();
	osDelay(100);
	osEventFlagsSet(xStartupEventFlags, IMU_INITIALIZED_BIT);
	BMI088_Calib();
	osDelay(50);

//	GPIOC->BSRR = (1 << 0);

	uint32_t flags = osEventFlagsWait(xStartupEventFlags, ALL_READY_BIT, osFlagsWaitAll, osWaitForever);
	if(flags == ALL_READY_BIT){
		EXTI->IMR &= ~(1 << 13) &~(1 << 14);

		drone_angle.Roll_angle = 0.0f;
		drone_angle.Pitch_angle = 0.0f;
		drone_angle.Yaw_angle = 0.0f;

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
void ERROR_TASK(void *argument){
	for(;;){
		uint32_t error_flags = osThreadFlagsWait(REQ_RESET_I2C1 | REQ_RESET_I2C3, osFlagsWaitAny, osWaitForever);
		if(error_flags & (REQ_RESET_I2C1)){
			I2C1_ClearBus();
			I2C1_RST_APB();
			I2C1_Initialized();

			if(I2C1_Broken_Task_Handle != NULL){
			   osThreadFlagsSet(I2C1_Broken_Task_Handle, ERROR_FIXED);
			}
		}

		if(error_flags & (REQ_RESET_I2C3)){
			I2C3_ClearBus();
			I2C3_RST_APB();
			I2C3_Initialized();

			if(I2C3_Broken_Task_Handle != NULL){
			   osThreadFlagsSet(I2C3_Broken_Task_Handle, ERROR_FIXED);
			}
		}
	}
}

volatile int bmi_count = 0;
void START_BMI088_TASK(void *argument){
	float dt = 0.005f;
	uint8_t acc_data[6] = {0};
	uint8_t gyro_data[6] = {0};
	GYRO_ACC_LSB(sens.acc_range, sens.gyro_range);

	osEventFlagsWait(xStartupEventFlags, IMU_INITIALIZED_BIT, osFlagsWaitAny, osWaitForever);

	if(status.acc_id == 0x1E){
		osEventFlagsSet(xStartupEventFlags, IMU_BIT);
		bmi_count = 1; // Node
	};

	for(;;){
		uint32_t flag = osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		if(flag == 0x0001){
			if(osMutexAcquire(bmi088_mutex, 2) == osOK){
				BMI088_Read_Data_DMA_With_Retry(GYRO_ADDR, GYRO_Data, gyro_data, 6);
				while(I2C3->SR2 & (1 << 1));
				BMI088_Read_Data_DMA_With_Retry(ACC_ADDR, ACC_Data, acc_data, 6);
				osMutexRelease(bmi088_mutex);
			}
			Calculate_And_Filter_Angle(acc_data, gyro_data, dt);
			Control_PWM();
		}
	}
}

volatile int Radio_count = 0,fly = 0, base = 0;
extern char nRF51822_Data[32];
extern int idx;
extern float base_pwm; // MOTOR POWER
extern int x_pos, y_pos;

void RADIO_TASK(void *argument){
	for(;;){
		osEventFlagsSet(xStartupEventFlags, RADIO_BIT);
		uint32_t Radio_Flag = osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		if(Radio_Flag == 0x0001){

			if(strstr(nRF51822_Data, "STATE") != NULL){
				Radio_count++;
				GPIOC->BSRR = (1 << 0); // LED ON
				USART6_Send_String("OK");
				osEventFlagsSet(xStartupEventFlags, RADIO_BIT);
			}
			else if(strstr(nRF51822_Data, "FLY") != NULL){ 			// Bay len
				GPIOC->BSRR = (1 << 16); // LED OFF
				USART6_Send_String("OK");
				fly++;
			}
			else if(strstr(nRF51822_Data, "BASE") != NULL){			// Nâng độ cao
				base_pwm = converted(nRF51822_Data);
				GPIOC->BSRR = (1 << 0); // LED ON
				USART6_Send_String("OK");
				base++;
			}
			else if(strstr(nRF51822_Data, "X_POS") != NULL){
				x_pos = converted(nRF51822_Data);
				GPIOC->BSRR = (1 << 0); // LED ON
				USART6_Send_String("OK");
			}

			else if(strstr(nRF51822_Data, "Y_POS") != NULL){
				y_pos = converted(nRF51822_Data);
				GPIOC->BSRR = (1 << 0); // LED ON
				USART6_Send_String("OK");
				base++;
			}

			else if(strstr(nRF51822_Data, "LANDING") != NULL){ 		// Hạ Cánh
				GPIOC->BSRR = (1 << 0); // LED ON
				USART6_Send_String("OK");
			}

		    idx = 0;
		    memset(nRF51822_Data, 0, sizeof(nRF51822_Data));
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

