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

void START_BMI088_TASK(void *argument);
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
  /* USER CODE END RTOS_MUTEX */
	bmi088_mutex = osMutexNew(&bmi088mutex_attributes);
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
  bmi088_thread = osThreadNew(START_BMI088_TASK, NULL, &bmi088task_attributes);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
	EXTI->IMR &= ~(1 << 13) &~(1 << 14);
	BMI088_Initialize();
	GYRO_ACC_LSB(sens.acc_range, sens.gyro_range);
	osDelay(100);
	BMI088_Calib();
//	GPIOC->BSRR = (1 << 16); // LED ON
	osDelay(50);

	drone_angle.Roll_angle = 0.0f;
	drone_angle.Pitch_angle = 0.0f;
	drone_angle.Yaw_angle = 0.0f;
	Setup_PID();

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
//	float dt = 0.005f;
//	uint8_t acc_data[6] = {0};
//	uint8_t gyro_data[6] = {0};

volatile int count = 0, count1 = 0;
void START_BMI088_TASK(void *argument){

	static uint8_t status_test = 0;
	for(;;){
		uint32_t flag = osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		if(flag == 0x0001){
			if(osMutexAcquire(bmi088_mutex, osWaitForever) == osOK){
				count1++;
				Read_Data_DMA(ACC_ADDR, ACC_CHIP_ID,&status_test,1); // i just test the read function
				osMutexRelease(bmi088_mutex);
			}

			if(status_test == 0x1E ){
				count = 1;
			}
			else {
				count = -1;
			}
		}
	}
}

/* USER CODE END Application */

