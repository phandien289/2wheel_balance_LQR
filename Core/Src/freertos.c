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
#include "encoder.h"
#include "encoder_speed.h"
#include "param.h"
#include "mpu6050.h"
#include "motor.h"
#include "pid.h"
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
MPU6050_t MPU;
float angle,targetSpeed, targetAngle;
int16_t pwm_out;
PIDController pid_vel;
PIDController pid_angle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for encoder */
osThreadId_t encoderHandle;
const osThreadAttr_t encoder_attributes = {
  .name = "encoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for control */
osThreadId_t controlHandle;
const osThreadAttr_t control_attributes = {
  .name = "control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for imu */
osThreadId_t imuHandle;
const osThreadAttr_t imu_attributes = {
  .name = "imu",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Encoder_Task(void *argument);
void Control_Task(void *argument);
void Imu_Task(void *argument);

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

  /* creation of encoder */
  encoderHandle = osThreadNew(Encoder_Task, NULL, &encoder_attributes);

  /* creation of control */
  controlHandle = osThreadNew(Control_Task, NULL, &control_attributes);

  /* creation of imu */
  imuHandle = osThreadNew(Imu_Task, NULL, &imu_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Encoder_Task */
/**
* @brief Function implementing the encoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Encoder_Task */
void Encoder_Task(void *argument)
{
  /* USER CODE BEGIN Encoder_Task */
	const TickType_t period = pdMS_TO_TICKS(5);
	TickType_t lastWake = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  g_speed1_mps = Encoder_GetSpeed_mps(ENCODER_1,DT_TIME);
	  g_speed2_mps = Encoder_GetSpeed_mps(ENCODER_2,DT_TIME);

	  speedAvg = (g_speed1_mps + g_speed2_mps)/2;

	  vTaskDelayUntil(&lastWake, period);
  }
  /* USER CODE END Encoder_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
void Control_Task(void *argument)
{
  /* USER CODE BEGIN Control_Task */
	const TickType_t period = pdMS_TO_TICKS(5);
	TickType_t lastWake = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  MPU6050_Read_All(&hi2c1, &MPU);
	  angle = -(MPU.KalmanAngleY);

	  targetSpeed = 0.0f;
	  targetAngle = PID_Update(&pid_vel, targetSpeed - speedAvg);
	  pwm_out = PID_Update(&pid_angle, targetAngle - angle);

	  Motor_SetSpeed(MOTOR_1, pwm_out);
	  Motor_SetSpeed(MOTOR_2, pwm_out);

	  vTaskDelayUntil(&lastWake, period);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_Imu_Task */
/**
* @brief Function implementing the imu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Imu_Task */
void Imu_Task(void *argument)
{
  /* USER CODE BEGIN Imu_Task */
	const TickType_t period = pdMS_TO_TICKS(5);
	TickType_t lastWake = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {

	  vTaskDelayUntil(&lastWake, period);
  }
  /* USER CODE END Imu_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

