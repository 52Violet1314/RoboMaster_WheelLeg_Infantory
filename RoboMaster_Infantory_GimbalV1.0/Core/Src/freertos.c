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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Task.h"
#include "Interrupt.h"
#include "Imu.h"
#include "Remote.h"
#include "Vision.h"
#include "Refree.h"
#include "Can_Motor.h"
#include "Mode.h"
#include "Debug.h"

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
/* Definitions for Debug_Task */
osThreadId_t Debug_TaskHandle;
const osThreadAttr_t Debug_Task_attributes = {
  .name = "Debug_Task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for InputTask */
osThreadId_t InputTaskHandle;
const osThreadAttr_t InputTask_attributes = {
  .name = "InputTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for CanTask */
osThreadId_t CanTaskHandle;
const osThreadAttr_t CanTask_attributes = {
  .name = "CanTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ModeTask */
osThreadId_t ModeTaskHandle;
const osThreadAttr_t ModeTask_attributes = {
  .name = "ModeTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for Gimbal_Task */
osThreadId_t Gimbal_TaskHandle;
const osThreadAttr_t Gimbal_Task_attributes = {
  .name = "Gimbal_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for IMU_Queue */
osMessageQueueId_t IMU_QueueHandle;
const osMessageQueueAttr_t IMU_Queue_attributes = {
  .name = "IMU_Queue"
};
/* Definitions for Gimbal_Imu_Queue */
osMessageQueueId_t Gimbal_Imu_QueueHandle;
const osMessageQueueAttr_t Gimbal_Imu_Queue_attributes = {
  .name = "Gimbal_Imu_Queue"
};
/* Definitions for Mode_Remote_Queue */
osMessageQueueId_t Mode_Remote_QueueHandle;
const osMessageQueueAttr_t Mode_Remote_Queue_attributes = {
  .name = "Mode_Remote_Queue"
};
/* Definitions for Referee_Data_Queue */
osMessageQueueId_t Referee_Data_QueueHandle;
const osMessageQueueAttr_t Referee_Data_Queue_attributes = {
  .name = "Referee_Data_Queue"
};
/* Definitions for Mode_Vision_Queue */
osMessageQueueId_t Mode_Vision_QueueHandle;
const osMessageQueueAttr_t Mode_Vision_Queue_attributes = {
  .name = "Mode_Vision_Queue"
};
/* Definitions for Mode_Referee_Queue */
osMessageQueueId_t Mode_Referee_QueueHandle;
const osMessageQueueAttr_t Mode_Referee_Queue_attributes = {
  .name = "Mode_Referee_Queue"
};
/* Definitions for Gimbal_Control_Queue */
osMessageQueueId_t Gimbal_Control_QueueHandle;
const osMessageQueueAttr_t Gimbal_Control_Queue_attributes = {
  .name = "Gimbal_Control_Queue"
};
/* Definitions for Can_Motor_Rx_Queue */
osMessageQueueId_t Can_Motor_Rx_QueueHandle;
const osMessageQueueAttr_t Can_Motor_Rx_Queue_attributes = {
  .name = "Can_Motor_Rx_Queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Debug_Task_Handle(void *argument);
void IMU_Task_Handle(void *argument);
void InputTask_Handle(void *argument);
void CanTask_Handle(void *argument);
void ModeTask_Handle(void *argument);
void Gimbal_Task_Handle(void *argument);

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

  /* Create the queue(s) */
  /* creation of IMU_Queue */
  IMU_QueueHandle = osMessageQueueNew (10, sizeof(Imu_Rx_Block_t), &IMU_Queue_attributes);

  /* creation of Gimbal_Imu_Queue */
  Gimbal_Imu_QueueHandle = osMessageQueueNew (1, sizeof(Imu_Pitch_Packet_t), &Gimbal_Imu_Queue_attributes);

  /* creation of Mode_Remote_Queue */
  Mode_Remote_QueueHandle = osMessageQueueNew (1, sizeof(Remote_Mode_Input_t), &Mode_Remote_Queue_attributes);

  /* creation of Referee_Data_Queue */
  Referee_Data_QueueHandle = osMessageQueueNew (10, sizeof(Referee_Event_t), &Referee_Data_Queue_attributes);

  /* creation of Mode_Vision_Queue */
  Mode_Vision_QueueHandle = osMessageQueueNew (1, sizeof(VisionToGimbal_t), &Mode_Vision_Queue_attributes);

  /* creation of Mode_Referee_Queue */
  Mode_Referee_QueueHandle = osMessageQueueNew (1, sizeof(Referee_VtmInput_t), &Mode_Referee_Queue_attributes);

  /* creation of Gimbal_Control_Queue */
  Gimbal_Control_QueueHandle = osMessageQueueNew (1, sizeof(Mode_Control_Command_t), &Gimbal_Control_Queue_attributes);

  /* creation of Can_Motor_Rx_Queue */
  Can_Motor_Rx_QueueHandle = osMessageQueueNew (1, sizeof(Can_Motor_Rx_Snapshot_t), &Can_Motor_Rx_Queue_attributes);

  /* Create the thread(s) */
  /* creation of Debug_Task */
  Debug_TaskHandle = osThreadNew(Debug_Task_Handle, NULL, &Debug_Task_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(IMU_Task_Handle, NULL, &IMU_Task_attributes);

  /* creation of InputTask */
  InputTaskHandle = osThreadNew(InputTask_Handle, NULL, &InputTask_attributes);

  /* creation of CanTask */
  CanTaskHandle = osThreadNew(CanTask_Handle, NULL, &CanTask_attributes);

  /* creation of ModeTask */
  ModeTaskHandle = osThreadNew(ModeTask_Handle, NULL, &ModeTask_attributes);

  /* creation of Gimbal_Task */
  Gimbal_TaskHandle = osThreadNew(Gimbal_Task_Handle, NULL, &Gimbal_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Debug_Task_Handle */
/**
  * @brief  Function implementing the Debug_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Debug_Task_Handle */
void Debug_Task_Handle(void *argument)
{
  /* USER CODE BEGIN Debug_Task_Handle */
  Task_DebugTask(argument);
  /* USER CODE END Debug_Task_Handle */
}

/* USER CODE BEGIN Header_IMU_Task_Handle */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task_Handle */
void IMU_Task_Handle(void *argument)
{
  /* USER CODE BEGIN IMU_Task_Handle */
  /* Infinite loop */
  Task_IMU_Task(argument);
  /* USER CODE END IMU_Task_Handle */
}

/* USER CODE BEGIN Header_InputTask_Handle */
/**
* @brief Function implementing the InputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InputTask_Handle */
void InputTask_Handle(void *argument)
{
  /* USER CODE BEGIN InputTask_Handle */
  /* Infinite loop */
  Task_InputTask(argument);
  /* USER CODE END InputTask_Handle */
}

/* USER CODE BEGIN Header_CanTask_Handle */
/**
* @brief Function implementing the CanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanTask_Handle */
void CanTask_Handle(void *argument)
{
  /* USER CODE BEGIN CanTask_Handle */
  /* Infinite loop */
  Task_CanTask(argument);
  /* USER CODE END CanTask_Handle */
}

/* USER CODE BEGIN Header_ModeTask_Handle */
/**
* @brief Function implementing the ModeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModeTask_Handle */
void ModeTask_Handle(void *argument)
{
  /* USER CODE BEGIN ModeTask_Handle */
  /* Infinite loop */
  Task_ModeTask(argument);
  /* USER CODE END ModeTask_Handle */
}

/* USER CODE BEGIN Header_Gimbal_Task_Handle */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task_Handle */
void Gimbal_Task_Handle(void *argument)
{
  /* USER CODE BEGIN Gimbal_Task_Handle */
  /* Infinite loop */
  Task_Gimbal_Task(argument);
  /* USER CODE END Gimbal_Task_Handle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

