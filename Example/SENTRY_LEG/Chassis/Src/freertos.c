/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "Ins_Task.h"
#include "Ref_Task.h"
#include "Check_Task.h"
#include "Chassis_Task.h"
#include "Super_Cap_Task.h"
#include "Board_Can_Task.h"

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
osThreadId Check_TaskHandle;
osThreadId Ins_TaskHandle;
osThreadId Chassis_TaskHandle;
osThreadId Board_Can_TaskHandle;
osThreadId Ref_TaskHandle;
osThreadId Super_Cap_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Check_Task(void const * argument);
void Start_Ins_Task(void const * argument);
void Start_Chassis_Task(void const * argument);
void Start_Board_Can_Task(void const * argument);
void Start_Ref_Task(void const * argument);
void Start_Super_Cap(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of Check_Task */
  osThreadDef(Check_Task, Start_Check_Task, osPriorityBelowNormal, 0, 256);
  Check_TaskHandle = osThreadCreate(osThread(Check_Task), NULL);

  /* definition and creation of Ins_Task */
  osThreadDef(Ins_Task, Start_Ins_Task, osPriorityRealtime, 0, 1024);
  Ins_TaskHandle = osThreadCreate(osThread(Ins_Task), NULL);

  /* definition and creation of Chassis_Task */
  osThreadDef(Chassis_Task, Start_Chassis_Task, osPriorityHigh, 0, 1024);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* definition and creation of Board_Can_Task */
  osThreadDef(Board_Can_Task, Start_Board_Can_Task, osPriorityNormal, 0, 256);
  Board_Can_TaskHandle = osThreadCreate(osThread(Board_Can_Task), NULL);

  /* definition and creation of Ref_Task */
  osThreadDef(Ref_Task, Start_Ref_Task, osPriorityNormal, 0, 256);
  Ref_TaskHandle = osThreadCreate(osThread(Ref_Task), NULL);

  /* definition and creation of Super_Cap_Task */
  osThreadDef(Super_Cap_Task, Start_Super_Cap, osPriorityAboveNormal, 0, 256);
  Super_Cap_TaskHandle = osThreadCreate(osThread(Super_Cap_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_Check_Task */
/**
  * @brief  Function implementing the Check_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Check_Task */
void Start_Check_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Start_Check_Task */
	
	Check_Init();
	
  /* Infinite loop */
  for(;;)
  {
		
    Check_Task();
		
    osDelay(10);
  }
  /* USER CODE END Start_Check_Task */
}

/* USER CODE BEGIN Header_Start_Ins_Task */
/**
* @brief Function implementing the Ins_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Ins_Task */
void Start_Ins_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Ins_Task */
	
	INS_Init();
	
  /* Infinite loop */
  for(;;)
  {
		
		INS_Task();
		
    osDelay(1);
  }
  /* USER CODE END Start_Ins_Task */
}

/* USER CODE BEGIN Header_Start_Chassis_Task */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Chassis_Task */
void Start_Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Chassis_Task */
	
	Chassis_Init();
	
  /* Infinite loop */
  for(;;)
  {
		
		Chassis_Task();
		
    osDelay(1);
  }
  /* USER CODE END Start_Chassis_Task */
}

/* USER CODE BEGIN Header_Start_Board_Can_Task */
/**
* @brief Function implementing the Board_Can_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Board_Can_Task */
void Start_Board_Can_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Board_Can_Task */
	
	Board_Can_Init();
	
  /* Infinite loop */
  for(;;)
  {
		
		Board_Can_Task();
		
    osDelay(1);
  }
  /* USER CODE END Start_Board_Can_Task */
}

/* USER CODE BEGIN Header_Start_Ref_Task */
/**
* @brief Function implementing the Ref_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Ref_Task */
void Start_Ref_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Ref_Task */
	
	Ref_Init();
	
  /* Infinite loop */
  for(;;)
  {
		Ref_Task();
			
    osDelay(5);
  }
  /* USER CODE END Start_Ref_Task */
}

/* USER CODE BEGIN Header_Start_Super_Cap */
/**
* @brief Function implementing the Super_Cap_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Super_Cap */
void Start_Super_Cap(void const * argument)
{
  /* USER CODE BEGIN Start_Super_Cap */
	
	Super_Cap_Init();
	
  /* Infinite loop */
  for(;;)
  {
		Super_Cap_Task();
		
    osDelay(5);
  }
  /* USER CODE END Start_Super_Cap */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
