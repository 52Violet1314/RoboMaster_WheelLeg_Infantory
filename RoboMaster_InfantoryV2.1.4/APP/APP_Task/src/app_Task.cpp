#include "app_Task.hpp"
#include "CalculateTask.hpp"
#include "ControlTask.hpp"
#include "app_INSTask.hpp"
#include "app_Data_Task.hpp"
#include "app_Remote_Task.hpp"
#include "FreeRTOS.h"
#include "WS2812.h"
#include "app_main.hpp"
#include "app_motor.h"
#include "event_groups.h"
#include "main.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "bsp_UART.h"
#include "POWER.h"
#include <cstdint>

void DefaultTask(void *pvParameters);
void MotorControlTask(void *pvParameters);
void CaculateTask(void *pvParameters);
void CreatTask(void);
void EnmegencyTask(void *pvParameters);


TaskHandle_t ControlHandle;
TaskHandle_t CaculateHandle;
TaskHandle_t INS_TaskHandle;
TaskHandle_t DefalutHandle;
TaskHandle_t RemoteTaskHandle;



QueueHandle_t SBUSQueue;

EventGroupHandle_t ControlEventGroup;
EventGroupHandle_t DataGroup;
EventGroupHandle_t EnmegencyEventGroup;

SemaphoreHandle_t IMUSemaphore;

void CreatTask(void);
void CreateEventGroup(void);
void CreateQueue(void);
void CreateSemaphore(void);




void Task_Init(void) {
  CreateQueue();
  CreateEventGroup();
  CreateSemaphore();
  CreatTask();
}

// 默认任务 用于检测任务栈空间是否足够
void DefaultTask(void *pvParameters) {
  uint8_t i = 0;
  while (1) {
    uint32_t RemoteStackHighWaterMark = uxTaskGetStackHighWaterMark(RemoteTaskHandle);
    uint32_t DefalutStackHighWaterMark = uxTaskGetStackHighWaterMark(DefalutHandle);
    switch (i) {
    case 0:
      WS2812_Ctrl(0, 0, 0);
      break;
    case 1:
      WS2812_Ctrl(0, 0, 255);
      break;
    case 2:
      WS2812_Ctrl(0, 255, 0);
      break;
    case 3:
      WS2812_Ctrl(0, 255, 255);
      break;
    case 4:
      WS2812_Ctrl(255, 0, 0);
      break;
    case 5:
      WS2812_Ctrl(255, 0, 255);
      break;
    case 6:
      WS2812_Ctrl(255, 255, 0);
      break;
    case 7:
      WS2812_Ctrl(255, 255, 255);
      break;
    default:
      WS2812_Ctrl(0, 0, 0);
      break;
    }
    i++;
    if (i > 7) {
      i = 0;
    }
    /* code */
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
void CreatTask(void) {
  BaseType_t res;
  // BaseType_t xReturned;
  res =xTaskCreate(DefaultTask, "Defalut", 256 * 2, NULL, 2, &DefalutHandle);
  if (res != pdPASS) {
    uart_print_blocking("DefaultTaskCreate fail\r\n");
  }
  res =xTaskCreate(app_INSTask, "INS", 256 * 4, &Classic_Data, 7, &INS_TaskHandle);
  if (res != pdPASS) {
    uart_print_blocking("INSTTaskCreate fail\r\n");
  }
  // Data Task 已合并到 Calculate Task，不再单独创建
  res =xTaskCreate(app_Remote_Task, "Remote", 256 * 4, &Classic_Data, 8, &RemoteTaskHandle);
  if (res != pdPASS) {
    uart_print_blocking("RemoteTaskCreate fail\r\n");
  }
  res = xTaskCreate(MotorControlTask,"Control",256 * 4,&Classic_Data,8,&ControlHandle);
  if (res != pdPASS) {
    uart_print_blocking("ControlTaskCreate fail\r\n");
  }
  res =xTaskCreate(CaculateTask, "Caculate", 256 * 6, &Classic_Data, 9, &CaculateHandle);
  if (res != pdPASS) {
    uart_print_blocking("CalculateTaskCreate fail\r\n");
  }
}

void CreateEventGroup(void)
{
  ControlEventGroup = xEventGroupCreate();
  if (ControlEventGroup == NULL) {
    uart_print_blocking("ControlEventGroup creat fail\r\n");
  }
  DataGroup = xEventGroupCreate();
  if (DataGroup == NULL) {
    uart_print_blocking("DataGroup creat fail\r\n");
  }
  
  EnmegencyEventGroup = xEventGroupCreate();
  if (EnmegencyEventGroup == NULL) {
    uart_print_blocking("EnmegencyEventGroup creat fail\r\n");
  }
}

void CreateQueue(void)
{
    SBUSQueue = xQueueCreate(16, 16 * sizeof(int16_t));
  if (SBUSQueue == NULL) {
    uart_print_blocking("SBUSQueue creat fail\r\n");
  }
}

void CreateSemaphore(void) {
  IMUSemaphore = xSemaphoreCreateBinary();
  if (IMUSemaphore == NULL) {
    uart_print_blocking("IMUSemaphore creat fail\r\n");
  }
}
