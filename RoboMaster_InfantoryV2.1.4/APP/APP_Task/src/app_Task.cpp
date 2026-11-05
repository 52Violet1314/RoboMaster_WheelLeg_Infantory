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
#include "Hipnuv_hi14.h"
#include <cstdint>

void DefaultTask(void *pvParameters);
void MotorControlTask(void *pvParameters);
void CaculateTask(void *pvParameters);
void CreatTask(void);
void EnmegencyTask(void *pvParameters);


TaskHandle_t ControlHandle;
TaskHandle_t CaculateHandle;
TaskHandle_t INS_TaskHandle;
QueueHandle_t HI14_RxQueueHandle;
QueueHandle_t IMU_ControlQueueHandle;
QueueHandle_t Remote_ControlQueueHandle;
QueueHandle_t Motor_ControlQueueHandle;
TaskHandle_t DefalutHandle;
TaskHandle_t RemoteTaskHandle;

volatile uint8_t imu_data_timeout_flag = 1U;
volatile uint8_t remote_data_timeout_flag = 1U;
static volatile TickType_t imu_last_valid_tick = 0U;
static volatile TickType_t remote_last_valid_tick = 0U;
static StaticTimer_t data_timeout_timer_buffer;
static TimerHandle_t data_timeout_timer = NULL;

#define INPUT_DATA_TIMEOUT_MS 10U

static void data_timeout_timer_callback(TimerHandle_t xTimer) {
  const TickType_t now = xTaskGetTickCount();
  (void)xTimer;

  imu_data_timeout_flag =
      ((imu_last_valid_tick == 0U) ||
       ((now - imu_last_valid_tick) > pdMS_TO_TICKS(INPUT_DATA_TIMEOUT_MS)))
          ? 1U : 0U;
  remote_data_timeout_flag =
      ((remote_last_valid_tick == 0U) ||
       ((now - remote_last_valid_tick) > pdMS_TO_TICKS(INPUT_DATA_TIMEOUT_MS)))
          ? 1U : 0U;
}

void DataTimeout_MarkImuValid(void) {
  imu_last_valid_tick = xTaskGetTickCount();
  imu_data_timeout_flag = 0U;
}

void DataTimeout_MarkRemoteValid(void) {
  remote_last_valid_tick = xTaskGetTickCount();
  remote_data_timeout_flag = 0U;
}



EventGroupHandle_t ControlEventGroup;
EventGroupHandle_t EnmegencyEventGroup;

void CreateEventGroup(void);




void Task_Init(void) {
  CreateEventGroup();
  HI14_RxQueueHandle = xQueueCreate(2, sizeof(Hi14_RxBlock_t));
  if (HI14_RxQueueHandle == NULL) {
    uart_print_blocking("HI14RxQueueCreate fail\r\n");
  }
  IMU_ControlQueueHandle = xQueueCreate(1, sizeof(IMU_ControlSnapshot_t));
  Remote_ControlQueueHandle = xQueueCreate(1, sizeof(Remote_ControlSnapshot_t));
  Motor_ControlQueueHandle = xQueueCreate(1, sizeof(Motor_ControlSnapshot_t));
  if ((IMU_ControlQueueHandle == NULL) ||
      (Remote_ControlQueueHandle == NULL) ||
      (Motor_ControlQueueHandle == NULL)) {
    uart_print_blocking("ControlSnapshotQueueCreate fail\r\n");
  }
  data_timeout_timer = xTimerCreateStatic(
      "InputWd", pdMS_TO_TICKS(1), pdTRUE, NULL,
      data_timeout_timer_callback, &data_timeout_timer_buffer);
  if (data_timeout_timer != NULL) {
    xTimerStart(data_timeout_timer, 0);
  }
  CreatTask();
}

// 默认任务 用于检测任务栈空间是否足够
void DefaultTask(void *pvParameters) {
  uint8_t i = 0;
  while (1) {
    uint32_t RemoteStackHighWaterMark = uxTaskGetStackHighWaterMark(RemoteTaskHandle);
    uint32_t DefalutStackHighWaterMark = uxTaskGetStackHighWaterMark(DefalutHandle);
    (void)RemoteStackHighWaterMark;
    (void)DefalutStackHighWaterMark;
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
  EnmegencyEventGroup = xEventGroupCreate();
  if (EnmegencyEventGroup == NULL) {
    uart_print_blocking("EnmegencyEventGroup creat fail\r\n");
  }
}

