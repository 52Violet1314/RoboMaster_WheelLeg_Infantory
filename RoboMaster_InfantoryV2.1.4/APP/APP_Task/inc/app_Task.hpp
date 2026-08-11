#ifndef __APP_TASK_HPP
#define __APP_TASK_HPP

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "SBUS.h"
#include "event_groups.h"
#include "app_motor.h"
#include "app_Data_Task.hpp"

//数据事件组位
#define SBUS_DATA_READY_BIT (1 << 0)
#define MOTOR_Data_READY_BIT (1 << 1)
#define IMU_DATA_READY_BIT      (1 << 2)  // IMU数据就绪

//控制事件组位（Data_UPDATE_OK_BIT 已移除：Data Task 合并到 Calculate Task）
#define Calculate_OK_BIT (1 << 0)
#define Control_OK_BIT (1 << 1)

#define ENMEGENCY_POWER_BIT           (1 << 3)  // 紧急停止位

extern TaskHandle_t ControlHandle;
extern TaskHandle_t CaculateHandle;
extern TaskHandle_t DefalutHandle;
extern TaskHandle_t EnmegencyHandle;
extern TaskHandle_t INS_TaskHandle;


extern QueueHandle_t SBUSQueue;

extern SemaphoreHandle_t IMUSemaphore;

extern EventGroupHandle_t ControlEventGroup;
extern EventGroupHandle_t DataGroup;
extern EventGroupHandle_t EnmegencyEventGroup;

#ifdef __cplusplus
extern "C" {
void Task_Init(void);
}
#endif

#endif
