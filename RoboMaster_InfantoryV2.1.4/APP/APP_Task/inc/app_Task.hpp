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

//控制事件组位（Data_UPDATE_OK_BIT 已移除：Data Task 合并到 Calculate Task）
#define Calculate_OK_BIT (1 << 0)
#define Control_OK_BIT (1 << 1)

#define ENMEGENCY_POWER_BIT           (1 << 3)  // 紧急停止位

extern TaskHandle_t ControlHandle;
extern TaskHandle_t CaculateHandle;
extern TaskHandle_t DefalutHandle;
extern TaskHandle_t EnmegencyHandle;
extern TaskHandle_t INS_TaskHandle;
extern QueueHandle_t HI14_RxQueueHandle;
extern QueueHandle_t IMU_ControlQueueHandle;
extern QueueHandle_t Remote_ControlQueueHandle;
extern QueueHandle_t Motor_ControlQueueHandle;

/* 成品 IMU / 遥控数据超过 10 ms 未更新时置位；Calculate 统一读取。 */
extern volatile uint8_t imu_data_timeout_flag;
extern volatile uint8_t remote_data_timeout_flag;
void DataTimeout_MarkImuValid(void);
void DataTimeout_MarkRemoteValid(void);


extern EventGroupHandle_t ControlEventGroup;
extern EventGroupHandle_t EnmegencyEventGroup;

#ifdef __cplusplus
extern "C" {
void Task_Init(void);
}
#endif

#endif
