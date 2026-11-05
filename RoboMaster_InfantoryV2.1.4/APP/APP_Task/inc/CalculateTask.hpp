#ifndef __CALCULATETASK_HPP
#define __CALCULATETASK_HPP
#include "FreeRTOS.h"
#include "app_Task.hpp"
#include "SBUS.h"
#include "main.h"
#include "app_Data_Task.hpp"

/* 台架测试：单次传感器缺帧不立刻清零髋关节力矩，最多保持上一拍 30 ms。
 * 超过该时间仍按故障安全策略清零。正式版始终使用即时清零。 */
#define LEG_CONTROL_TEST_HOLD_LAST_TORQUE_ON_SENSOR_GAP 1
#define LEG_CONTROL_TEST_SENSOR_HOLD_MS 30U

void CaculateTask(void *pvParameters);
float angle_to_rad(float degrees);
void IMU_Data_Print(Classic_Data_t *pClassicData);
#endif
