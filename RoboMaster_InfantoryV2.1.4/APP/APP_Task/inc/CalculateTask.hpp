#ifndef __CALCULATETASK_HPP
#define __CALCULATETASK_HPP
#include "FreeRTOS.h"
#include "app_Task.hpp"
#include "SBUS.h"
#include "main.h"
#include "app_Data_Task.hpp"

/* 1：台架气弹簧测试；0：正式控制。
 * 切换版本只改此处。测试版强制已起身、关闭腿重力补偿与 roll，且关闭
 * 起身前/后 F0 预输出；正式版恢复状态机、roll、预输出，并在总使能且已起身后开腿重力补偿。 */
#define LEG_CONTROL_TEST_MODE 1

/* 仅台架：气弹簧补偿力截断到 0.1 N，并限制每周期最大变化量 [N]。 */
#define LEG_TEST_SPRING_COMPENSATION_QUANTIZATION_STEP_N 0.1f
#define LEG_TEST_SPRING_COMPENSATION_SLEW_STEP_N  0.3f

extern uint8_t ControlState;

void CaculateTask(void *pvParameters);
void VMC_Calculate(Classic_Data_t *data, float F0L, float F0R, float TpL, float TpR, float dt);
float angle_to_rad(float degrees);
void Control_Data_Print(Classic_Data_t *pClassicData);
void IMU_Data_Print(Classic_Data_t *pClassicData);
#endif
