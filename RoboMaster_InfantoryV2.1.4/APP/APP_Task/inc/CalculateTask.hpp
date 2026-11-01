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

/* 仅台架诊断：只限制气弹簧与腿重力补偿映射出的 T1/T2 变化率。
 * LQR 与腿长 PID 的 VMC 输出不经过此限制，稳态补偿力矩也不改变。 */
#define VMC_TEST_COMPENSATION_TORQUE_SLEW_LIMIT_ENABLE 1
#define VMC_TEST_COMPENSATION_TORQUE_SLEW_RATE_NM_S    50.0f

extern uint8_t ControlState;

void CaculateTask(void *pvParameters);
void VMC_Calculate(Classic_Data_t *data, float F0L, float F0R, float TpL, float TpR, float dt,
                   float spring_f0_l, float spring_f0_r, float gravity_tp_l, float gravity_tp_r);
float angle_to_rad(float degrees);
void Control_Data_Print(Classic_Data_t *pClassicData);
void IMU_Data_Print(Classic_Data_t *pClassicData);
#endif
