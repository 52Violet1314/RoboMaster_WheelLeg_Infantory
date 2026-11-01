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

/* 仅台架气弹簧测试：F0 速度阻尼 [N/(m/s)]。
 * Fd = -Kd*dL0：腿变长时增加收腿力，腿变短时增加撑腿力。
 * 仅 LEG_CONTROL_TEST_MODE=1 时叠加；正式控制不受影响。 */
#define LEG_TEST_SPRING_F0_DAMPING_KD 20.0f

extern uint8_t ControlState;

void CaculateTask(void *pvParameters);
void VMC_Calculate(Classic_Data_t *data, float F0L, float F0R, float TpL, float TpR, float dt,
                   float spring_f0_l, float spring_f0_r, float gravity_tp_l, float gravity_tp_r);
float angle_to_rad(float degrees);
void Control_Data_Print(Classic_Data_t *pClassicData);
void IMU_Data_Print(Classic_Data_t *pClassicData);
#endif
