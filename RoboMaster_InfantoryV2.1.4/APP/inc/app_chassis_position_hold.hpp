#ifndef __APP_CHASSIS_POSITION_HOLD_HPP__
#define __APP_CHASSIS_POSITION_HOLD_HPP__

#include <stdbool.h>
#include "app_Data_Task.hpp"

/*
 * 速度指令小于该值即认为操作手已松杆。该死区必须同时用于速度误差和
 * 位置保持，否则位置参考已锁定而速度环仍会追逐摇杆回中残余。
 * 4 m/s 满量程下约等于 12 个 SBUS 计数。
 */
#define CHASSIS_POSITION_HOLD_VELOCITY_EPS_M_S  0.010f

/* 松杆后，实际 KF 平均速度低于该值才捕获当前位置作为位置保持参考。
 * 在此之前仅做 Target_X_vel=0 的速度刹车，减速路程不计入 X_pos。 */
#define CHASSIS_POSITION_HOLD_CAPTURE_SPEED_M_S  0.007f

void ChassisPositionHold_Init(float current_position_m);
float ChassisPositionHold_Update(float current_position_m,
                                 float current_velocity_m_s,
                                 float target_velocity_m_s,
                                 bool hold_enable);

/* 将左右 KF 后验统一成 LQR 的纵向三项：位移误差、当前速度、速度误差。
 * Target_X_vel 必须已完成遥控死区处理；本函数不改变目标值。 */
void ChassisLongitudinalState_Update(Classic_Data_t *data,
                                     float left_position_m, float right_position_m,
                                     float left_velocity_m_s, float right_velocity_m_s,
                                     bool position_hold_enable);

#endif
