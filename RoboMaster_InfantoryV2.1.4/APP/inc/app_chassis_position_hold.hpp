#ifndef __APP_CHASSIS_POSITION_HOLD_HPP__
#define __APP_CHASSIS_POSITION_HOLD_HPP__

#include <stdbool.h>

/*
 * 速度指令小于该值即认为操作手已松杆。该死区必须同时用于速度误差和
 * 位置保持，否则位置参考已锁定而速度环仍会追逐摇杆回中残余。
 * 4 m/s 满量程下约等于 12 个 SBUS 计数。
 */
#define CHASSIS_POSITION_HOLD_VELOCITY_EPS_M_S  0.060f

void ChassisPositionHold_Init(float current_position_m);
float ChassisPositionHold_Update(float current_position_m,
                                 float target_velocity_m_s,
                                 bool hold_enable);

#endif
