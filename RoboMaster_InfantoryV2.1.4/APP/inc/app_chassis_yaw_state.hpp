#ifndef __APP_CHASSIS_YAW_STATE_HPP__
#define __APP_CHASSIS_YAW_STATE_HPP__

#include <stdbool.h>

#include "app_Data_Task.hpp"

/* 构造 LQR yaw 两个反馈状态。遥控 yaw 指令与 IMU yaw 物理正方向相反，
 * 因此角速度误差按 current - (-target) 计算。 */
void ChassisYawState_Update(Classic_Data_t *data, bool yaw_angle_hold_enable);

#endif
