#ifndef __APP_CHASSIS_POSITION_HOLD_HPP__
#define __APP_CHASSIS_POSITION_HOLD_HPP__

#include <stdbool.h>

#define CHASSIS_POSITION_HOLD_VELOCITY_EPS_M_S  0.001f

void ChassisPositionHold_Init(float current_position_m);
float ChassisPositionHold_Update(float current_position_m,
                                 float target_velocity_m_s,
                                 bool hold_enable);

#endif
