#include "app_chassis_position_hold.hpp"

#include <cmath>

static float s_position_reference_m = 0.0f;
static bool s_holding = false;

void ChassisPositionHold_Init(float current_position_m) {
    s_position_reference_m = std::isfinite(current_position_m)
            ? current_position_m : 0.0f;
    s_holding = false;
}

float ChassisPositionHold_Update(float current_position_m,
                                 float current_velocity_m_s,
                                 float target_velocity_m_s,
                                 bool hold_enable) {
    if (!std::isfinite(current_position_m) ||
        !std::isfinite(current_velocity_m_s) ||
        !std::isfinite(target_velocity_m_s)) {
        s_holding = false;
        return 0.0f;
    }

    const bool velocity_command_active =
            std::fabs(target_velocity_m_s) >
            CHASSIS_POSITION_HOLD_VELOCITY_EPS_M_S;
    const bool should_hold = hold_enable && !velocity_command_active;

    if (!should_hold) {
        s_position_reference_m = current_position_m;
        s_holding = false;
        return 0.0f;
    }

    /* 松杆后先只执行零速度闭环。参考点持续跟随当前位置，因而整段
     * 刹车路程不会成为位移误差；实际速度足够小才捕获停车位置。 */
    if (!s_holding) {
        if (std::fabs(current_velocity_m_s) >
            CHASSIS_POSITION_HOLD_CAPTURE_SPEED_M_S) {
            s_position_reference_m = current_position_m;
            return 0.0f;
        }
        s_position_reference_m = current_position_m;
        s_holding = true;
        return 0.0f;
    }

    return current_position_m - s_position_reference_m;
}

void ChassisLongitudinalState_Update(Classic_Data_t *data,
                                     float left_position_m, float right_position_m,
                                     float left_velocity_m_s, float right_velocity_m_s,
                                     bool position_hold_enable) {
    const float current_position_m = (left_position_m + right_position_m) * 0.5f;
    data->States_Data.d_X = (left_velocity_m_s + right_velocity_m_s) * 0.5f;
    data->States_Data.d_x_err =
            data->States_Data.d_X - data->Target_Data.Target_X_vel;
    data->States_Data.X_pos = ChassisPositionHold_Update(
            current_position_m, data->States_Data.d_X,
            data->Target_Data.Target_X_vel,
            position_hold_enable);
}
