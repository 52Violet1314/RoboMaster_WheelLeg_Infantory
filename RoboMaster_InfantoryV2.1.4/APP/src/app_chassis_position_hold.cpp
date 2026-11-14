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
                                 float target_velocity_m_s,
                                 bool hold_enable) {
    if (!std::isfinite(current_position_m) ||
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

    if (!s_holding) {
        s_position_reference_m = current_position_m;
        s_holding = true;
        return 0.0f;
    }

    return current_position_m - s_position_reference_m;
}
