#include "app_stand_state.hpp"

#include <cmath>

static StandState_t s_state = STAND_STATE_NOT_UP;
static float s_not_up_time_s = 0.0f;

void StandState_Init(void) {
    s_state = STAND_STATE_NOT_UP;
    s_not_up_time_s = 0.0f;
}

StandState_t StandState_Update(float dt_s, float theta_l_rad, float theta_r_rad) {
    if (!std::isfinite(dt_s) || !std::isfinite(theta_l_rad) ||
        !std::isfinite(theta_r_rad) || dt_s <= 0.0f) {
        return s_state;
    }

    const float theta_l_abs = std::fabs(theta_l_rad);
    const float theta_r_abs = std::fabs(theta_r_rad);
    const bool both_legs_up =
            theta_l_abs < STAND_UP_ENTER_ANGLE_ABS_RAD &&
            theta_r_abs < STAND_UP_ENTER_ANGLE_ABS_RAD;
    const bool either_leg_not_up =
            theta_l_abs > STAND_UP_EXIT_ANGLE_ABS_RAD ||
            theta_r_abs > STAND_UP_EXIT_ANGLE_ABS_RAD;

    if (both_legs_up) {
        s_state = STAND_STATE_UP;
        s_not_up_time_s = 0.0f;
    } else if (either_leg_not_up && s_state != STAND_STATE_NOT_UP) {
        s_not_up_time_s += dt_s;
        if (s_not_up_time_s >= STAND_UP_EXIT_CONFIRM_TIME_S) {
            s_state = STAND_STATE_NOT_UP;
            s_not_up_time_s = STAND_UP_EXIT_CONFIRM_TIME_S;
        }
    }
    return s_state;
}

void StandState_ForceNotUp(void) {
    s_state = STAND_STATE_NOT_UP;
    s_not_up_time_s = 0.0f;
}

StandState_t StandState_Get(void) {
    return s_state;
}

bool StandState_IsUp(void) {
    return s_state == STAND_STATE_UP;
}
