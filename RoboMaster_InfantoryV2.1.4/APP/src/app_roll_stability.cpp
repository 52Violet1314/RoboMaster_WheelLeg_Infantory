#include "app_roll_stability.hpp"

#include <cmath>

static float s_angle_iout_m = 0.0f;
static float s_rate_iout_n = 0.0f;
static float s_last_roll_rate_rad_s = 0.0f;
static float s_roll_accel_filt_rad_s2 = 0.0f;
static bool s_rate_initialized = false;

static float ClampSymmetric(float value, float limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static bool SaturationPushesSameDirection(float raw_output,
                                          float output_limit,
                                          float error) {
    return (raw_output > output_limit && error > 0.0f) ||
           (raw_output < -output_limit && error < 0.0f);
}

void RollStability_Init(void) {
    s_angle_iout_m = 0.0f;
    s_rate_iout_n = 0.0f;
    s_last_roll_rate_rad_s = 0.0f;
    s_roll_accel_filt_rad_s2 = 0.0f;
    s_rate_initialized = false;
}

RollStabilityOutput_t RollStability_Update(float roll_rad,
                                           float roll_rate_rad_s,
                                           float dt_s,
                                           bool enabled) {
    RollStabilityOutput_t output = {};
    if (!enabled || !std::isfinite(roll_rad) ||
        !std::isfinite(roll_rate_rad_s) || !std::isfinite(dt_s) ||
        dt_s <= 0.0f) {
        RollStability_Init();
        return output;
    }

#if ROLL_ANGLE_LENGTH_PID_ENABLE
    const float angle_error_rad = -roll_rad;
    const float previous_angle_iout_m = s_angle_iout_m;
    s_angle_iout_m += ROLL_ANGLE_LENGTH_PID_KI_M_PER_RAD_S *
                      angle_error_rad * dt_s;
    s_angle_iout_m = ClampSymmetric(
            s_angle_iout_m, ROLL_ANGLE_LENGTH_PID_I_LIMIT_M);

    const float angle_raw_m =
            ROLL_ANGLE_LENGTH_PID_KP_M_PER_RAD * angle_error_rad +
            s_angle_iout_m -
            ROLL_ANGLE_LENGTH_PID_KD_M_S_PER_RAD * roll_rate_rad_s;
    output.leg_length_delta_m = ClampSymmetric(
            angle_raw_m, ROLL_ANGLE_LENGTH_PID_OUTPUT_LIMIT_M);
    if (SaturationPushesSameDirection(
            angle_raw_m, ROLL_ANGLE_LENGTH_PID_OUTPUT_LIMIT_M,
            angle_error_rad)) {
        s_angle_iout_m = previous_angle_iout_m;
    }
#else
    s_angle_iout_m = 0.0f;
#endif

#if ROLL_RATE_FORCE_PID_ENABLE
    if (!s_rate_initialized) {
        s_last_roll_rate_rad_s = roll_rate_rad_s;
        s_roll_accel_filt_rad_s2 = 0.0f;
        s_rate_initialized = true;
    }
    const float roll_accel_raw_rad_s2 =
            (roll_rate_rad_s - s_last_roll_rate_rad_s) / dt_s;
    s_last_roll_rate_rad_s = roll_rate_rad_s;
    s_roll_accel_filt_rad_s2 += ROLL_RATE_D_FILTER_ALPHA *
            (roll_accel_raw_rad_s2 - s_roll_accel_filt_rad_s2);

    const float rate_error_rad_s = -roll_rate_rad_s;
    const float previous_rate_iout_n = s_rate_iout_n;
    s_rate_iout_n += ROLL_RATE_FORCE_PID_KI_N_PER_RAD *
                     rate_error_rad_s * dt_s;
    s_rate_iout_n = ClampSymmetric(
            s_rate_iout_n, ROLL_RATE_FORCE_PID_I_LIMIT_N);

    const float rate_raw_n =
            ROLL_RATE_FORCE_PID_KP_N_PER_RAD_S * rate_error_rad_s +
            s_rate_iout_n -
            ROLL_RATE_FORCE_PID_KD_N_S2_PER_RAD *
                    s_roll_accel_filt_rad_s2;
    output.support_force_delta_n = ClampSymmetric(
            rate_raw_n, ROLL_RATE_FORCE_PID_OUTPUT_LIMIT_N);
    if (SaturationPushesSameDirection(
            rate_raw_n, ROLL_RATE_FORCE_PID_OUTPUT_LIMIT_N,
            rate_error_rad_s)) {
        s_rate_iout_n = previous_rate_iout_n;
    }
#else
    s_rate_iout_n = 0.0f;
    s_rate_initialized = false;
#endif

    return output;
}
