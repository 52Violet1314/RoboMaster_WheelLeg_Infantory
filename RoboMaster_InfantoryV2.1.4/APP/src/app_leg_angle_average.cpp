#include "app_leg_angle_average.hpp"

#include <math.h>

void LegAngleAverage_Reset(LegAngleAverage_t *average) {
    average->sum_left_rad = 0.0f;
    average->sum_right_rad = 0.0f;
    average->average_left_rad = 0.0f;
    average->average_right_rad = 0.0f;
    average->sample_count = 0U;
    average->ready = false;
}

void LegAngleAverage_Update(LegAngleAverage_t *average, bool stand_up,
                            float theta_left_rad, float theta_right_rad,
                            float dtheta_left_rad_s, float dtheta_right_rad_s) {
    const bool valid = stand_up && isfinite(theta_left_rad) &&
                       isfinite(theta_right_rad) && isfinite(dtheta_left_rad_s) &&
                       isfinite(dtheta_right_rad_s) &&
                       fabsf(dtheta_left_rad_s) <= LEG_ANGLE_AVERAGE_MAX_RATE_RAD_S &&
                       fabsf(dtheta_right_rad_s) <= LEG_ANGLE_AVERAGE_MAX_RATE_RAD_S;

    /* 未起身或仍在明显摆动：丢弃本轮，确保最终值只来自稳定窗口。 */
    if (!valid) {
        LegAngleAverage_Reset(average);
        return;
    }

    /* 已完成后冻结结果，等待下一次明显动作再开始新的 1 s 窗口。 */
    if (average->ready)
        return;

    average->sum_left_rad += theta_left_rad;
    average->sum_right_rad += theta_right_rad;
    ++average->sample_count;

    if (average->sample_count >= LEG_ANGLE_AVERAGE_SAMPLE_COUNT) {
        const float count = (float)average->sample_count;
        average->average_left_rad = average->sum_left_rad / count;
        average->average_right_rad = average->sum_right_rad / count;
        average->ready = true;
    }
}

float LegAngleAverage_GetLeftRad(const LegAngleAverage_t *average) {
    return average->ready ? average->average_left_rad : 0.0f;
}

float LegAngleAverage_GetRightRad(const LegAngleAverage_t *average) {
    return average->ready ? average->average_right_rad : 0.0f;
}

float LegAngleAverage_GetMeanRad(const LegAngleAverage_t *average) {
    return average->ready
               ? (average->average_left_rad + average->average_right_rad) * 0.5f
               : 0.0f;
}
