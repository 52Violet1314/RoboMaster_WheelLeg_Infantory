#include "app_leg_angle_target.hpp"
#include <stdint.h>

const LegAngleTargetParam_t
        LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT] = {
    {0.150f, +0.127231205f},
    {0.155f, +0.123105755f},
    {0.160f, +0.119240062f},
    {0.165f, +0.115610285f},
    {0.170f, +0.112195424f},
    {0.175f, +0.108976905f},
    {0.180f, +0.105938223f},
    {0.185f, +0.103064706f},
    {0.190f, +0.100343212f},
    {0.195f, +0.097761974f},
    {0.200f, +0.095310396f},
    {0.205f, +0.092978950f},
    {0.210f, +0.090758988f},
    {0.215f, +0.088642701f},
    {0.220f, +0.086622983f},
    {0.225f, +0.084693361f},
    {0.230f, +0.082847928f},
    {0.235f, +0.081081290f},
    {0.240f, +0.079388500f},
    {0.245f, +0.077765019f},
    {0.250f, +0.076206670f},
    {0.255f, +0.074709607f},
    {0.260f, +0.073270282f},
    {0.265f, +0.071885415f},
    {0.270f, +0.070551968f},
    {0.275f, +0.069267131f},
    {0.280f, +0.068028289f},
    {0.285f, +0.066833014f},
    {0.290f, +0.065679047f},
    {0.295f, +0.064564280f},
    {0.300f, +0.063486748f},
    {0.305f, +0.062444615f},
    {0.310f, +0.061436164f},
    {0.315f, +0.060459787f},
    {0.320f, +0.059513978f},
    {0.325f, +0.058597321f},
    {0.330f, +0.057708488f},
    {0.335f, +0.056846231f},
    {0.340f, +0.056009375f},
    {0.345f, +0.055196814f},
    {0.350f, +0.054407504f}
};

LegAngleTargetParam_t LegAngleTarget_GetParam(float leg_length_m) {
    if (leg_length_m <= LegAngleTargetTable[0].leg_length_m)
        return LegAngleTargetTable[0];
    if (leg_length_m >=
        LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT - 1U].leg_length_m)
        return LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT - 1U];

    const float table_index = (leg_length_m - LEG_ANGLE_TARGET_MIN_LENGTH_M) /
                              LEG_ANGLE_TARGET_LENGTH_STEP_M;
    uint32_t lower_index = (uint32_t)table_index;
    if (lower_index >= LEG_ANGLE_TARGET_TABLE_COUNT - 1U)
        lower_index = LEG_ANGLE_TARGET_TABLE_COUNT - 2U;

    const LegAngleTargetParam_t &lower = LegAngleTargetTable[lower_index];
    const LegAngleTargetParam_t &upper = LegAngleTargetTable[lower_index + 1U];
    const float ratio = (leg_length_m - lower.leg_length_m) /
                        (upper.leg_length_m - lower.leg_length_m);

    return {
        lower.leg_length_m + (upper.leg_length_m - lower.leg_length_m) * ratio,
        lower.theta_target_rad +
                (upper.theta_target_rad - lower.theta_target_rad) * ratio,
    };
}
float LegAngleTarget_GetTheta(float leg_length_m) {
    return LegAngleTarget_GetParam(leg_length_m).theta_target_rad;
}
