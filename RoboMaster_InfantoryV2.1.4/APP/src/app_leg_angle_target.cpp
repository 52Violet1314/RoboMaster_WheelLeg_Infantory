#include "app_leg_angle_target.hpp"
#include <stdint.h>

const LegAngleTargetParam_t
        LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT] = {
    {0.150f, +0.031727377f},
    {0.155f, +0.030703586f},
    {0.160f, +0.029743811f},
    {0.165f, +0.028842230f},
    {0.170f, +0.027993703f},
    {0.175f, +0.027193683f},
    {0.180f, +0.026438125f},
    {0.185f, +0.025723421f},
    {0.190f, +0.025046346f},
    {0.195f, +0.024404003f},
    {0.200f, +0.023793786f},
    {0.205f, +0.023213344f},
    {0.210f, +0.022660550f},
    {0.215f, +0.022133473f},
    {0.220f, +0.021630360f},
    {0.225f, +0.021149613f},
    {0.230f, +0.020689772f},
    {0.235f, +0.020249503f},
    {0.240f, +0.019827583f},
    {0.245f, +0.019422887f},
    {0.250f, +0.019034382f},
    {0.255f, +0.018661115f},
    {0.260f, +0.018302207f},
    {0.265f, +0.017956845f},
    {0.270f, +0.017624276f},
    {0.275f, +0.017303802f},
    {0.280f, +0.016994776f},
    {0.285f, +0.016696594f},
    {0.290f, +0.016408695f},
    {0.295f, +0.016130558f},
    {0.300f, +0.015861692f},
    {0.305f, +0.015601643f},
    {0.310f, +0.015349984f},
    {0.315f, +0.015106315f},
    {0.320f, +0.014870261f},
    {0.325f, +0.014641471f},
    {0.330f, +0.014419615f},
    {0.335f, +0.014204383f},
    {0.340f, +0.013995480f},
    {0.345f, +0.013792635f},
    {0.350f, +0.013595585f}
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
