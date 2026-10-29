#include "app_leg_angle_target.hpp"
#include <stdint.h>

const LegAngleTargetParam_t
        LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT] = {
    {0.150f, -0.050777104f},
    {0.155f, -0.049137791f},
    {0.160f, -0.047601056f},
    {0.165f, -0.046157558f},
    {0.170f, -0.044799061f},
    {0.175f, -0.043518267f},
    {0.180f, -0.042308694f},
    {0.185f, -0.041164560f},
    {0.190f, -0.040080694f},
    {0.195f, -0.039052454f},
    {0.200f, -0.038075665f},
    {0.205f, -0.037146557f},
    {0.210f, -0.036261723f},
    {0.215f, -0.035418070f},
    {0.220f, -0.034612788f},
    {0.225f, -0.033843318f},
    {0.230f, -0.033107322f},
    {0.235f, -0.032402663f},
    {0.240f, -0.031727377f},
    {0.245f, -0.031079669f},
    {0.250f, -0.030457881f},
    {0.255f, -0.029860488f},
    {0.260f, -0.029286082f},
    {0.265f, -0.028733361f},
    {0.270f, -0.028201120f},
    {0.275f, -0.027688240f},
    {0.280f, -0.027193684f},
    {0.285f, -0.026716488f},
    {0.290f, -0.026255751f},
    {0.295f, -0.025810639f},
    {0.300f, -0.025380368f},
    {0.305f, -0.024964209f},
    {0.310f, -0.024561479f},
    {0.315f, -0.024171537f},
    {0.320f, -0.023793786f},
    {0.325f, -0.023427660f},
    {0.330f, -0.023072632f},
    {0.335f, -0.022728205f},
    {0.340f, -0.022393911f},
    {0.345f, -0.022069308f},
    {0.350f, -0.021753982f}
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
