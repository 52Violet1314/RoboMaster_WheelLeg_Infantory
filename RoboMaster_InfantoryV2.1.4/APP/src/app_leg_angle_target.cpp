#include "app_leg_angle_target.hpp"
#include <stdint.h>

const LegAngleTargetParam_t
        LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT] = {
    {0.150f, +0.012689162f},
    {0.155f, +0.012279813f},
    {0.160f, +0.011896051f},
    {0.165f, +0.011535548f},
    {0.170f, +0.011196252f},
    {0.175f, +0.010876347f},
    {0.180f, +0.010574215f},
    {0.185f, +0.010288415f},
    {0.190f, +0.010017658f},
    {0.195f, +0.009760787f},
    {0.200f, +0.009516760f},
    {0.205f, +0.009284637f},
    {0.210f, +0.009063568f},
    {0.215f, +0.008852782f},
    {0.220f, +0.008651577f},
    {0.225f, +0.008459315f},
    {0.230f, +0.008275413f},
    {0.235f, +0.008099336f},
    {0.240f, +0.007930597f},
    {0.245f, +0.007768745f},
    {0.250f, +0.007613367f},
    {0.255f, +0.007464082f},
    {0.260f, +0.007320539f},
    {0.265f, +0.007182414f},
    {0.270f, +0.007049404f},
    {0.275f, +0.006921231f},
    {0.280f, +0.006797635f},
    {0.285f, +0.006678377f},
    {0.290f, +0.006563231f},
    {0.295f, +0.006451988f},
    {0.300f, +0.006344453f},
    {0.305f, +0.006240445f},
    {0.310f, +0.006139791f},
    {0.315f, +0.006042333f},
    {0.320f, +0.005947920f},
    {0.325f, +0.005856413f},
    {0.330f, +0.005767678f},
    {0.335f, +0.005681593f},
    {0.340f, +0.005598038f},
    {0.345f, +0.005516907f},
    {0.350f, +0.005438093f}
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
