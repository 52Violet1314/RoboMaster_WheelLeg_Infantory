#include "app_leg_angle_target.hpp"
#include <stdint.h>

/* Auto-generated from -25.000000 mm fore-aft CoM static equilibrium. */
const LegAngleTargetParam_t LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT] = {
    {0.150f, -0.159282947f},
    {0.155f, -0.154103021f},
    {0.160f, -0.149250625f},
    {0.165f, -0.144695538f},
    {0.170f, -0.140411161f},
    {0.175f, -0.136373979f},
    {0.180f, -0.132563141f},
    {0.185f, -0.128960075f},
    {0.190f, -0.125548198f},
    {0.195f, -0.122312647f},
    {0.200f, -0.119240066f},
    {0.205f, -0.116318417f},
    {0.210f, -0.113536821f},
    {0.215f, -0.110885423f},
    {0.220f, -0.108355274f},
    {0.225f, -0.105938226f},
    {0.230f, -0.103626849f},
    {0.235f, -0.101414349f},
    {0.240f, -0.099294504f},
    {0.245f, -0.097261604f},
    {0.250f, -0.095310399f},
    {0.255f, -0.093436055f},
    {0.260f, -0.091634111f},
    {0.265f, -0.089900448f},
    {0.270f, -0.088231250f},
    {0.275f, -0.086622983f},
    {0.280f, -0.085072367f},
    {0.285f, -0.083576354f},
    {0.290f, -0.082132104f},
    {0.295f, -0.080736976f},
    {0.300f, -0.079388501f},
    {0.305f, -0.078084376f},
    {0.310f, -0.076822447f},
    {0.315f, -0.075600696f},
    {0.320f, -0.074417232f},
    {0.325f, -0.073270282f},
    {0.330f, -0.072158181f},
    {0.335f, -0.071079362f},
    {0.340f, -0.070032353f},
    {0.345f, -0.069015764f},
    {0.350f, -0.068028289f}
};

LegAngleTargetParam_t LegAngleTarget_GetParam(float leg_length_m) {
    if (leg_length_m <= LegAngleTargetTable[0].leg_length_m) return LegAngleTargetTable[0];
    if (leg_length_m >= LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT - 1U].leg_length_m) return LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT - 1U];
    uint32_t lower = (uint32_t)((leg_length_m - LEG_ANGLE_TARGET_MIN_LENGTH_M) / LEG_ANGLE_TARGET_LENGTH_STEP_M);
    if (lower >= LEG_ANGLE_TARGET_TABLE_COUNT - 1U) lower = LEG_ANGLE_TARGET_TABLE_COUNT - 2U;
    const LegAngleTargetParam_t &lo = LegAngleTargetTable[lower];
    const LegAngleTargetParam_t &hi = LegAngleTargetTable[lower + 1U];
    const float ratio = (leg_length_m - lo.leg_length_m) / (hi.leg_length_m - lo.leg_length_m);
    return {leg_length_m, lo.theta_target_rad + (hi.theta_target_rad - lo.theta_target_rad) * ratio};
}

float LegAngleTarget_GetTheta(float leg_length_m) { return LegAngleTarget_GetParam(leg_length_m).theta_target_rad; }
