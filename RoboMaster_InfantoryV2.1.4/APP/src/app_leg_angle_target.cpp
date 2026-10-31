#include "app_leg_angle_target.hpp"
#include <stdint.h>

/* Auto-generated from the same static equilibrium as the K schedule. */
const LegAngleTargetParam_t LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT] = {
    {0.150f, -0.234087190f},
    {0.155f, -0.226401938f},
    {0.160f, -0.219209356f},
    {0.165f, -0.212463183f},
    {0.170f, -0.206122806f},
    {0.175f, -0.200152426f},
    {0.180f, -0.194520371f},
    {0.185f, -0.189198500f},
    {0.190f, -0.184161728f},
    {0.195f, -0.179387619f},
    {0.200f, -0.174856042f},
    {0.205f, -0.170548882f},
    {0.210f, -0.166449796f},
    {0.215f, -0.162543993f},
    {0.220f, -0.158818059f},
    {0.225f, -0.155259799f},
    {0.230f, -0.151858101f},
    {0.235f, -0.148602815f},
    {0.240f, -0.145484654f},
    {0.245f, -0.142495103f},
    {0.250f, -0.139626340f},
    {0.255f, -0.136871160f},
    {0.260f, -0.134222949f},
    {0.265f, -0.131675557f},
    {0.270f, -0.129223325f},
    {0.275f, -0.126861005f},
    {0.280f, -0.124583727f},
    {0.285f, -0.122386968f},
    {0.290f, -0.120266525f},
    {0.295f, -0.118218475f},
    {0.300f, -0.116239169f},
    {0.305f, -0.114325193f},
    {0.310f, -0.112473358f},
    {0.315f, -0.110680681f},
    {0.320f, -0.108944365f},
    {0.325f, -0.107261789f},
    {0.330f, -0.105630492f},
    {0.335f, -0.104048159f},
    {0.340f, -0.102512616f},
    {0.345f, -0.101021814f},
    {0.350f, -0.099573821f}
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
