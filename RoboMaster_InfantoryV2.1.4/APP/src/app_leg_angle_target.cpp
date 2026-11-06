#include "app_leg_angle_target.hpp"
#include <stdint.h>

/* Static leg-angle targets paired with Q=77 K schedule: CoM rear offset 21.936332 mm; L0=150 mm -> -8 deg. */
const LegAngleTargetParam_t LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT] = {
{0.150f, -0.139626340f},
    {0.155f, -0.135094193f},
    {0.160f, -0.130847840f},
    {0.165f, -0.126861005f},
    {0.170f, -0.123110542f},
    {0.175f, -0.119575984f},
    {0.180f, -0.116239169f},
    {0.185f, -0.113083920f},
    {0.190f, -0.110095779f},
    {0.195f, -0.107261789f},
    {0.200f, -0.104570300f},
    {0.205f, -0.102010809f},
    {0.210f, -0.099573821f},
    {0.215f, -0.097250735f},
    {0.220f, -0.095033734f},
    {0.225f, -0.092915705f},
    {0.230f, -0.090890155f},
    {0.235f, -0.088951150f},
    {0.240f, -0.087093250f},
    {0.245f, -0.085311467f},
    {0.250f, -0.083601210f},
    {0.255f, -0.081958254f},
    {0.260f, -0.080378697f},
    {0.265f, -0.078858935f},
    {0.270f, -0.077395634f},
    {0.275f, -0.075985701f},
    {0.280f, -0.074626266f},
    {0.285f, -0.073314661f},
    {0.290f, -0.072048404f},
    {0.295f, -0.070825181f},
    {0.300f, -0.069642833f},
    {0.305f, -0.068499343f},
    {0.310f, -0.067392825f},
    {0.315f, -0.066321514f},
    {0.320f, -0.065283754f},
    {0.325f, -0.064277992f},
    {0.330f, -0.063302770f},
    {0.335f, -0.062356717f},
    {0.340f, -0.061438543f},
    {0.345f, -0.060547031f},
    {0.350f, -0.059681037f}
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
