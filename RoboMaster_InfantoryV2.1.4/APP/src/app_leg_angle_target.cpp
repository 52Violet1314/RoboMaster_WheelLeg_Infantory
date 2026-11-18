#include "app_leg_angle_target.hpp"
#include <stdint.h>

/* Auto-generated from static equilibrium with fore-aft CoM offset -26.000000 mm. */
const LegAngleTargetParam_t LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT] = {
    {0.150f, -0.165712065f},
    {0.155f, -0.160319446f},
    {0.160f, -0.155268136f},
    {0.165f, -0.150526604f},
    {0.170f, -0.146067101f},
    {0.175f, -0.141865106f},
    {0.180f, -0.137898870f},
    {0.185f, -0.134149035f},
    {0.190f, -0.130598310f},
    {0.195f, -0.127231205f},
    {0.200f, -0.124033798f},
    {0.205f, -0.120993545f},
    {0.210f, -0.118099111f},
    {0.215f, -0.115340228f},
    {0.220f, -0.112707571f},
    {0.225f, -0.110192654f},
    {0.230f, -0.107787736f},
    {0.235f, -0.105485743f},
    {0.240f, -0.103280190f},
    {0.245f, -0.101165135f},
    {0.250f, -0.099135109f},
    {0.255f, -0.097185079f},
    {0.260f, -0.095310399f},
    {0.265f, -0.093506779f},
    {0.270f, -0.091770248f},
    {0.275f, -0.090097126f},
    {0.280f, -0.088483998f},
    {0.285f, -0.086927690f},
    {0.290f, -0.085425248f},
    {0.295f, -0.083973915f},
    {0.300f, -0.082571138f},
    {0.305f, -0.081214503f},
    {0.310f, -0.079901774f},
    {0.315f, -0.078630851f},
    {0.320f, -0.077399765f},
    {0.325f, -0.076206670f},
    {0.330f, -0.075049834f},
    {0.335f, -0.073927627f},
    {0.340f, -0.072838515f},
    {0.345f, -0.071781054f},
    {0.350f, -0.070753883f}
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
