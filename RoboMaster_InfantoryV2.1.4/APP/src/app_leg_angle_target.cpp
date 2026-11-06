#include "app_leg_angle_target.hpp"
#include <stdint.h>

/* Static leg-angle targets generated with the paired K schedule: CoM rear offset 36.561 mm. */
const LegAngleTargetParam_t LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT] = {
{0.150f, -0.234090108f},
    {0.155f, -0.226404756f},
    {0.160f, -0.219212084f},
    {0.165f, -0.212465823f},
    {0.170f, -0.206125363f},
    {0.175f, -0.200154907f},
    {0.180f, -0.194522781f},
    {0.185f, -0.189200843f},
    {0.190f, -0.184164007f},
    {0.195f, -0.179389837f},
    {0.200f, -0.174858203f},
    {0.205f, -0.170550989f},
    {0.210f, -0.166451851f},
    {0.215f, -0.162545999f},
    {0.220f, -0.158820018f},
    {0.225f, -0.155261714f},
    {0.230f, -0.151859973f},
    {0.235f, -0.148604647f},
    {0.240f, -0.145486445f},
    {0.245f, -0.142496858f},
    {0.250f, -0.139628059f},
    {0.255f, -0.136872852f},
    {0.260f, -0.134224601f},
    {0.265f, -0.131677177f},
    {0.270f, -0.129224916f},
    {0.275f, -0.126862565f},
    {0.280f, -0.124585259f},
    {0.285f, -0.122388474f},
    {0.290f, -0.120268003f},
    {0.295f, -0.118219929f},
    {0.300f, -0.116240598f},
    {0.305f, -0.114326598f},
    {0.310f, -0.112474740f},
    {0.315f, -0.110682040f},
    {0.320f, -0.108945703f},
    {0.325f, -0.107263106f},
    {0.330f, -0.105631789f},
    {0.335f, -0.104049437f},
    {0.340f, -0.102513875f},
    {0.345f, -0.101023054f},
    {0.350f, -0.099575043f}
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
