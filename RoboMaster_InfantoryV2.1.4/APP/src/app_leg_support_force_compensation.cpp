#include "app_leg_support_force_compensation.hpp"
#include <stdint.h>

/* 每条腿独立以各自 L0 查询此表。
 * 第二列单位 N；当前全部为 0，等待起身后各腿长的实测支撑力补偿。 */
const LegSupportForceCompensationParam_t
        LegSupportForceCompensationTable[LEG_SUPPORT_FORCE_COMP_TABLE_COUNT] = {
    {0.150f, 0.0f}, {0.155f, 0.0f}, {0.160f, 0.0f}, {0.165f, 0.0f},
    {0.170f, 0.0f}, {0.175f, 0.0f}, {0.180f, 0.0f}, {0.185f, 0.0f},
    {0.190f, 0.0f}, {0.195f, 0.0f}, {0.200f, 0.0f}, {0.205f, 0.0f},
    {0.210f, 0.0f}, {0.215f, 0.0f}, {0.220f, 0.0f}, {0.225f, 0.0f},
    {0.230f, 0.0f}, {0.235f, 0.0f}, {0.240f, 0.0f}, {0.245f, 0.0f},
    {0.250f, 0.0f}, {0.255f, 0.0f}, {0.260f, 0.0f}, {0.265f, 0.0f},
    {0.270f, 0.0f}, {0.275f, 0.0f}, {0.280f, 0.0f}, {0.285f, 0.0f},
    {0.290f, 0.0f}, {0.295f, 0.0f}, {0.300f, 0.0f}, {0.305f, 0.0f},
    {0.310f, 0.0f}, {0.315f, 0.0f}, {0.320f, 0.0f}, {0.325f, 0.0f},
    {0.330f, 0.0f}, {0.335f, 0.0f}, {0.340f, 0.0f}, {0.345f, 0.0f},
    {0.350f, 0.0f},
};

LegSupportForceCompensationParam_t
LegSupportForceCompensation_GetParam(float leg_length_m) {
    if (leg_length_m <= LegSupportForceCompensationTable[0].leg_length_m)
        return LegSupportForceCompensationTable[0];
    if (leg_length_m >= LegSupportForceCompensationTable[
                              LEG_SUPPORT_FORCE_COMP_TABLE_COUNT - 1U]
                              .leg_length_m)
        return LegSupportForceCompensationTable[
                LEG_SUPPORT_FORCE_COMP_TABLE_COUNT - 1U];

    const float table_index =
            (leg_length_m - LEG_SUPPORT_FORCE_COMP_MIN_LENGTH_M) /
            LEG_SUPPORT_FORCE_COMP_LENGTH_STEP_M;
    uint32_t lower_index = (uint32_t)table_index;
    if (lower_index >= LEG_SUPPORT_FORCE_COMP_TABLE_COUNT - 1U)
        lower_index = LEG_SUPPORT_FORCE_COMP_TABLE_COUNT - 2U;

    const LegSupportForceCompensationParam_t &lower =
            LegSupportForceCompensationTable[lower_index];
    const LegSupportForceCompensationParam_t &upper =
            LegSupportForceCompensationTable[lower_index + 1U];
    const float ratio = (leg_length_m - lower.leg_length_m) /
                        (upper.leg_length_m - lower.leg_length_m);

    return {
        lower.leg_length_m + (upper.leg_length_m - lower.leg_length_m) * ratio,
        lower.support_f0_compensation_n +
                (upper.support_f0_compensation_n -
                 lower.support_f0_compensation_n) * ratio,
    };
}

float LegSupportForceCompensation_GetF0(float leg_length_m) {
    return LegSupportForceCompensation_GetParam(leg_length_m)
            .support_f0_compensation_n;
}
