#include "app_leg_force_compensation.hpp"
#include <stdint.h>

/* 每条腿独立以各自实际 L0 查询此表。
 * 第二列单位 N，负值为收腿/拉腿；350~384 mm 恒用 350 mm 项。 */
const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT] = {
    {0.150f, -14.0f}, {0.160f, -13.0f}, {0.170f, -7.0f},  {0.180f, -9.0f},
    {0.190f, -10.0f}, {0.200f, -9.0f},  {0.210f, -10.0f}, {0.220f, -10.0f},
    {0.230f, -13.0f}, {0.240f, -14.0f}, {0.250f, -15.0f}, {0.260f, -13.0f},
    {0.270f, -15.0f}, {0.280f, -15.0f}, {0.290f, -12.0f}, {0.300f, -12.0f},
    {0.310f, -13.0f}, {0.320f, -13.0f}, {0.330f, -13.0f}, {0.340f, -13.0f},
    {0.350f, -14.0f},
};

LegForceCompensationParam_t LegForceCompensation_GetParam(float leg_length_m) {
    if (leg_length_m <= LegForceCompensationTable[0].leg_length_m)
        return LegForceCompensationTable[0];
    if (leg_length_m >= LEG_FORCE_COMP_PLATEAU_START_LENGTH_M)
        return LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT - 1U];

    const float table_index = (leg_length_m - LEG_FORCE_COMP_MIN_LENGTH_M) /
                              LEG_FORCE_COMP_LENGTH_STEP_M;
    uint32_t lower_index = (uint32_t)table_index;
    if (lower_index >= LEG_FORCE_COMP_TABLE_COUNT - 1U)
        lower_index = LEG_FORCE_COMP_TABLE_COUNT - 2U;

    const LegForceCompensationParam_t &lower = LegForceCompensationTable[lower_index];
    const LegForceCompensationParam_t &upper = LegForceCompensationTable[lower_index + 1U];
    const float ratio = (leg_length_m - lower.leg_length_m) /
                        (upper.leg_length_m - lower.leg_length_m);

    return {
        lower.leg_length_m + (upper.leg_length_m - lower.leg_length_m) * ratio,
        lower.f0_compensation_n +
                (upper.f0_compensation_n - lower.f0_compensation_n) * ratio,
    };
}

float LegForceCompensation_GetF0(float leg_length_m) {
    return LegForceCompensation_GetParam(leg_length_m).f0_compensation_n;
}
