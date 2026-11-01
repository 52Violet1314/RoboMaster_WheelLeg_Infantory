#include "app_leg_force_compensation.hpp"
#include <math.h>
#include <stdint.h>

/* 每条腿独立以各自实际 L0 查询此表。
 * 第二列单位 N，负值为收腿/拉腿；350~384 mm 恒用 350 mm 项。 */
const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT] = {
    {0.130f, 0.0f},  {0.140f, -6.0f},
    {0.150f, -10.0f}, {0.160f, -12.0f}, {0.170f, -11.0f}, {0.180f, -10.0f},
    {0.190f, -10.0f}, {0.200f, -10.0f}, {0.210f, -10.0f}, {0.220f, -11.0f},
    {0.230f, -12.0f}, {0.240f, -13.0f}, {0.250f, -14.0f}, {0.260f, -14.0f},
    {0.270f, -14.0f}, {0.280f, -14.0f}, {0.290f, -13.5f}, {0.300f, -13.0f},
    {0.310f, -13.0f}, {0.320f, -13.0f}, {0.330f, -13.0f}, {0.340f, -12.5f},
    {0.350f, -12.0f},
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

float LegForceCompensation_GetFittedF0(float leg_length_m) {
    if (!isfinite(leg_length_m)) return 0.0f;
    if (leg_length_m <= LEG_FORCE_COMP_MIN_LENGTH_M) return 0.0f;
    if (leg_length_m >= LEG_FORCE_COMP_PLATEAU_START_LENGTH_M) return -12.0f;

    /* MATLAB fit_leg_spring_compensation.m 的 6 阶拟合结果。
     * z 被归一化到 [-1, 1]，Horner 形式减少嵌入式浮点运算误差。 */
    const float z = (leg_length_m - 0.240f) / 0.110f;
    return ((((((31.690724f * z - 21.919334f) * z - 38.477450f) * z +
                23.245344f) * z + 14.559939f) * z - 7.326009f) * z -
                13.773214f);
}
