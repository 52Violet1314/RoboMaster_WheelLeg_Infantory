#include "app_leg_force_compensation.hpp"
#include <math.h>

/* fit_leg_spring_compensation.m 的受约束平滑拟合采样，单位 N。
 * 140 mm 强制通过实测 -28 N；380 mm 的 +2 N 原始读数相对 370 mm 跳变过大，
 * 仅作为低权重端点样本，不强制穿过该孤立点。 */
const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT] = {
    {0.130f,   0.000f}, {0.140f, -28.000f}, {0.150f, -30.046f},
    {0.160f, -31.757f}, {0.170f, -33.166f}, {0.180f, -34.304f},
    {0.190f, -35.199f}, {0.200f, -35.881f}, {0.210f, -36.376f},
    {0.220f, -36.713f}, {0.230f, -36.916f}, {0.240f, -37.010f},
    {0.250f, -37.019f}, {0.260f, -36.966f}, {0.270f, -36.873f},
    {0.280f, -36.761f}, {0.290f, -36.650f}, {0.300f, -36.558f},
    {0.310f, -36.544f}, {0.320f, -36.763f}, {0.330f, -37.263f},
    {0.340f, -37.982f}, {0.350f, -38.801f}, {0.360f, -39.601f},
    {0.370f, -40.315f}, {0.380f, -40.987f},
};

float LegForceCompensation_GetFittedF0(float leg_length_m) {
    if (!isfinite(leg_length_m)) return 0.0f;
    /* 130 mm 以下撤销补偿；高端保持 380 mm 的收腿补偿，避免最长处
     * 突然跳为零。 */
    if (leg_length_m < LEG_FORCE_COMP_MIN_LENGTH_M) return 0.0f;
    if (leg_length_m > LEG_FORCE_COMP_MAX_LENGTH_M)
        leg_length_m = LEG_FORCE_COMP_MAX_LENGTH_M;

    /* 130~140 mm：三次 Hermite 过渡。
     * 130 mm 的力和斜率都为零；140 mm 精确为实测 -28 N，且斜率与下方
     * 主拟合一致。因此越过 140 mm 时 F0 不再有突变。 */
    if (leg_length_m < 0.140f) {
        const float t = (leg_length_m - 0.130f) / 0.010f;
        const float h01 = -2.0f * t * t * t + 3.0f * t * t;
        const float h11 = t * t * t - t * t;
        constexpr float slope_at_140_n_per_m = -222.33253f;
        return -28.0f * h01 + (0.010f * slope_at_140_n_per_m) * h11;
    }

    /* MATLAB 受约束四阶平滑趋势拟合，z ∈ [-1,1]，Horner 形式。
     * 约束：140 mm 的拟合值严格为实测 -28 N。 */
    const float z = (leg_length_m - 0.260f) / 0.120f;
    const float base_f0 = ((((0.83489717f * z - 6.16044808f) * z +
            2.88790652f) * z + 0.91684180f) * z - 36.96640998f);
    if (leg_length_m <= 0.300f) return base_f0;

    const float t = (leg_length_m - 0.300f) / 0.080f;
    const float smoothstep5 = 6.0f * t * t * t * t * t
            - 15.0f * t * t * t * t + 10.0f * t * t * t;
    return base_f0 - 2.5f * smoothstep5;
}
