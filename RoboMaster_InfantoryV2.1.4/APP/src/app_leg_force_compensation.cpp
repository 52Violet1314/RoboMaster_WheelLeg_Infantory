#include "app_leg_force_compensation.hpp"
#include <math.h>

/* fit_leg_spring_compensation.m 的受约束平滑拟合采样，单位 N。
 * 140 mm 强制通过实测 -28 N；380 mm 的 +2 N 原始读数相对 370 mm 跳变过大，
 * 仅作为低权重端点样本，不强制穿过该孤立点。 */
const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT] = {
    {0.130f,   0.000f}, {0.140f, -28.000f}, {0.150f, -29.971f},
    {0.160f, -31.652f}, {0.170f, -33.063f}, {0.180f, -34.226f},
    {0.190f, -35.162f}, {0.200f, -35.891f}, {0.210f, -36.433f},
    {0.220f, -36.809f}, {0.230f, -37.036f}, {0.240f, -37.135f},
    {0.250f, -37.124f}, {0.260f, -37.022f}, {0.270f, -36.846f},
    {0.280f, -36.614f}, {0.290f, -36.342f}, {0.300f, -36.049f},
    {0.310f, -35.751f}, {0.320f, -35.463f}, {0.330f, -35.201f},
    {0.340f, -34.981f}, {0.350f, -34.818f}, {0.360f, -34.727f},
    {0.370f, -34.722f}, {0.380f, -34.816f},
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
        constexpr float slope_at_140_n_per_m = -212.41398f;
        return -28.0f * h01 + (0.010f * slope_at_140_n_per_m) * h11;
    }

    /* MATLAB 受约束四阶平滑趋势拟合，z ∈ [-1,1]，Horner 形式。
     * 约束：140 mm 的拟合值严格为实测 -28 N。 */
    const float z = (leg_length_m - 0.260f) / 0.120f;
    return ((((0.31189121f * z - 5.11503692f) * z + 5.30203689f) * z +
             1.70707227f) * z - 37.02189275f);
}
