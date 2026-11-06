#include "app_leg_force_compensation.hpp"
#include <math.h>

/* fit_leg_spring_compensation.m 的平滑拟合采样，单位 N。
 * 380 mm 的 +2 N 原始读数相对 370 mm 跳变过大，未强制穿过该孤立点。 */
const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT] = {
    {0.140f, -31.011f}, {0.150f, -30.936f}, {0.160f, -31.140f},
    {0.170f, -31.578f}, {0.180f, -32.207f}, {0.190f, -32.983f},
    {0.200f, -33.863f}, {0.210f, -34.802f}, {0.220f, -35.757f},
    {0.230f, -36.683f}, {0.240f, -37.538f}, {0.250f, -38.276f},
    {0.260f, -38.855f}, {0.270f, -39.230f}, {0.280f, -39.357f},
    {0.290f, -39.193f}, {0.300f, -38.694f}, {0.310f, -37.816f},
    {0.320f, -36.515f}, {0.330f, -34.747f}, {0.340f, -32.469f},
    {0.350f, -29.637f}, {0.360f, -26.206f}, {0.370f, -22.133f},
    {0.380f, -17.374f},
};

float LegForceCompensation_GetFittedF0(float leg_length_m) {
    if (!isfinite(leg_length_m)) return 0.0f;
    /* 低端越界仍撤销补偿；高端保持 380 mm 的收腿补偿，避免 380 mm 处
     * 从约 -17.4 N 突然跳到 0 N。这样继续变长时仍有同方向的回拉趋势。 */
    if (leg_length_m < LEG_FORCE_COMP_MIN_LENGTH_M) return 0.0f;
    if (leg_length_m > LEG_FORCE_COMP_MAX_LENGTH_M)
        leg_length_m = LEG_FORCE_COMP_MAX_LENGTH_M;

    /* MATLAB 三阶平滑趋势拟合。z ∈ [-1,1]，Horner 形式。 */
    const float z = (leg_length_m - 0.260f) / 0.120f;
    return (((12.627790f * z + 14.662207f) * z - 5.808986f) * z -
            38.854686f);
}
