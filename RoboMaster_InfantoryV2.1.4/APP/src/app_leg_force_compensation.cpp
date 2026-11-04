#include "app_leg_force_compensation.hpp"
#include <math.h>

/* 140~350 mm 为实测记录；360~380 mm 为 350 mm 平台延拓。
 * 第二列单位 N，负值为收腿/拉腿。 */
const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT] = {
    {0.140f, -6.0f}, {0.150f, -10.0f}, {0.160f, -12.0f}, {0.170f, -11.0f},
    {0.180f, -10.0f},
    {0.190f, -10.0f}, {0.200f, -10.0f}, {0.210f, -10.0f}, {0.220f, -11.0f},
    {0.230f, -12.0f}, {0.240f, -13.0f}, {0.250f, -14.0f}, {0.260f, -14.0f},
    {0.270f, -14.0f}, {0.280f, -14.0f}, {0.290f, -13.5f}, {0.300f, -13.0f},
    {0.310f, -13.0f}, {0.320f, -13.0f}, {0.330f, -13.0f}, {0.340f, -12.5f},
    {0.350f, -12.0f}, {0.360f, -12.0f}, {0.370f, -12.0f}, {0.380f, -12.0f},
};

float LegForceCompensation_GetFittedF0(float leg_length_m) {
    if (!isfinite(leg_length_m)) return 0.0f;
    if (leg_length_m < LEG_FORCE_COMP_MIN_LENGTH_M) return 0.0f;
    if (leg_length_m >= LEG_FORCE_COMP_PLATEAU_START_LENGTH_M) return -12.0f;

    /* MATLAB fit_leg_spring_compensation.m 的 6 阶拟合结果。
     * z 被归一化到 [-1, 1]，Horner 形式减少嵌入式浮点运算误差。 */
    const float z = (leg_length_m - 0.240f) / 0.110f;
    return ((((((31.690724f * z - 21.919334f) * z - 38.477450f) * z +
                23.245344f) * z + 14.559939f) * z - 7.326009f) * z -
                13.773214f);
}
