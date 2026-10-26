#include "app_leg_gravity_compensation.hpp"
#include "main.h"
#include "arm_math.h"
#include <math.h>

/* 腿长、质心、偏转角及转动惯量均来自 腿长质心.xlsx。
 * 质心 mm -> m，转动惯量 kg*mm^2 -> kg*m^2 已在此表完成转换。 */
const LegGravityParam_t LegGravityTable[LEG_GRAVITY_TABLE_COUNT] = {
    {0.150f, 0.08166f, 0.04559f, 3.73f, 4.19f, 0.016879924f, 0.028495679f},
    {0.155f, 0.08148f, 0.04879f, 4.96f, 5.39f, 0.017010244f, 0.028988080f},
    {0.160f, 0.08126f, 0.05194f, 6.14f, 6.57f, 0.017143739f, 0.029495402f},
    {0.165f, 0.08102f, 0.05504f, 7.32f, 7.72f, 0.017280536f, 0.030020893f},
    {0.170f, 0.08075f, 0.05809f, 8.46f, 8.84f, 0.017420741f, 0.030561315f},
    {0.175f, 0.08046f, 0.06110f, 9.57f, 9.95f, 0.017564449f, 0.031119374f},
    {0.180f, 0.08013f, 0.06408f, 10.67f, 11.03f, 0.017711742f, 0.031691682f},
    {0.185f, 0.07979f, 0.06702f, 11.75f, 12.10f, 0.017862692f, 0.032285281f},
    {0.190f, 0.07942f, 0.06992f, 12.83f, 13.15f, 0.018017365f, 0.032886108f},
    {0.195f, 0.07903f, 0.07280f, 13.86f, 14.19f, 0.018175816f, 0.033508347f},
    {0.200f, 0.07862f, 0.07564f, 14.90f, 15.22f, 0.018338099f, 0.034144638f},
    {0.205f, 0.07818f, 0.07846f, 15.93f, 16.24f, 0.018504257f, 0.034796273f},
    {0.210f, 0.07773f, 0.08126f, 16.94f, 17.24f, 0.018674335f, 0.035467082f},
    {0.215f, 0.07725f, 0.08404f, 17.95f, 18.24f, 0.018848369f, 0.036152586f},
    {0.220f, 0.07676f, 0.08679f, 18.95f, 19.24f, 0.019026394f, 0.036854265f},
    {0.225f, 0.07624f, 0.08952f, 19.94f, 20.22f, 0.019208444f, 0.037569861f},
    {0.230f, 0.07570f, 0.09224f, 20.92f, 21.20f, 0.019394546f, 0.038303550f},
    {0.235f, 0.07515f, 0.09494f, 21.91f, 22.18f, 0.019584730f, 0.039054705f},
    {0.240f, 0.07457f, 0.09762f, 22.89f, 23.15f, 0.019779021f, 0.039819005f},
    {0.245f, 0.07398f, 0.10029f, 23.86f, 24.13f, 0.019977442f, 0.040602775f},
    {0.250f, 0.07336f, 0.10294f, 24.83f, 25.10f, 0.020180018f, 0.041399244f},
    {0.255f, 0.07272f, 0.10558f, 25.81f, 26.06f, 0.020386769f, 0.042212894f},
    {0.260f, 0.07207f, 0.10820f, 26.78f, 27.03f, 0.020597717f, 0.043042676f},
    {0.265f, 0.07139f, 0.11081f, 27.75f, 28.00f, 0.020812880f, 0.043887396f},
    {0.270f, 0.07069f, 0.11341f, 28.72f, 28.97f, 0.021032279f, 0.044748904f},
    {0.275f, 0.06996f, 0.11600f, 29.70f, 29.95f, 0.021255931f, 0.045625264f},
    {0.280f, 0.06922f, 0.11858f, 30.68f, 30.92f, 0.021483855f, 0.046520141f},
    {0.285f, 0.06845f, 0.12115f, 31.66f, 31.90f, 0.021716067f, 0.047429766f},
    {0.290f, 0.06766f, 0.12371f, 32.64f, 32.89f, 0.021952585f, 0.048355934f},
    {0.295f, 0.06685f, 0.12626f, 33.63f, 33.87f, 0.022193426f, 0.049298583f},
    {0.300f, 0.06601f, 0.12880f, 34.63f, 34.87f, 0.022438606f, 0.050255903f},
    {0.305f, 0.06515f, 0.13134f, 35.63f, 35.87f, 0.022688142f, 0.051233128f},
    {0.310f, 0.06426f, 0.13386f, 36.64f, 36.88f, 0.022942049f, 0.052221590f},
    {0.315f, 0.06334f, 0.13638f, 37.66f, 37.90f, 0.023200346f, 0.053228365f},
    {0.320f, 0.06239f, 0.13889f, 38.68f, 38.92f, 0.023463048f, 0.054249998f},
    {0.325f, 0.06142f, 0.14139f, 39.72f, 39.96f, 0.023730172f, 0.055288164f},
    {0.330f, 0.06041f, 0.14389f, 40.77f, 41.01f, 0.024001736f, 0.056343450f},
    {0.335f, 0.05938f, 0.14637f, 41.82f, 42.07f, 0.024277758f, 0.057411572f},
    {0.340f, 0.05831f, 0.14886f, 42.90f, 43.14f, 0.024558255f, 0.058501079f},
    {0.345f, 0.05720f, 0.15133f, 43.98f, 44.23f, 0.024843248f, 0.059600473f},
    {0.350f, 0.05606f, 0.15308f, 45.09f, 45.34f, 0.025132756f, 0.060425963f},
};

static float LegGravity_Lerp(float lower, float upper, float ratio) {
    return lower + (upper - lower) * ratio;
}

static float LegGravity_ClampTp(float tp) {
    if (!isfinite(tp)) return 0.0f;
    if (tp > LEG_GRAVITY_TP_LIMIT_NM) return LEG_GRAVITY_TP_LIMIT_NM;
    if (tp < -LEG_GRAVITY_TP_LIMIT_NM) return -LEG_GRAVITY_TP_LIMIT_NM;
    return tp;
}

LegGravityParam_t LegGravity_GetParam(float leg_length_m) {
    if (leg_length_m <= LegGravityTable[0].leg_length_m) return LegGravityTable[0];
    if (leg_length_m >= LegGravityTable[LEG_GRAVITY_TABLE_COUNT - 1U].leg_length_m)
        return LegGravityTable[LEG_GRAVITY_TABLE_COUNT - 1U];

    const float table_index = (leg_length_m - LEG_GRAVITY_MIN_LENGTH_M) /
                              LEG_GRAVITY_LENGTH_STEP_M;
    uint32_t lower_index = (uint32_t)table_index;
    if (lower_index >= LEG_GRAVITY_TABLE_COUNT - 1U)
        lower_index = LEG_GRAVITY_TABLE_COUNT - 2U;
    const LegGravityParam_t &lower = LegGravityTable[lower_index];
    const LegGravityParam_t &upper = LegGravityTable[lower_index + 1U];
    const float ratio = (leg_length_m - lower.leg_length_m) /
                        (upper.leg_length_m - lower.leg_length_m);

    return {
        LegGravity_Lerp(lower.leg_length_m, upper.leg_length_m, ratio),
        LegGravity_Lerp(lower.com_x_m, upper.com_x_m, ratio),
        LegGravity_Lerp(lower.com_y_m, upper.com_y_m, ratio),
        LegGravity_Lerp(lower.theta1_deg, upper.theta1_deg, ratio),
        LegGravity_Lerp(lower.theta2_deg, upper.theta2_deg, ratio),
        LegGravity_Lerp(lower.inertia_hip_kg_m2, upper.inertia_hip_kg_m2, ratio),
        LegGravity_Lerp(lower.inertia_total_kg_m2, upper.inertia_total_kg_m2, ratio),
    };
}

float LegGravity_GetTp(float leg_length_m, float leg_angle_from_vertical_rad) {
    const LegGravityParam_t param = LegGravity_GetParam(leg_length_m);

    /* 表：x 后、y 下；VMC：x 前、y 下。腿角 theta=0 时竖直，正值向 VMC x 正方向摆。 */
    const float com_x_vmc =
        -param.com_x_m * arm_cos_f32(leg_angle_from_vertical_rad) +
         param.com_y_m * arm_sin_f32(leg_angle_from_vertical_rad);
    const float gravity_tp_vmc = LEG_GRAVITY_MASS_KG *
                                 LEG_GRAVITY_ACCELERATION_M_S2 * com_x_vmc;

    /* 输出的是抵消 gravity_tp_vmc 的反向补偿，不是重力本身造成的力矩。 */
    return LegGravity_ClampTp(-LEG_GRAVITY_TP_SIGN * gravity_tp_vmc);
}
