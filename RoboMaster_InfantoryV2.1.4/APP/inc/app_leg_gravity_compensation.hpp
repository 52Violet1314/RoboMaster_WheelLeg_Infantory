#ifndef __APP_LEG_GRAVITY_COMPENSATION_HPP__
#define __APP_LEG_GRAVITY_COMPENSATION_HPP__

/* 腿重力补偿常量：质心表以髋关节为原点，X 向后、Y 向下。 */
#define LEG_GRAVITY_MASS_KG              0.900f
#define LEG_GRAVITY_ACCELERATION_M_S2    9.81f
#define LEG_GRAVITY_TABLE_COUNT          41U
#define LEG_GRAVITY_MIN_LENGTH_M         0.150f
#define LEG_GRAVITY_LENGTH_STEP_M        0.005f
/* 重力补偿自身的独立安全限幅；在 LQR Tp 完成其 2/4 N·m 限幅后叠加进入 VMC。 */
#define LEG_GRAVITY_TP_LIMIT_NM          2.20f
/* 安装方向修正。1 为表 x后/y下 转 VMC x前/y下 后的理论补偿方向。 */
#define LEG_GRAVITY_TP_SIGN              1.0f
#define LEG_GRAVITY_COMPENSATION_ENABLE  1

typedef struct {
    float leg_length_m;
    float com_x_m;
    float com_y_m;
    float theta1_deg;
    float theta2_deg;
    float inertia_hip_kg_m2;
    float inertia_total_kg_m2;
} LegGravityParam_t;

extern const LegGravityParam_t LegGravityTable[LEG_GRAVITY_TABLE_COUNT];

LegGravityParam_t LegGravity_GetParam(float leg_length_m);
float LegGravity_GetTp(float leg_length_m, float leg_angle_from_vertical_rad);

#endif
