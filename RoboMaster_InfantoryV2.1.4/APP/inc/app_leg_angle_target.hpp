#ifndef __APP_LEG_ANGLE_TARGET_HPP__
#define __APP_LEG_ANGLE_TARGET_HPP__

/* 按单腿实际 L0 查询的腿摆角目标表。
 * 表内数值是新版 MATLAB 模型的 theta_l 静态平衡解（rad），机身质心前 20 mm。
 * MATLAB theta_l 与实车 States_Data.theta_L/theta_R 的正方向相反；
 * 因此只能经 LegAngleTarget_GetTheta() 取得可直接写入实车 LQR 的目标角。 */
#define LEG_ANGLE_TARGET_TABLE_COUNT    41U
#define LEG_ANGLE_TARGET_MIN_LENGTH_M   0.150f
#define LEG_ANGLE_TARGET_LENGTH_STEP_M  0.005f

typedef struct {
    float leg_length_m;
    float theta_target_rad;
} LegAngleTargetParam_t;

extern const LegAngleTargetParam_t
        LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT];

/* 越界腿长钳到表端点；表内腿长做线性插值。GetParam() 返回表内 MATLAB 原值。 */
LegAngleTargetParam_t LegAngleTarget_GetParam(float leg_length_m);
/* 返回实车 theta_L/theta_R 定义下的目标角（rad）。 */
float LegAngleTarget_GetTheta(float leg_length_m);

#endif
