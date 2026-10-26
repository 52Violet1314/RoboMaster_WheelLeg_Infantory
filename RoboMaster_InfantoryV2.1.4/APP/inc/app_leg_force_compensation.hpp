#ifndef __APP_LEG_FORCE_COMPENSATION_HPP__
#define __APP_LEG_FORCE_COMPENSATION_HPP__

/* 预留：按实际腿长查得的单腿 F0 轴向力补偿表。
 * 当前模块未接入 CalculateTask；填入实测值后再决定叠加位置和使能逻辑。 */
#define LEG_FORCE_COMP_TABLE_COUNT    41U
#define LEG_FORCE_COMP_MIN_LENGTH_M   0.150f
#define LEG_FORCE_COMP_LENGTH_STEP_M  0.005f

typedef struct {
    float leg_length_m;
    float f0_compensation_n;
} LegForceCompensationParam_t;

extern const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT];

/* 越界腿长钳到表端点；表内腿长做线性插值。 */
LegForceCompensationParam_t LegForceCompensation_GetParam(float leg_length_m);
float LegForceCompensation_GetF0(float leg_length_m);

#endif
