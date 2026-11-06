#ifndef __APP_LEG_FORCE_COMPENSATION_HPP__
#define __APP_LEG_FORCE_COMPENSATION_HPP__

/* 气弹簧补偿的平滑拟合采样：140~380 mm 每 10 mm 一项。
 * 数组来自 fit_leg_spring_compensation.m，用于审阅和重新拟合记录；
 * 实际运行使用同一拟合的连续 Horner 函数。负值为收腿/拉腿。 */
#define LEG_FORCE_COMP_TABLE_COUNT    25U
#define LEG_FORCE_COMP_MIN_LENGTH_M   0.140f
#define LEG_FORCE_COMP_LENGTH_STEP_M  0.010f
#define LEG_FORCE_COMP_MAX_LENGTH_M   0.380f

typedef struct {
    float leg_length_m;
    float f0_compensation_n;
} LegForceCompensationParam_t;

extern const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT];

/* 连续平滑拟合函数。L0 位于 [140,380] mm 外时返回 0 N。 */
float LegForceCompensation_GetFittedF0(float leg_length_m);

#endif
