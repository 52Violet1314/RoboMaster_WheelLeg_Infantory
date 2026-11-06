#ifndef __APP_LEG_FORCE_COMPENSATION_HPP__
#define __APP_LEG_FORCE_COMPENSATION_HPP__

/* 气弹簧补偿的平滑拟合采样：130~380 mm 每 10 mm 一项。
 * 数组来自 fit_leg_spring_compensation.m，用于审阅和重新拟合记录；
 * 实际运行使用同一拟合的连续 Horner 函数。负值为收腿/拉腿。 */
#define LEG_FORCE_COMP_TABLE_COUNT    26U
#define LEG_FORCE_COMP_MIN_LENGTH_M   0.130f
#define LEG_FORCE_COMP_LENGTH_STEP_M  0.010f
#define LEG_FORCE_COMP_MAX_LENGTH_M   0.380f

typedef struct {
    float leg_length_m;
    float f0_compensation_n;
} LegForceCompensationParam_t;

extern const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT];

/* 连续平滑拟合函数。
 * L0 <= 130 mm 时返回 0 N；130~140 mm 平滑接入 140 mm 的实测 -28 N，
 * 既避免 140 mm 的 F0 突变，又在最短可用腿长提供收短阻力。
 * L0 > 380 mm 时保持 380 mm 的拟合值，避免最长腿长处补偿突然归零。 */
float LegForceCompensation_GetFittedF0(float leg_length_m);

#endif
