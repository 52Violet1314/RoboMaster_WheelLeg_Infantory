#ifndef __APP_LEG_FORCE_COMPENSATION_HPP__
#define __APP_LEG_FORCE_COMPENSATION_HPP__

/* 气弹簧补偿记录：140~380 mm 每 10 mm 一项。
 * 140~350 mm 为实测值；360~380 mm 沿用 350 mm 平台值。
 * f0_compensation_n 按现有 F0 符号填写：负值为收腿/拉腿。 */
#define LEG_FORCE_COMP_TABLE_COUNT    25U
#define LEG_FORCE_COMP_MIN_LENGTH_M   0.140f
#define LEG_FORCE_COMP_LENGTH_STEP_M  0.010f
#define LEG_FORCE_COMP_PLATEAU_START_LENGTH_M  0.350f
#define LEG_FORCE_COMP_MAX_LENGTH_M   0.380f

typedef struct {
    float leg_length_m;
    float f0_compensation_n;
} LegForceCompensationParam_t;

extern const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT];

/* 测试版连续拟合函数：由 fit_leg_spring_compensation.m 根据实测表生成。
 * 实测数组仅作为后续重新拟合的记录，不参与查表输出。 */
float LegForceCompensation_GetFittedF0(float leg_length_m);

#endif
