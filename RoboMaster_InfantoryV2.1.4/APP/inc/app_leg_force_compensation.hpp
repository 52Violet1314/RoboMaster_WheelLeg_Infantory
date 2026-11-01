#ifndef __APP_LEG_FORCE_COMPENSATION_HPP__
#define __APP_LEG_FORCE_COMPENSATION_HPP__

/* 气弹簧预压：按实际单腿 L0 查询的 F0 轴向拉腿补偿。
 * 150~350 mm 每 10 mm 一个独立实测值；350~384 mm 恒用 350 mm 的值。
 * f0_compensation_n 按现有 F0 符号填写：负值为收腿/拉腿。 */
#define LEG_FORCE_COMP_TABLE_COUNT    21U
#define LEG_FORCE_COMP_MIN_LENGTH_M   0.150f
#define LEG_FORCE_COMP_LENGTH_STEP_M  0.010f
#define LEG_FORCE_COMP_PLATEAU_START_LENGTH_M  0.350f
#define LEG_FORCE_COMP_MAX_LENGTH_M   0.384f

typedef struct {
    float leg_length_m;
    float f0_compensation_n;
} LegForceCompensationParam_t;

extern const LegForceCompensationParam_t
        LegForceCompensationTable[LEG_FORCE_COMP_TABLE_COUNT];

/* 150~350 mm 表内线性插值；350~384 mm 固定返回 350 mm 项。 */
LegForceCompensationParam_t LegForceCompensation_GetParam(float leg_length_m);
float LegForceCompensation_GetF0(float leg_length_m);

#endif
