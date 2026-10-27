#ifndef __APP_LEG_SUPPORT_FORCE_COMPENSATION_HPP__
#define __APP_LEG_SUPPORT_FORCE_COMPENSATION_HPP__

/* 预留：起身后按实际腿长查得的单腿 F0 支撑力补偿表。
 *
 * 该表与 app_leg_force_compensation 的悬空预压表独立：
 * - 悬空预压：用于消除气弹簧/机构本身的预紧力影响；
 * - 本表：仅在 StandUp_Flag 为 true 后，用于补足整车支撑所需的基础力。
 *
 * 后续接入 CalculateTask 时，单腿 F0 的设计叠加顺序为：
 *   F0 = 悬空预压(L0) + 起身后支撑补偿(L0) + 腿长_PID
 * 当前模块只提供表和查询，不接入输出，全部初值为 0 N。 */
#define LEG_SUPPORT_FORCE_COMP_TABLE_COUNT    41U
#define LEG_SUPPORT_FORCE_COMP_MIN_LENGTH_M   0.150f
#define LEG_SUPPORT_FORCE_COMP_LENGTH_STEP_M  0.005f

typedef struct {
    float leg_length_m;
    float support_f0_compensation_n;
} LegSupportForceCompensationParam_t;

extern const LegSupportForceCompensationParam_t
        LegSupportForceCompensationTable[LEG_SUPPORT_FORCE_COMP_TABLE_COUNT];

/* 越界腿长钳到表端点；表内腿长做线性插值。 */
LegSupportForceCompensationParam_t
LegSupportForceCompensation_GetParam(float leg_length_m);
float LegSupportForceCompensation_GetF0(float leg_length_m);

#endif
