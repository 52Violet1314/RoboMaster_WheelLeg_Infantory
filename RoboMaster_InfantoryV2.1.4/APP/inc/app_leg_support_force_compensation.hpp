#ifndef __APP_LEG_SUPPORT_FORCE_COMPENSATION_HPP__
#define __APP_LEG_SUPPORT_FORCE_COMPENSATION_HPP__

/* 起身后支撑力实测记录；当前全部为 0 N。 */
#define LEG_SUPPORT_FORCE_COMP_TABLE_COUNT    41U
#define LEG_SUPPORT_FORCE_COMP_MIN_LENGTH_M   0.150f
#define LEG_SUPPORT_FORCE_COMP_LENGTH_STEP_M  0.005f

typedef struct {
    float leg_length_m;
    float support_f0_compensation_n;
} LegSupportForceCompensationParam_t;

extern const LegSupportForceCompensationParam_t
        LegSupportForceCompensationTable[LEG_SUPPORT_FORCE_COMP_TABLE_COUNT];

#endif
