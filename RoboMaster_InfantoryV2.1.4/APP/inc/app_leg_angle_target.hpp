#ifndef __APP_LEG_ANGLE_TARGET_HPP__
#define __APP_LEG_ANGLE_TARGET_HPP__

/* Per-leg firmware theta target table, generated at a 0 mm fore-aft CoM offset. */
#define LEG_ANGLE_TARGET_TABLE_COUNT 41U
#define LEG_ANGLE_TARGET_MIN_LENGTH_M 0.150f
#define LEG_ANGLE_TARGET_LENGTH_STEP_M 0.005f

typedef struct { float leg_length_m; float theta_target_rad; } LegAngleTargetParam_t;

extern const LegAngleTargetParam_t LegAngleTargetTable[LEG_ANGLE_TARGET_TABLE_COUNT];
LegAngleTargetParam_t LegAngleTarget_GetParam(float leg_length_m);
float LegAngleTarget_GetTheta(float leg_length_m);

#endif
