#ifndef __APP_ROLL_STABILITY_HPP__
#define __APP_ROLL_STABILITY_HPP__

#include <stdbool.h>

/* roll 角度环：输出左右腿长目标差。单位为 rad、m、s。 */
#define ROLL_ANGLE_LENGTH_PID_ENABLE              1
#define ROLL_ANGLE_LENGTH_PID_KP_M_PER_RAD        0.050f
#define ROLL_ANGLE_LENGTH_PID_KI_M_PER_RAD_S      0.010f
#define ROLL_ANGLE_LENGTH_PID_KD_M_S_PER_RAD      0.000f
#define ROLL_ANGLE_LENGTH_PID_I_LIMIT_M           0.020f
#define ROLL_ANGLE_LENGTH_PID_OUTPUT_LIMIT_M      0.020f

/* roll 角速度环：输出左右腿 F0 支持力差。默认关闭，待单独实车标定。 */
#define ROLL_RATE_FORCE_PID_ENABLE                 0
#define ROLL_RATE_FORCE_PID_KP_N_PER_RAD_S         2.500f
#define ROLL_RATE_FORCE_PID_KI_N_PER_RAD           0.000f
#define ROLL_RATE_FORCE_PID_KD_N_S2_PER_RAD        0.000f
#define ROLL_RATE_FORCE_PID_I_LIMIT_N             20.000f
#define ROLL_RATE_FORCE_PID_OUTPUT_LIMIT_N        20.000f
#define ROLL_RATE_D_FILTER_ALPHA                   0.100f

typedef struct {
    float leg_length_delta_m;
    float support_force_delta_n;
} RollStabilityOutput_t;

void RollStability_Init(void);
RollStabilityOutput_t RollStability_Update(float roll_rad,
                                           float roll_rate_rad_s,
                                           float dt_s,
                                           bool enabled);

#endif
