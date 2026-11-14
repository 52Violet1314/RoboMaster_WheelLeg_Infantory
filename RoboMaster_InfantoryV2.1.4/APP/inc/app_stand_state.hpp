#ifndef __APP_STAND_STATE_HPP__
#define __APP_STAND_STATE_HPP__

#include <stdbool.h>

/* 起身与台阶触发共用的腿角判定参数。theta 单位为 rad，均取绝对值。 */
#define STAND_UP_ENTER_ANGLE_ABS_RAD       0.40f
#define STAND_UP_EXIT_ANGLE_ABS_RAD        1.20f
#define STAND_UP_EXIT_CONFIRM_TIME_S       1.00f
#define STEP_CLIMB_DETECT_ANGLE_ABS_RAD    0.35f

typedef enum {
    STAND_STATE_UP,
    STAND_STATE_NOT_UP,
} StandState_t;

void StandState_Init(void);
StandState_t StandState_Update(float dt_s, float theta_l_rad, float theta_r_rad);
void StandState_ForceNotUp(void);
StandState_t StandState_Get(void);
bool StandState_IsUp(void);

#endif
