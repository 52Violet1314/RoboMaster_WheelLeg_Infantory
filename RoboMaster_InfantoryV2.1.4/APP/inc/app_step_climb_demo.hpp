#ifndef __APP_STEP_CLIMB_DEMO_HPP__
#define __APP_STEP_CLIMB_DEMO_HPP__

#include "app_stand_state.hpp"
#include <stdbool.h>

/* 台阶状态机编译开关；CH6 仍是运行时使能。 */
#define STEP_CLIMB_CONTROL_ENABLE              0

/* CH6（Channel[6]）模式开关；保持 CH7=LQR、CH9=总使能由操作手控制。 */
#define STEP_CLIMB_SWITCH_THRESHOLD             100
#define STEP_CLIMB_DETECT_CONFIRM_S            0.020f
#define STEP_CLIMB_TARGET_LENGTH_M             0.140f
#define STEP_CLIMB_TARGET_REAR_ANGLE_RAD      -0.45f
#define STEP_CLIMB_LENGTH_TOLERANCE_M          0.005f
#define STEP_CLIMB_ANGLE_TOLERANCE_RAD         0.050f
#define STEP_CLIMB_ACTION_TIMEOUT_S            3.0f
#define STEP_CLIMB_RECOVER_TIMEOUT_S           3.0f

typedef enum { STEP_CLIMB_IDLE, STEP_CLIMB_ARMED, STEP_CLIMB_SWING_REAR,
               STEP_CLIMB_RETRACT, STEP_CLIMB_RECOVER,
               STEP_CLIMB_FAULT_HOLD } StepClimbState_t;

typedef struct {
    bool force_not_up;
    bool override_leg_length;
    bool swing_pid_enable;
    bool allow_lqr;
    float leg_length_target_m;
    float swing_target_rad;
} StepClimbCommand_t;

void StepClimb_Init(void);
StepClimbCommand_t StepClimb_Update(float dt, bool ch6_enable, bool total_enable,
                                    bool lqr_switch_enable, bool stand_up,
                                    float l0_l, float l0_r, float theta_l, float theta_r);
StepClimbState_t StepClimb_GetState(void);

#endif
