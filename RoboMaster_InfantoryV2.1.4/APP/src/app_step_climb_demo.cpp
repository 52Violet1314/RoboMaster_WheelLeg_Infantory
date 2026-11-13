#include "app_step_climb_demo.hpp"
#include <math.h>

static StepClimbState_t s_state = STEP_CLIMB_IDLE;
static float s_action_time_s = 0.0f;
static float s_detect_time_s = 0.0f;

void StepClimb_Init(void) {
    s_state = STEP_CLIMB_IDLE;
    s_action_time_s = 0.0f;
    s_detect_time_s = 0.0f;
}
StepClimbState_t StepClimb_GetState(void) { return s_state; }

StepClimbCommand_t StepClimb_Update(float dt, bool ch6, bool enabled, bool lqr_switch,
                                    bool stand_up, float l0_l, float l0_r,
                                    float theta_l, float theta_r) {
    StepClimbCommand_t cmd = {};
    cmd.allow_gravity = true;
    cmd.allow_lqr = true;
    if (!ch6) {
        StepClimb_Init();
        return cmd;
    }
    if (s_state == STEP_CLIMB_IDLE) {
        s_state = STEP_CLIMB_ARMED;
    }
    if (s_state == STEP_CLIMB_ARMED) {
        const bool angle_detected =
                fabsf(theta_l) >= STEP_CLIMB_DETECT_ANGLE_ABS_RAD ||
                fabsf(theta_r) >= STEP_CLIMB_DETECT_ANGLE_ABS_RAD;
        if (enabled && stand_up && angle_detected) {
            s_detect_time_s += dt;
            if (s_detect_time_s >= STEP_CLIMB_DETECT_CONFIRM_S) {
                s_state = STEP_CLIMB_ACTION;
                s_action_time_s = 0.0f;
                s_detect_time_s = 0.0f;
            }
        } else {
            s_detect_time_s = 0.0f;
        }
    }
    if (s_state == STEP_CLIMB_ACTION) {
        s_action_time_s += dt;
        cmd.owns_control = true;
        cmd.force_not_up = true;
        cmd.suppress_ground_detection = true;
        cmd.override_leg_length = true;
        cmd.swing_pid_enable = enabled;
        cmd.allow_gravity = false;
        cmd.allow_lqr = false;
        cmd.leg_length_target_m = STEP_CLIMB_TARGET_LENGTH_M;
        cmd.swing_target_rad = STEP_CLIMB_TARGET_REAR_ANGLE_RAD;
        const bool length_ok =
                fabsf(l0_l - cmd.leg_length_target_m) <=
                        STEP_CLIMB_LENGTH_TOLERANCE_M &&
                fabsf(l0_r - cmd.leg_length_target_m) <=
                        STEP_CLIMB_LENGTH_TOLERANCE_M;
        const bool angle_ok =
                fabsf(theta_l - cmd.swing_target_rad) <=
                        STEP_CLIMB_ANGLE_TOLERANCE_RAD &&
                fabsf(theta_r - cmd.swing_target_rad) <=
                        STEP_CLIMB_ANGLE_TOLERANCE_RAD;
        if (length_ok && angle_ok) {
            s_state = STEP_CLIMB_RECOVER;
            s_action_time_s = 0.0f;
        } else if (s_action_time_s >= STEP_CLIMB_ACTION_TIMEOUT_S) {
            s_state = STEP_CLIMB_FAULT_HOLD;
        }
    } else if (s_state == STEP_CLIMB_RECOVER) {
        /* 对位完成后释放摆角/腿长目标，由原 LQR 起身；起身完成前仍屏蔽
         * 普通离地检测，避免台阶边沿的 Fn 跳变再次改写控制模式。 */
        cmd.owns_control = true;
        cmd.suppress_ground_detection = true;
        cmd.allow_lqr = enabled && lqr_switch;
        cmd.allow_gravity = cmd.allow_lqr;
        s_action_time_s += dt;
        if (stand_up) {
            s_state = STEP_CLIMB_ARMED;
            s_action_time_s = 0.0f;
            cmd = {};
            cmd.allow_gravity = true;
            cmd.allow_lqr = true;
        } else if (s_action_time_s >= STEP_CLIMB_RECOVER_TIMEOUT_S) {
            s_state = STEP_CLIMB_FAULT_HOLD;
        }
    }
    if (s_state == STEP_CLIMB_FAULT_HOLD) {
        cmd.owns_control = true;
        cmd.force_not_up = true;
        cmd.suppress_ground_detection = true;
        cmd.override_leg_length = true;
        cmd.swing_pid_enable = enabled;
        cmd.allow_gravity = false;
        cmd.allow_lqr = false;
        cmd.leg_length_target_m = STEP_CLIMB_TARGET_LENGTH_M;
        cmd.swing_target_rad = STEP_CLIMB_TARGET_REAR_ANGLE_RAD;
    }
    return cmd;
}
