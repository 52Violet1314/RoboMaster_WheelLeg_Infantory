#ifndef __APP_LEG_CONTROL_HPP__
#define __APP_LEG_CONTROL_HPP__

#include "app_Data_Task.hpp"
#include <cstdint>

#define LEG_CONTROL_TEST_MODE                       1
#define LEG_CONTROL_TEST_ENABLE_SPRING_COMPENSATION 1

/* 腿长控制标定参数，单位统一为 m、N、s。
 * 仅供 LegControl_Update() 使用：PID 算法本体不保存任何腿的专用限幅。
 * I_OUTPUT_LIMIT 是积分“输出项”的限幅（N），不是 Err_Int 内部的 m·s。
 * 左右腿使用同一组参数。 */
#define LEG_LENGTH_PID_KP                   0.0f
#define LEG_LENGTH_PID_KI                   0.0f
#define LEG_LENGTH_PID_KD                   0.0f
#define LEG_LENGTH_PID_DEADBAND_M           0.0001f
#define LEG_LENGTH_PID_I_OUTPUT_LIMIT_N    80.0f
#define LEG_LENGTH_PID_OUTPUT_LIMIT_N      80.0f
#define LEG_SWING_PID_KP                    0.0f
#define LEG_SWING_PID_KI                    0.0f
#define LEG_SWING_PID_KD                    0.0f
#define LEG_SWING_PID_OUTPUT_LIMIT_NM       1.5f
#define LEG_SWING_PID_OUTPUT_SIGN           1.0f

/* 腿长旋钮映射、气弹簧补偿、roll 差分和腿长 PID 的统一入口。
 * 起身状态机仍由 CalculateTask 维护；本模块只使用其本轮判定结果。 */
void LegControl_Init(void);
void LegControl_Update(Classic_Data_t *pClassicData, float dt,
                       bool stand_up, bool leg_pid_enabled,
                       int16_t leg_knob_channel, bool override_leg_target,
                       float override_leg_target_m, bool swing_pid_enable,
                       float swing_target_rad);

#endif
