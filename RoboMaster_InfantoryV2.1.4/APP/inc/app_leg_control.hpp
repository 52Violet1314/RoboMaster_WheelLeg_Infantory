#ifndef __APP_LEG_CONTROL_HPP__
#define __APP_LEG_CONTROL_HPP__

#include "app_Data_Task.hpp"
#include <cstdint>

/* 1：台架气弹簧测试；0：正式控制。
 * 切换版本只改此处。测试版强制已起身、关闭腿重力补偿与 roll，且关闭
 * 起身前/后 F0 预输出；正式版恢复状态机、roll、预输出，并在总使能且已起身后开腿重力补偿。 */
#define LEG_CONTROL_TEST_MODE 1

/* 腿长旋钮映射、气弹簧补偿、roll 差分和腿长 PID 的统一入口。
 * 起身状态机仍由 CalculateTask 维护；本模块只使用其本轮判定结果。 */
void LegControl_Init(void);
void LegControl_Update(Classic_Data_t *pClassicData, float dt,
                       bool stand_up, bool leg_pid_enabled,
                       int16_t leg_knob_channel);

#endif
