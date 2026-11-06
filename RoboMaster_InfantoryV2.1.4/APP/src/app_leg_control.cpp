#include "app_leg_control.hpp"

#include "app_leg_force_compensation.hpp"
#include "app_pid.h"
#include <cmath>
#include <cstdint>

/* 腿长力闭环：L0 由四连杆正运动学计算得到，PID 输出为腿向力修正量。 */
static Float_PID_Typedef leg_length_pid_l;
static Float_PID_Typedef leg_length_pid_r;
static float last_leg_target = 0.0f;

void LegControl_Init(void) {
    Float_PID_Init(&leg_length_pid_l, LEG_LENGTH_PID_KP,
                   LEG_LENGTH_PID_KI, LEG_LENGTH_PID_KD);
    Float_PID_Init(&leg_length_pid_r, LEG_LENGTH_PID_KP,
                   LEG_LENGTH_PID_KI, LEG_LENGTH_PID_KD);
}

void LegControl_Update(Classic_Data_t *pClassicData, float dt,
                       bool stand_up, bool leg_pid_enabled,
                       int16_t leg_knob_channel) {
    /* 气弹簧预压补偿全程按本轮左右实际 L0 分别计算，不依赖起身状态。 */
    const float spring_preload_f0_l =
            LegForceCompensation_GetFittedF0(pClassicData->Leg_Data.L0_L);
    const float spring_preload_f0_r =
            LegForceCompensation_GetFittedF0(pClassicData->Leg_Data.L0_R);

    /* 腿长旋钮控制：VrB=云台 CH6=底盘 Channel[5]。
     * SBUS 解包后的 Channel[5] 是 raw-1024；此处恢复原始 0~2047 值。
     * 实测旋钮最低端约 raw=240、最高端约 raw=1814：0~250 为低端死区。
     *
     * 旋钮位置始终先按同一条 140~380 mm 标尺换算。进入起身状态后只将
     * 140~150 mm、350~380 mm 两端的多余行程裁为 150/350 mm 死区，绝不把
     * 整段旋钮重新映射到 150~350 mm。因而例如 250 mm 对应的旋钮位置，在
     * 起身前后目标仍严格为 250 mm，不会发生目标突变。 */
    constexpr int32_t leg_knob_raw_min = 0;
    constexpr int32_t leg_knob_raw_max = 2047;
    constexpr int32_t leg_knob_low_deadband_raw = 250;
    constexpr int32_t leg_knob_high_deadband_raw = 1800;
    constexpr int32_t leg_knob_active_min_raw = leg_knob_low_deadband_raw + 1;
    constexpr int32_t leg_knob_active_max_raw = leg_knob_high_deadband_raw - 1;
    constexpr float leg_knob_active_span_raw =
            (float)(leg_knob_active_max_raw - leg_knob_active_min_raw);
    int32_t leg_knob_raw = (int32_t)leg_knob_channel + 1024;
    if (leg_knob_raw < leg_knob_raw_min) {
        leg_knob_raw = leg_knob_raw_min;
    } else if (leg_knob_raw > leg_knob_raw_max) {
        leg_knob_raw = leg_knob_raw_max;
    }
    constexpr float leg_knob_full_min_m = 0.140f;
    constexpr float leg_knob_full_max_m = 0.380f;
    constexpr float leg_knob_stand_min_m = 0.150f;
    constexpr float leg_knob_stand_max_m = 0.350f;
    float leg_knob_full_target_m = leg_knob_full_min_m;
    if (leg_knob_raw <= leg_knob_low_deadband_raw) {
        leg_knob_full_target_m = leg_knob_full_min_m;
    } else if (leg_knob_raw >= leg_knob_high_deadband_raw) {
        leg_knob_full_target_m = leg_knob_full_max_m;
    } else {
        const float leg_knob_ratio =
                (float)(leg_knob_raw - leg_knob_active_min_raw) / leg_knob_active_span_raw;
        leg_knob_full_target_m = leg_knob_full_min_m +
                (leg_knob_full_max_m - leg_knob_full_min_m) * leg_knob_ratio;
    }
    float leg_knob_target_m = leg_knob_full_target_m;
    /* 起身前可用 140~380 mm；起身后将多余两端裁为 150/350 mm 死区。
     * 二者均基于同一条旋钮标尺，不会因起身切换重映射中段目标。 */
    if (stand_up) {
        /* 起身可用行程之外的旋钮位置直接视为端点死区。 */
        if (leg_knob_target_m < leg_knob_stand_min_m) {
            leg_knob_target_m = leg_knob_stand_min_m;
        } else if (leg_knob_target_m > leg_knob_stand_max_m) {
            leg_knob_target_m = leg_knob_stand_max_m;
        }
    }
    pClassicData->Target_Data.Target_LegLong = leg_knob_target_m;

    /* 仅已起身时计算左右反向的 roll F0 补偿。 */
    float roll_comp = 0.0f;
    if (stand_up) {
        constexpr float roll_kp = 40.0f;
        constexpr float roll_kd = 2.5f;
        constexpr float roll_f0_limit = 8.0f;
        roll_comp = -(roll_kp * pClassicData->IMU_Data.roll
                      + roll_kd * pClassicData->IMU_Data.delta_roll);
        if (roll_comp > roll_f0_limit) {
            roll_comp = roll_f0_limit;
        } else if (roll_comp < -roll_f0_limit) {
            roll_comp = -roll_f0_limit;
        }
    }

    /* 腿长 PID。积分按输出限幅，避免长期悬空时累积。 */
    const float leg_target_r = pClassicData->Target_Data.Target_LegLong;
    const float leg_target_l = leg_target_r;
    const float leg_err_l = leg_target_l - pClassicData->Leg_Data.L0_L;
    const float leg_err_r = leg_target_r - pClassicData->Leg_Data.L0_R;
    /* 仅用于腿长 PID 的误差死区。保留原始误差给调试和目标切换判断；
     * 死区内不产生新的 P/I/D 修正，但不清已有积分维持力。 */
    constexpr float leg_pid_deadband_m = LEG_LENGTH_PID_DEADBAND_M;
    float leg_pid_current_r = pClassicData->Leg_Data.L0_R;
    float leg_pid_current_l = pClassicData->Leg_Data.L0_L;
    if (fabsf(leg_err_l) <= leg_pid_deadband_m) {
        leg_pid_current_l = leg_target_l;
    }
    if (fabsf(leg_err_r) <= leg_pid_deadband_m) {
        leg_pid_current_r = leg_target_r;
    }
    if (fabsf(leg_target_r - last_leg_target) > 0.001f) {//发生换档
        /* 目标切换不清积分，连续调节旋钮时保持已有持力；仅更新微分历史。 */
        leg_length_pid_l.Last_Error = leg_err_l;
        leg_length_pid_r.Last_Error = leg_err_r;//更新误差长度
        leg_length_pid_l.Last_Dout = 0.0f;
        leg_length_pid_r.Last_Dout = 0.0f;//清输出
        last_leg_target = leg_target_r;
    }
    /* 总使能前禁止积分；限幅直接使用积分输出单位 N。 */
    constexpr float leg_pid_iout_limit_n = LEG_LENGTH_PID_I_OUTPUT_LIMIT_N;
    const float leg_r_iout_limit_n = leg_pid_enabled ? leg_pid_iout_limit_n : 0.0f;
    const float leg_r_out_limit = leg_pid_enabled ? LEG_LENGTH_PID_OUTPUT_LIMIT_N : 0.0f;
    const float leg_l_iout_limit_n = leg_pid_enabled ? leg_pid_iout_limit_n : 0.0f;
    const float leg_l_out_limit = leg_pid_enabled ? LEG_LENGTH_PID_OUTPUT_LIMIT_N : 0.0f;
    /* PID_Max_Float 由 Ki 是否为正决定是否累加积分；失能时临时置零，调用后恢复参数，避免未使能期间产生任何积分项。 */
    const float leg_l_ki = leg_length_pid_l.Ki;
    const float leg_r_ki = leg_length_pid_r.Ki;
    if (!leg_pid_enabled) {
        leg_length_pid_l.Err_Int = 0.0f;
        leg_length_pid_r.Err_Int = 0.0f;
        leg_length_pid_l.Ki = 0.0f;
        leg_length_pid_r.Ki = 0.0f;
    }
    float leg_l_out = 0.0f;
    leg_l_out = PID_Max_Float_Dt(&leg_length_pid_l, leg_target_l,
                                 leg_pid_current_l,
                                 leg_l_iout_limit_n, leg_l_out_limit, dt);
    float leg_r_out = PID_Max_Float_Dt(&leg_length_pid_r, leg_target_r,
                                       leg_pid_current_r,
                                       leg_r_iout_limit_n, leg_r_out_limit, dt);
    leg_length_pid_l.Ki = leg_l_ki;
    leg_length_pid_r.Ki = leg_r_ki;
    /* 仅已起身时保留原有 +10 N F0 预输出；未起身不再叠加写死的预输出。 */
    const float stand_offset_l = stand_up ? 10.0f : 0.0f;
    const float stand_offset_r = stand_up ? 10.0f : 0.0f;
    pClassicData->Contronller_Data.F0_L = leg_l_out + spring_preload_f0_l
            + stand_offset_l
            + roll_comp;
    pClassicData->Contronller_Data.F0_R = leg_r_out + spring_preload_f0_r
            + stand_offset_r
            - roll_comp;

    if (!leg_pid_enabled) {
        /* 总使能关闭时不保留任何腿向力输出。
         * PID 积分同时清零，下一次使能从当前腿长重新建立。 */
        leg_length_pid_l.Err_Int = 0.0f;
        leg_length_pid_r.Err_Int = 0.0f;
        pClassicData->Contronller_Data.F0_L = 0.0f;
        pClassicData->Contronller_Data.F0_R = 0.0f;
    }

    /* 腿长算法的最终虚拟腿向力保护。放在本文件，保证 PID 的各模式、
     * 补偿叠加和 F0 限幅是一个可审查的整体；进入 VMC 的 F0 永远不超限。 */
    const float f0_limit_n = leg_pid_enabled ? LEG_LENGTH_PID_OUTPUT_LIMIT_N : 0.0f;
    if (pClassicData->Contronller_Data.F0_L > f0_limit_n) {
        pClassicData->Contronller_Data.F0_L = f0_limit_n;
    } else if (pClassicData->Contronller_Data.F0_L < -f0_limit_n) {
        pClassicData->Contronller_Data.F0_L = -f0_limit_n;
    }
    if (pClassicData->Contronller_Data.F0_R > f0_limit_n) {
        pClassicData->Contronller_Data.F0_R = f0_limit_n;
    } else if (pClassicData->Contronller_Data.F0_R < -f0_limit_n) {
        pClassicData->Contronller_Data.F0_R = -f0_limit_n;
    }
}
