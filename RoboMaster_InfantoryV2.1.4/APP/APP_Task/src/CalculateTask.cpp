#include "CalculateTask.hpp"
#include "ControlTask.hpp"
#include "FreeRTOS.h"
#include "POWER.h"
#include "Power.h"
#include "SBUS.h"
#include "WS2812.h"
#include "app_Task.hpp"
#include "app_Data_Task.hpp"
#include "app_leg_angle_target.hpp"
#include "app_leg_gravity_compensation.hpp"
#include "app_motor.h"
#include "arm_math.h"
#include "bsp_UART.h"
#include "event_groups.h"
#include "fdcan.h"
#include "main.h"
#include "math.h"
#include "queue.h"
#include "stm32h7xx_hal_tim.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <stdint.h>
#include <type_traits>
#include "wheel_kalman_fliter.h"
#include "Ground_clearance_detection.h"
#include "app_K_value.hpp"
#include "app_vmc.hpp"
#include "app_control_limit.hpp"
#include "app_leg_control.hpp"

// 定义参考值（根据实际需求调整）
float theta_ref = 0.0f; // 期望整体角度
float d_theta_ref = 0.0f; // 期望整体角速度
float d_theta_L_ref = 0.0f; // 期望左腿角速度
float d_theta_R_ref = 0.0f; // 期望右腿角速度
float theta_L_ref = 0.0f; // 由左右各自腿长目标表实时更新，默认 0 rad
float theta_R_ref = 0.0f; // 由左右各自腿长目标表实时更新，默认 0 rad

bool StandUp_Flag = true;

typedef enum
{
    StandState_Up,
    StandState_NotUp,
} StandState_t;

#define a 0.2f // 一阶低通滤波

float angle_to_rad(float degrees);

void LEG_Data_Print(Classic_Data_t *pClassicData);

void Motor_Data_Print(Classic_Data_t *pClassicData);

void IMU_Data_Print(Classic_Data_t *pClassicData);

void State_Data_Print(Classic_Data_t *pClassicData);

/* FireWater CSV：目标L0、左L0、右L0、左误差、右误差、左F0、右F0。 */
void LegLengthPID_Data_Print(Classic_Data_t *pClassicData);

/* 仅供电机反馈超时/驱动故障恢复尝试使用。
 * 只向当前八台受反馈状态机监管、且已完成初始化的电机发送一次使能帧。 */
static void DM_Motor_EnableAllMonitored(void) {
    static DM_Motor_t *const motors[] = {
        &DM_8009P1, &DM_8009P2, &DM_8009P3, &DM_8009P4,
        &DM_3519L, &DM_3519R, &DM_3507, &DM_4310,
    };
    for (uint32_t i = 0; i < (sizeof(motors) / sizeof(motors[0])); ++i) {
        /* 3507/4310 在 RemoteTask 初始化；未初始化时 ID 为 0，不能发 ID 0。 */
        if (motors[i]->ID != 0U) {
            DM_Motor_Enable(motors[i]);
        }
    }
}

void CaculateTask(void *pvParameters) {
    float Pos_INT = 0.0f;
    /* 每次 LQR 重新有效时，仅记录一次当前位置为位置控制零点。 */
    bool lqr_pos_ref_recorded = false;
    static float Max_F0 = 50.0f;
    static float Max_Tp = 1.8f;
    VMC_Kinematics_t vmc_kinematics;
    // 左右腿共用一个状态：两侧角度都在阈值内才认为已起身，
    // 任一侧超出阈值连续 1 s 才切换为未起身。
    /* 上电先等待腿角给出一次已起身判定，避免起身过程提前记录位置零点。 */
    StandState_t stand_state = StandState_NotUp;
    float stand_not_up_time = 0.0f;
    Classic_Data_t *pClassicData = (Classic_Data_t *) pvParameters;
    LegControl_Init();
    // --- DM 电机初始化（从 Data Task 移入） ---
    DM_Motor_Init(&DM_8009P1, 0x051, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_8009P2, 0x052, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_8009P3, 0x053, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_8009P4, 0x054, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_3519L, 0x055, DM_MOTOR_MODE_MIT, &hfdcan2);
    DM_Motor_Init(&DM_3519R, 0x056, DM_MOTOR_MODE_MIT, &hfdcan2);
    DM_Motor_MIT_Control(&DM_8009P1, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_8009P2, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_8009P4, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_8009P3, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_3519L, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    DM_Motor_MIT_Control(&DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    
    WheelKalman_Init(&Wheel_Kalman_L, 0.001f, 0.0f, 0.0f);
    WheelKalman_Init(&Wheel_Kalman_R, 0.001f, 0.0f, 0.0f);

    // 串口 DMA 接收初始化（从 Data Task 移入）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE);
    if (huart5.hdmarx != nullptr) {
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }

    // uart_print("CaculateTask start\r\n");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_cal_tick = 0;  // 使用 htim5 微秒计数器跟踪真实 dt
    /* Calculate 只读取这三份队列快照，不直接读取 ISR/解析任务正在写的
     * 成品数据。队列均为深度 1；本轮没有新快照时继续使用上一次完整副本。 */
    IMU_ControlSnapshot_t imu_snapshot = {};
    Remote_ControlSnapshot_t remote_snapshot = {};
    Motor_ControlSnapshot_t motor_snapshot = {};
    bool motor_fault_reenable_sent = false;
    while (1) {
        IMU_ControlSnapshot_t received_imu;
        Remote_ControlSnapshot_t received_remote;
        Motor_ControlSnapshot_t received_motor;

        if ((IMU_ControlQueueHandle != NULL) &&
            (xQueueReceive(IMU_ControlQueueHandle, &received_imu, 0) == pdPASS)) {
            imu_snapshot = received_imu;
        }
        if ((Remote_ControlQueueHandle != NULL) &&
            (xQueueReceive(Remote_ControlQueueHandle, &received_remote, 0) == pdPASS)) {
            remote_snapshot = received_remote;
        }
        if ((Motor_ControlQueueHandle != NULL) &&
            (xQueueReceive(Motor_ControlQueueHandle, &received_motor, 0) == pdPASS)) {
            motor_snapshot = received_motor;
        }

        /* Classic_Data 从这里起由 Calculate 持有并使用本周期输入快照。 */
        pClassicData->IMU_Data = imu_snapshot.data;
        pClassicData->Target_Data.Target_X_vel = remote_snapshot.target_x_vel;
        pClassicData->Target_Data.Target_yaw_vel = remote_snapshot.target_yaw_vel;
        pClassicData->Contronller_Data.total_enable =
                (remote_snapshot.sbus_data.Channel[9] >= 100) ? 1U : 0U;

        /* 总使能关闭时，无论本轮随后等到完整反馈、超时，还是走测试保持分支，
         * F0 状态都不能保留上一拍的值。 */
        if (pClassicData->Contronller_Data.total_enable == 0U) {
            pClassicData->Contronller_Data.F0_L = 0.0f;
            pClassicData->Contronller_Data.F0_R = 0.0f;
        }
        /* 台架腿长 PID 测试：关闭任务心跳调试串口输出。 */
        // static TickType_t xLastAlivePrint = 0;
        // TickType_t xAliveNow = xTaskGetTickCount();
        // if ((xAliveNow - xLastAlivePrint) > pdMS_TO_TICKS(1000)) {
        //     uart_print_blocking("CAL_ALIVE\r\n");
        //     xLastAlivePrint = xAliveNow;
        // }
        /* 固定 1 ms 调度：不等待反馈事件。四类统一状态机任一异常时，
         * 本轮全部底盘输出清零：八电机反馈超时、IMU 超时、遥控超时，
         * 或八电机任一驱动状态码故障。 */
        const bool motor_feedback_fault =
                (dm_motor_feedback_timeout_flag != 0U) ||
                (motor_snapshot.driver_fault_flag != 0U);
        if (motor_feedback_fault ||
            (imu_data_timeout_flag != 0U) ||
            (remote_data_timeout_flag != 0U)) {
            /* 同一段电机故障只尝试恢复一次，不能在 1 ms 控制周期反复刷使能帧。 */
            if (motor_feedback_fault && !motor_fault_reenable_sent) {
                DM_Motor_EnableAllMonitored();
                motor_fault_reenable_sent = true;
            }
            pClassicData->Contronller_Data.Fw_L = 0.0f;
            pClassicData->Contronller_Data.Fw_R = 0.0f;
            pClassicData->Contronller_Data.Tp_L = 0.0f;
            pClassicData->Contronller_Data.Tp_R = 0.0f;
            pClassicData->Contronller_Data.T1_L = 0.0f;
            pClassicData->Contronller_Data.T2_L = 0.0f;
            pClassicData->Contronller_Data.T1_R = 0.0f;
            pClassicData->Contronller_Data.T2_R = 0.0f;
            xEventGroupSetBits(ControlEventGroup, Calculate_OK_BIT);
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
            continue;
        }
        motor_fault_reenable_sent = false;

        // ====== 计算本周期真实时间步长 (s) ======
        uint32_t now_tick = __HAL_TIM_GET_COUNTER(&htim5);
        float cal_dt = (float)(now_tick - last_cal_tick) * 1e-6f;
        if (cal_dt <= 0.0f || cal_dt > 0.1f) cal_dt = 0.001f;  // 1ms 默认值
        last_cal_tick = now_tick;
        
        // ====== 原 Data Task：读取电机数据 ======
        pClassicData->Motor_Data.L1_Motor_POS =
                motor_snapshot.data[0].position_rad + 3.1415926f;
        pClassicData->Motor_Data.L2_Motor_POS =
                motor_snapshot.data[1].position_rad + 6.2831853f;
        pClassicData->Motor_Data.R1_Motor_POS =
                -motor_snapshot.data[3].position_rad - 3.1415926f;
        pClassicData->Motor_Data.R2_Motor_POS =
                -motor_snapshot.data[2].position_rad;
        pClassicData->Motor_Data.L1_Motor_VEL = motor_snapshot.data[0].velocity_rad_s;
        pClassicData->Motor_Data.L2_Motor_VEL = motor_snapshot.data[1].velocity_rad_s;
        pClassicData->Motor_Data.R1_Motor_VEL = motor_snapshot.data[3].velocity_rad_s;
        pClassicData->Motor_Data.R2_Motor_VEL = motor_snapshot.data[2].velocity_rad_s;
        pClassicData->Motor_Data.Left_Wheel_Motor_VEL =
                -(motor_snapshot.data[4].velocity_rad_s / 15.8f * 19.2f + 0.048f);
        pClassicData->Motor_Data.Right_Wheel_Motor_VEL =
                +(motor_snapshot.data[5].velocity_rad_s / 15.8f * 19.2f + 0.048f);

        /* 本轮四个髋电机反馈已经就绪。先完成正运动学并写入 Leg_Data，
         * 后面的 Kalman、起身判断、LQR、PID 与补偿都只使用本轮同一套腿状态。 */
        VMC_UpdateLegFeedback(pClassicData, &vmc_kinematics, cal_dt);

        pClassicData->States_Data.d_yaw_err =
                pClassicData->Target_Data.Target_yaw_vel + pClassicData->IMU_Data.delta_yaw;//计算yaw速度误差
        pClassicData->States_Data.yaw_err +=
                (pClassicData->States_Data.d_yaw_err) / 1000.f;//计算目标yaw角度误差
        if (pClassicData->States_Data.yaw_err > 3.1415926f)
          pClassicData->States_Data.yaw_err = 3.1415926f;
        if (pClassicData->States_Data.yaw_err < -3.1415926f)
          pClassicData->States_Data.yaw_err = -3.1415926f;//限制yaw的LQR角度差范围
        pClassicData->States_Data.theta_L =
                +1.5708f - pClassicData->Leg_Data.phi_0_L - pClassicData->IMU_Data.pitch;//（左腿）将相对于vmc坐标系的腿角度转化为相对于地面的供LQR使用
        pClassicData->States_Data.d_theta_L = -pClassicData->Leg_Data.delta_phi_0_L -
                                              pClassicData->IMU_Data.delta_pitch;//角速度
        pClassicData->States_Data.theta_R =
                +1.5708f - pClassicData->Leg_Data.phi_0_R - pClassicData->IMU_Data.pitch;//（右腿）将相对于vmc坐标系的腿角度转化为相对于地面的供LQR使用
        pClassicData->States_Data.d_theta_R = -pClassicData->Leg_Data.delta_phi_0_R -
                                              pClassicData->IMU_Data.delta_pitch;//角速度
        pClassicData->States_Data.theta = pClassicData->IMU_Data.pitch;//车身pitch角度
        pClassicData->States_Data.d_theta = pClassicData->IMU_Data.delta_pitch;//车身pitch角速度

        /* theta_L / theta_R 二阶导（角加速度）：后向差分 + 一阶低通，使用定时器 dt */
        {
            float inv_dt = 1.0f / cal_dt;
            float raw_dd_theta_L = (pClassicData->States_Data.d_theta_L
                                  - pClassicData->States_Data.d_last_d_theta_L) * inv_dt;
            pClassicData->States_Data.dd_theta_L =
                    a * raw_dd_theta_L + (1 - a) * pClassicData->States_Data.dd_theta_L;
            pClassicData->States_Data.d_last_d_theta_L = pClassicData->States_Data.d_theta_L;

            float raw_dd_theta_R = (pClassicData->States_Data.d_theta_R
                                  - pClassicData->States_Data.d_last_d_theta_R) * inv_dt;
            pClassicData->States_Data.dd_theta_R =
                    a * raw_dd_theta_R + (1 - a) * pClassicData->States_Data.dd_theta_R;
            pClassicData->States_Data.d_last_d_theta_R = pClassicData->States_Data.d_theta_R;
        }
        //二阶导腿角加速度使用于离地检测
        
        /* Kalman 的控制输入必须是带方向的前向加速度；accel_h 是幅值，
         * 会在静止/转向时持续向位置状态注入正向偏置。 */
        //上一次预测加速度，周期时间间隔，腿角，腿角速度，腿长，腿长变化速度，车身pitch角速度，轮毂电机角速度预测车位置速度
        WheelKalman_Update(&Wheel_Kalman_L, pClassicData->IMU_Data.accel_x, cal_dt, -pClassicData->States_Data.theta_L, -pClassicData->States_Data.d_theta_L, pClassicData->Leg_Data.L0_L, pClassicData->Leg_Data.d_L0_L, pClassicData->IMU_Data.delta_pitch,pClassicData->Motor_Data.Left_Wheel_Motor_VEL);
        WheelKalman_Update(&Wheel_Kalman_R, pClassicData->IMU_Data.accel_x, cal_dt, -pClassicData->States_Data.theta_R, -pClassicData->States_Data.d_theta_R, pClassicData->Leg_Data.L0_R, pClassicData->Leg_Data.d_L0_R, pClassicData->IMU_Data.delta_pitch,pClassicData->Motor_Data.Right_Wheel_Motor_VEL);

        /* Kalman 校正完成后再生成公共底盘状态，供下游状态监视使用。 */
        pClassicData->States_Data.d_X =
                (Wheel_Kalman_L.x[1] + Wheel_Kalman_R.x[1]) * 0.5f; // 算出平均车身速度
        Pos_INT += pClassicData->Target_Data.Target_X_vel / 1000.0f;//算出车身位移目标值（只有目标速度没输入的时候才会保持不变维持车身位置）
        pClassicData->States_Data.X_pos =
                (Wheel_Kalman_L.x[0] + Wheel_Kalman_R.x[0]) * 0.5f - Pos_INT;//计算车身位移误差
        pClassicData->States_Data.d_x_err =
                pClassicData->States_Data.d_X - pClassicData->Target_Data.Target_X_vel;//计算速度误差
        
        // ====== 起身状态机（左右腿共用一个状态）======
        const float theta_l_abs = fabsf(pClassicData->States_Data.theta_L);
        const float theta_r_abs = fabsf(pClassicData->States_Data.theta_R);
        /* 两腿靠近竖直才起身，避免收腿经过中间角度时提前切换支撑力。 */
        const bool both_legs_up = (theta_l_abs < 0.4f) && (theta_r_abs < 0.4f);
        const bool either_leg_not_up = (theta_l_abs > 1.2f) || (theta_r_abs > 1.2f);
        if (both_legs_up)
        {
          stand_state = StandState_Up;
          stand_not_up_time = 0.0f;
        }
        else if (either_leg_not_up && stand_state != StandState_NotUp)
        {
          stand_not_up_time += cal_dt;
          if (stand_not_up_time >= 1.0f)
          {
            stand_state = StandState_NotUp;
            stand_not_up_time = 1.0f;
            }
        }
        /* 测试版：固定按已起身处理，独立于总使能开关；正式版使用原起身状态机。
         * 腿长 PID 是否输出仍由后面的 Channel[9] 总使能控制。 */
#if LEG_CONTROL_TEST_MODE
        StandUp_Flag = true;
#else
        StandUp_Flag = (stand_state == StandState_Up);
#endif

        // WheelKalman_Print(&Wheel_Kalman_L, 'L');
        // WheelKalman_Print(&Wheel_Kalman_R, 'R');
        //离地检测：识别腿支持力
        float Fn_Left = Ground_Clearance_Detection(pClassicData->Contronller_Data.F0_L,pClassicData->Contronller_Data.Tp_L, pClassicData->Leg_Data.L0_L,pClassicData->Leg_Data.d_L0_L,pClassicData->Leg_Data.dd_L0_L,pClassicData->IMU_Data.accel_v,pClassicData->States_Data.theta_L,pClassicData->States_Data.d_theta_L,pClassicData->States_Data.dd_theta_L);
        float Fn_Right = Ground_Clearance_Detection(pClassicData->Contronller_Data.F0_R,pClassicData->Contronller_Data.Tp_R, pClassicData->Leg_Data.L0_R,pClassicData->Leg_Data.d_L0_R,pClassicData->Leg_Data.dd_L0_R,pClassicData->IMU_Data.accel_v,pClassicData->States_Data.theta_R,pClassicData->States_Data.d_theta_R,pClassicData->States_Data.dd_theta_R);
        (void)Fn_Left;
        (void)Fn_Right;

        // ====== LQR + 腿长 PID + roll 差分补偿（phi PID 当前关闭） ======
        /* LQR 仅在遥控器开关打开时运行；关闭时清零，避免沿用旧输出。 */
        pClassicData->Contronller_Data.Fw_L = 0.0f;
        pClassicData->Contronller_Data.Fw_R = 0.0f;
        pClassicData->Contronller_Data.Tp_L = 0.0f;
        pClassicData->Contronller_Data.Tp_R = 0.0f;

        /* 左右腿按各自实际 L0 查询实车定义下的角度目标（rad）。 */
        theta_L_ref = LegAngleTarget_GetTheta(pClassicData->Leg_Data.L0_L);
        theta_R_ref = LegAngleTarget_GetTheta(pClassicData->Leg_Data.L0_R);

        const bool lqr_enabled = (remote_snapshot.sbus_data.Channel[7] > 100);
        const bool leg_pid_enabled = (remote_snapshot.sbus_data.Channel[9] >= 100);
        /* LQR 起身后以当前位姿作为位置/yaw 工作点。 */
        const bool pos_ref_ready = leg_pid_enabled && lqr_enabled && StandUp_Flag;
        if (pos_ref_ready && !lqr_pos_ref_recorded) {
            Pos_INT = (Wheel_Kalman_L.x[0] + Wheel_Kalman_R.x[0]) * 0.5f;
            pClassicData->States_Data.X_pos = 0.0f;
            pClassicData->States_Data.yaw_err = 0.0f;
            lqr_pos_ref_recorded = true;
        } else if (!pos_ref_ready) {
            lqr_pos_ref_recorded = false;
        }

        /* LQR 仅由 Channel[7] 控制。正式版用左右实际腿长均值查表；
         * 右腿标定测试完全隔离，只使用右腿实际 L0_R。 */
        if (lqr_enabled) {
#if LEG_CONTROL_TEST_MODE
            const float scheduled_leg_length = pClassicData->Leg_Data.L0_R;
#else
            const float average_leg_length =
                    (pClassicData->Leg_Data.L0_L + pClassicData->Leg_Data.L0_R) * 0.5f;
            const float scheduled_leg_length = average_leg_length;
#endif
            K_Fixed_Leg_t scheduled_gain;
            LQR_K_GetInterpolated(scheduled_leg_length, scheduled_gain);
            LQR_K_CalculateWithGain(pClassicData, scheduled_gain,
                                    theta_ref, d_theta_ref,
                                    theta_L_ref, d_theta_L_ref,
                                    theta_R_ref, d_theta_R_ref);
        }

        /* 测试版不叠加腿重力补偿；正式版总使能且已起身后叠加。
         * 任一模式下，总使能关闭时 ControlTask 均向全部电机发送零力矩。 */
        float gravity_tp_l = 0.0f;
        float gravity_tp_r = 0.0f;
#if LEG_GRAVITY_COMPENSATION_ENABLE && !LEG_CONTROL_TEST_MODE
        if (leg_pid_enabled && StandUp_Flag) {
            gravity_tp_l = LegGravity_GetTp(pClassicData->Leg_Data.L0_L,
                                            pClassicData->States_Data.theta_L);
            gravity_tp_r = LegGravity_GetTp(pClassicData->Leg_Data.L0_R,
                                            pClassicData->States_Data.theta_R);
        }
#endif

        LegControl_Update(pClassicData, cal_dt, StandUp_Flag, leg_pid_enabled,
                          remote_snapshot.sbus_data.Channel[5]);
       
        // ====== 输入限幅：VMC 之前，F0/Tp 必须在此限幅（Fw 不经过 VMC） ======
        /* 台架气弹簧预压测试：总使能前总 F0 为 ±10 N，使能后为 ±50 N。
         * 保留原起身前/后 F0、Tp 限幅逻辑，测试结束时恢复。 */
        // if(StandUp_Flag == true)
        // {
        // Max_F0 = 30.0f;
        // Max_Tp = 2.0f;
        // }
        // else
        // {
        // Max_F0 = 60.0f;
        // Max_Tp = 4.0f;
        // };
        Max_F0 = leg_pid_enabled ? 50.0f : 0.0f;
        Max_Tp = 2.0f;

        // F0 总限幅：总使能前 ±10 N；总使能后 ±50 N。
        if (pClassicData->Contronller_Data.F0_L > Max_F0)
            pClassicData->Contronller_Data.F0_L = Max_F0;
        if (pClassicData->Contronller_Data.F0_L < -Max_F0)
            pClassicData->Contronller_Data.F0_L = -Max_F0;
        if (pClassicData->Contronller_Data.F0_R > Max_F0)
            pClassicData->Contronller_Data.F0_R = Max_F0;
        if (pClassicData->Contronller_Data.F0_R < -Max_F0)
            pClassicData->Contronller_Data.F0_R = -Max_F0;

        if (pClassicData->Contronller_Data.Tp_L > Max_Tp)
            pClassicData->Contronller_Data.Tp_L = Max_Tp;
        if (pClassicData->Contronller_Data.Tp_L < -Max_Tp)
            pClassicData->Contronller_Data.Tp_L = -Max_Tp;
        if (pClassicData->Contronller_Data.Tp_R > Max_Tp)
            pClassicData->Contronller_Data.Tp_R = Max_Tp;
        if (pClassicData->Contronller_Data.Tp_R < -Max_Tp)
            pClassicData->Contronller_Data.Tp_R = -Max_Tp;
        // ====== VMC 力矩分配 ======
        VMC_MapForceToJointTorques(
                pClassicData, &vmc_kinematics,
                pClassicData->Contronller_Data.F0_L,
                pClassicData->Contronller_Data.F0_R,
                pClassicData->Contronller_Data.Tp_L + gravity_tp_l,
                pClassicData->Contronller_Data.Tp_R + gravity_tp_r);

        // ====== 输出限幅：VMC 之后，T1/T2 限幅 ======
        Controler_Limit(pClassicData);

        {
            /* 原控制周期末尾调试选择：默认不输出。 */
            static TickType_t xLastPrintTick = 0;
            TickType_t xNow = xTaskGetTickCount();
            if ((xNow - xLastPrintTick) > pdMS_TO_TICKS(100)) {
                /* 台架打印选择：一次只启用一种，避免不同字段混在同一 CSV 流。 */
                // Motor_Data_Print(pClassicData);          // 四髋电机角度/速度 + 左右轮速
                // LEG_Data_Print(pClassicData);          // 腿长、腿长速度、腿角、腿角速度
                 LegLengthPID_Data_Print(pClassicData); // 旋钮目标腿长、实际腿长、误差、F0
                // IMU_Data_Print(pClassicData);
                // State_Data_Print(pClassicData);
                // WheelKalman_Print(&Wheel_Kalman_L, 'L'); // 左侧：原始位置、原始速度、滤波位置、滤波速度
                // WheelKalman_Print(&Wheel_Kalman_R, 'R'); // 右侧：原始位置、原始速度、滤波位置、滤波速度
                xLastPrintTick = xNow;
            }

        }
        xEventGroupSetBits(ControlEventGroup, Calculate_OK_BIT);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

#if 0 /* VMC implementation moved to APP/src/app_vmc.cpp. */
static VMC_LegKinematics_t VMC_ForwardLeg(float phi1, float phi4) {
    constexpr float l1 = 0.210f;
    constexpr float l2 = 0.250f;
    constexpr float l3 = 0.250f;
    constexpr float l4 = 0.210f;
    VMC_LegKinematics_t leg = {};
    leg.phi1 = phi1;
    leg.phi4 = phi4;
    const float yd = l4 * arm_sin_f32(phi4);
    const float yb = l1 * arm_sin_f32(phi1);
    const float xd = l4 * arm_cos_f32(phi4);
    const float xb = l1 * arm_cos_f32(phi1);
    const float dx = xd - xb;
    const float dy = yd - yb;
    float lbd;
    arm_sqrt_f32(dx * dx + dy * dy, &lbd);
    const float a0 = 2.0f * l2 * dx;
    const float b0 = 2.0f * l2 * dy;
    const float c0 = l2 * l2 + lbd * lbd - l3 * l3;
    float root;
    arm_sqrt_f32(a0 * a0 + b0 * b0 - c0 * c0, &root);
    leg.phi2 = 2.0f * atan2f(b0 + root, a0 + c0);
    leg.phi3 = atan2f(yb - yd + l2 * arm_sin_f32(leg.phi2),
                      xb - xd + l2 * arm_cos_f32(leg.phi2));
    const float xc = l1 * arm_cos_f32(phi1) + l2 * arm_cos_f32(leg.phi2);
    const float yc = l1 * arm_sin_f32(phi1) + l2 * arm_sin_f32(leg.phi2);
    arm_sqrt_f32(xc * xc + yc * yc, &leg.L0);
    leg.phi0 = atan2f(yc, xc);
    return leg;
}

static void VMC_UpdateLegFeedback(Classic_Data_t *data,
                                  VMC_Kinematics_t *kinematics, float dt) {
    const float inv_dt = 1.0f / dt;
    kinematics->left = VMC_ForwardLeg(data->Motor_Data.L1_Motor_POS,
                                      data->Motor_Data.L2_Motor_POS);
    kinematics->right = VMC_ForwardLeg(data->Motor_Data.R1_Motor_POS,
                                       data->Motor_Data.R2_Motor_POS);
    VMC_LegKinematics_t *legs[2] = {&kinematics->left, &kinematics->right};
    float *l0[2] = {&data->Leg_Data.L0_L, &data->Leg_Data.L0_R};
    float *phi0[2] = {&data->Leg_Data.phi_0_L, &data->Leg_Data.phi_0_R};
    float *dl0[2] = {&data->Leg_Data.d_L0_L, &data->Leg_Data.d_L0_R};
    float *ddl0[2] = {&data->Leg_Data.dd_L0_L, &data->Leg_Data.dd_L0_R};
    float *last_dl0[2] = {&data->Leg_Data.d_last_L0_L, &data->Leg_Data.d_last_L0_R};
    float *dphi0[2] = {&data->Leg_Data.delta_phi_0_L, &data->Leg_Data.delta_phi_0_R};
    static float last_l0[2] = {0.0f, 0.0f};
    static float last_phi0[2] = {0.0f, 0.0f};
    for (uint32_t side = 0; side < 2U; ++side) {
        *l0[side] = legs[side]->L0;
        *phi0[side] = legs[side]->phi0;
        const float raw_dl0 = (legs[side]->L0 - last_l0[side]) * inv_dt;
        *dl0[side] = a * raw_dl0 + (1.0f - a) * *dl0[side];
        const float raw_ddl0 = (*dl0[side] - *last_dl0[side]) * inv_dt;
        *ddl0[side] = a * raw_ddl0 + (1.0f - a) * *ddl0[side];
        *last_dl0[side] = *dl0[side];
        /* 保持原来的腿角速度定义；本次仅调整正运动学与控制的先后顺序。 */
        const float raw_dphi0 = (legs[side]->phi0 - last_phi0[side]) * 1000.0f;
        *dphi0[side] = a * raw_dphi0 + (1.0f - a) * *dphi0[side];
        last_l0[side] = legs[side]->L0;
        last_phi0[side] = legs[side]->phi0;
    }
}

static void VMC_MapForceToJointTorques(Classic_Data_t *data,
                                       const VMC_Kinematics_t *kinematics,
                                       float F0L, float F0R, float TpL,
                                       float TpR) {
    constexpr float l1 = 0.210f;
    constexpr float l4 = 0.210f;
    const VMC_LegKinematics_t *legs[2] = {&kinematics->left, &kinematics->right};
    const float f0[2] = {F0L, F0R};
    const float tp[2] = {TpL, TpR};
    float *t1[2] = {&data->Contronller_Data.T1_L, &data->Contronller_Data.T1_R};
    float *t2[2] = {&data->Contronller_Data.T2_L, &data->Contronller_Data.T2_R};
    for (uint32_t side = 0; side < 2U; ++side) {
        const VMC_LegKinematics_t *leg = legs[side];
        const float sin32 = arm_sin_f32(leg->phi3 - leg->phi2);
        const float sin12 = arm_sin_f32(leg->phi1 - leg->phi2);
        const float sin03 = arm_sin_f32(leg->phi0 - leg->phi3);
        const float cos03 = arm_cos_f32(leg->phi0 - leg->phi3);
        const float sin02 = arm_sin_f32(leg->phi0 - leg->phi2);
        const float cos02 = arm_cos_f32(leg->phi0 - leg->phi2);
        const float sin34 = arm_sin_f32(leg->phi3 - leg->phi4);
        const float j11 = l1 * sin03 * sin12 / sin32;
        const float j12 = l1 * cos03 * sin12 / (leg->L0 * sin32);
        const float j21 = l4 * sin02 * sin34 / sin32;
        const float j22 = l4 * cos02 * sin34 / (leg->L0 * sin32);
        *t1[side] = j11 * f0[side] + j12 * tp[side];
        *t2[side] = j21 * f0[side] + j22 * tp[side];
    }
    data->Contronller_Data.Tp_L = TpL;
    data->Contronller_Data.Tp_R = TpR;
}
#endif

/**
 * @brief 将角度（度）转换为弧度
 * @param degrees 输入角度值（度）
 * @return float 转换后的弧度值
 */
float angle_to_rad(float degrees) { return degrees * (PI / 180.0f); }

void LEG_Data_Print(Classic_Data_t *pClassicData) {
    /* FireWater 纯 CSV，依次为：
     * L0_L,L0_R,dL0_L,dL0_R,phi0_L,phi0_R,dphi0_L,dphi0_R。
     * 单位依次为 m,m,m/s,m/s,rad,rad,rad/s,rad/s。 */
    const float values[8] = {
        pClassicData->Leg_Data.L0_L,
        pClassicData->Leg_Data.L0_R,
        pClassicData->Leg_Data.d_L0_L,
        pClassicData->Leg_Data.d_L0_R,
        pClassicData->Leg_Data.phi_0_L,
        pClassicData->Leg_Data.phi_0_R,
        pClassicData->Leg_Data.delta_phi_0_L,
        pClassicData->Leg_Data.delta_phi_0_R,
    };
    int milli[8];
    for (uint32_t index = 0; index < 8U; ++index)
        milli[index] = (int)lroundf(values[index] * 1000.0f);

    uart_print("%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,"
               "%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d\r\n",
               milli[0] < 0 ? '-' : ' ', abs(milli[0]) / 1000, abs(milli[0]) % 1000,
               milli[1] < 0 ? '-' : ' ', abs(milli[1]) / 1000, abs(milli[1]) % 1000,
               milli[2] < 0 ? '-' : ' ', abs(milli[2]) / 1000, abs(milli[2]) % 1000,
               milli[3] < 0 ? '-' : ' ', abs(milli[3]) / 1000, abs(milli[3]) % 1000,
               milli[4] < 0 ? '-' : ' ', abs(milli[4]) / 1000, abs(milli[4]) % 1000,
               milli[5] < 0 ? '-' : ' ', abs(milli[5]) / 1000, abs(milli[5]) % 1000,
               milli[6] < 0 ? '-' : ' ', abs(milli[6]) / 1000, abs(milli[6]) % 1000,
               milli[7] < 0 ? '-' : ' ', abs(milli[7]) / 1000, abs(milli[7]) % 1000);
}



void Motor_Data_Print(Classic_Data_t *pClassicData) {
    /* FireWater 纯 CSV，依次为：
     * phi1_L,phi4_L,phi1_R,phi4_R,
     * dphi1_L,dphi4_L,dphi1_R,dphi4_R,wheelVel_L,wheelVel_R,
     * rawP1,rawP2,rawP4,rawP3。
     * 前四列为送入 VMC 的映射角 rad；第 5~8 列为髋电机原始速度
     * rad/s；第 9~10 列为处理后的轮速 rad/s；最后四列为达妙直接
     * 回传的原始髋电机角度 rad。所有髋电机均按左后、左前、右后、
     * 右前顺序排列。 */
    const float values[14] = {
        pClassicData->Motor_Data.L1_Motor_POS,
        pClassicData->Motor_Data.L2_Motor_POS,
        pClassicData->Motor_Data.R1_Motor_POS,
        pClassicData->Motor_Data.R2_Motor_POS,
        pClassicData->Motor_Data.L1_Motor_VEL,
        pClassicData->Motor_Data.L2_Motor_VEL,
        pClassicData->Motor_Data.R1_Motor_VEL,
        pClassicData->Motor_Data.R2_Motor_VEL,
        pClassicData->Motor_Data.Left_Wheel_Motor_VEL,
        pClassicData->Motor_Data.Right_Wheel_Motor_VEL,
        pClassicData->Motor_Data.L1_Motor_POS - 3.1415926f,
        pClassicData->Motor_Data.L2_Motor_POS - 6.2831853f,
        -pClassicData->Motor_Data.R1_Motor_POS - 3.1415926f,
        -pClassicData->Motor_Data.R2_Motor_POS,
    };
    int milli[14];
    for (uint32_t index = 0; index < 14U; ++index)
        milli[index] = (int)lroundf(values[index] * 1000.0f);

    uart_print("%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,"
               "%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,"
               "%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,"
               "%c%d.%03d,%c%d.%03d\r\n",
               milli[0] < 0 ? '-' : ' ', abs(milli[0]) / 1000, abs(milli[0]) % 1000,
               milli[1] < 0 ? '-' : ' ', abs(milli[1]) / 1000, abs(milli[1]) % 1000,
               milli[2] < 0 ? '-' : ' ', abs(milli[2]) / 1000, abs(milli[2]) % 1000,
               milli[3] < 0 ? '-' : ' ', abs(milli[3]) / 1000, abs(milli[3]) % 1000,
               milli[4] < 0 ? '-' : ' ', abs(milli[4]) / 1000, abs(milli[4]) % 1000,
               milli[5] < 0 ? '-' : ' ', abs(milli[5]) / 1000, abs(milli[5]) % 1000,
               milli[6] < 0 ? '-' : ' ', abs(milli[6]) / 1000, abs(milli[6]) % 1000,
               milli[7] < 0 ? '-' : ' ', abs(milli[7]) / 1000, abs(milli[7]) % 1000,
               milli[8] < 0 ? '-' : ' ', abs(milli[8]) / 1000, abs(milli[8]) % 1000,
               milli[9] < 0 ? '-' : ' ', abs(milli[9]) / 1000, abs(milli[9]) % 1000,
               milli[10] < 0 ? '-' : ' ', abs(milli[10]) / 1000, abs(milli[10]) % 1000,
               milli[11] < 0 ? '-' : ' ', abs(milli[11]) / 1000, abs(milli[11]) % 1000,
               milli[12] < 0 ? '-' : ' ', abs(milli[12]) / 1000, abs(milli[12]) % 1000,
               milli[13] < 0 ? '-' : ' ', abs(milli[13]) / 1000, abs(milli[13]) % 1000);
}

void IMU_Data_Print(Classic_Data_t *pClassicData) {
    /* FireWater: 纯 CSV，依次为 roll,pitch,yaw,droll,dpitch,dyaw。 */
    const int value_milli[6] = {
        (int)lroundf(pClassicData->IMU_Data.roll * 1000.0f),
        (int)lroundf(pClassicData->IMU_Data.pitch * 1000.0f),
        (int)lroundf(pClassicData->IMU_Data.yaw * 1000.0f),
        (int)lroundf(pClassicData->IMU_Data.delta_roll * 1000.0f),
        (int)lroundf(pClassicData->IMU_Data.delta_pitch * 1000.0f),
        (int)lroundf(pClassicData->IMU_Data.delta_yaw * 1000.0f),
    };
    uart_print("%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d\r\n",
               value_milli[0] < 0 ? '-' : ' ', abs(value_milli[0]) / 1000, abs(value_milli[0]) % 1000,
               value_milli[1] < 0 ? '-' : ' ', abs(value_milli[1]) / 1000, abs(value_milli[1]) % 1000,
               value_milli[2] < 0 ? '-' : ' ', abs(value_milli[2]) / 1000, abs(value_milli[2]) % 1000,
               value_milli[3] < 0 ? '-' : ' ', abs(value_milli[3]) / 1000, abs(value_milli[3]) % 1000,
               value_milli[4] < 0 ? '-' : ' ', abs(value_milli[4]) / 1000, abs(value_milli[4]) % 1000,
               value_milli[5] < 0 ? '-' : ' ', abs(value_milli[5]) / 1000, abs(value_milli[5]) % 1000);
}

void LegLengthPID_Data_Print(Classic_Data_t *pClassicData) {
    /* FireWater 纯 CSV，单位依次为 m,m,m,m,m,N,N：
     * VrB 映射目标L0、左L0、右L0、左误差、右误差、左F0、右F0。 */
    const float target = pClassicData->Target_Data.Target_LegLong;
    const float values[7] = {
        target,
        pClassicData->Leg_Data.L0_L,
        pClassicData->Leg_Data.L0_R,
        target - pClassicData->Leg_Data.L0_L,
        target - pClassicData->Leg_Data.L0_R,
        pClassicData->Contronller_Data.F0_L,
        pClassicData->Contronller_Data.F0_R,
    };
    int milli[7];
    for (uint32_t index = 0; index < 7U; ++index)
        milli[index] = (int)lroundf(values[index] * 1000.0f);

    uart_print("%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,"
               "%c%d.%03d,%c%d.%03d\r\n",
               milli[0] < 0 ? '-' : ' ', abs(milli[0]) / 1000, abs(milli[0]) % 1000,
               milli[1] < 0 ? '-' : ' ', abs(milli[1]) / 1000, abs(milli[1]) % 1000,
               milli[2] < 0 ? '-' : ' ', abs(milli[2]) / 1000, abs(milli[2]) % 1000,
               milli[3] < 0 ? '-' : ' ', abs(milli[3]) / 1000, abs(milli[3]) % 1000,
               milli[4] < 0 ? '-' : ' ', abs(milli[4]) / 1000, abs(milli[4]) % 1000,
                milli[5] < 0 ? '-' : ' ', abs(milli[5]) / 1000, abs(milli[5]) % 1000,
                milli[6] < 0 ? '-' : ' ', abs(milli[6]) / 1000, abs(milli[6]) % 1000);
}

void State_Data_Print(Classic_Data_t *pClassicData) {
    // 计算缩放后的绝对值
    float x_pos_abs = 0.0f;
    float d_x_abs = 0.0f;
    float yaw_abs = 0.0f;
    float d_yaw_abs = 0.0f;
    float theta_l_abs = 0.0f;
    float d_theta_l_abs = 0.0f;
    float theta_r_abs = 0.0f;
    float d_theta_r_abs = 0.0f;
    float theta_abs = 0.0f;
    float d_theta_abs = 0.0f;
    float dx_err = pClassicData->States_Data.d_X - pClassicData->Target_Data.Target_X_vel;
    // 使用ARMmath库的绝对值函数
    arm_abs_f32(&pClassicData->States_Data.X_pos, &x_pos_abs, 1);
    arm_abs_f32(&dx_err, &d_x_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.yaw_err, &yaw_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.d_yaw_err, &d_yaw_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.theta_L, &theta_l_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.d_theta_L, &d_theta_l_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.theta_R, &theta_r_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.d_theta_R, &d_theta_r_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.theta, &theta_abs, 1);
    arm_abs_f32(&pClassicData->States_Data.d_theta, &d_theta_abs, 1);

    // 计算每个状态的整数部分
    int x_pos_int = (int) (pClassicData->States_Data.X_pos * 1000) / 1000;
    int d_x_int = (int) ((pClassicData->States_Data.d_X - pClassicData->Target_Data.Target_X_vel) * 1000) / 1000;
    int yaw_int = (int) (pClassicData->States_Data.yaw_err * 1000) / 1000;
    int d_yaw_int = (int) (pClassicData->States_Data.d_yaw_err * 1000) / 1000;
    int theta_l_int = (int) (pClassicData->States_Data.theta_L * 1000) / 1000;
    int d_theta_l_int = (int) (pClassicData->States_Data.d_theta_L * 1000) / 1000;
    int theta_r_int = (int) (pClassicData->States_Data.theta_R * 1000) / 1000;
    int d_theta_r_int = (int) (pClassicData->States_Data.d_theta_R * 1000) / 1000;
    int theta_int = (int) (pClassicData->States_Data.theta * 1000) / 1000;
    int d_theta_int = (int) (pClassicData->States_Data.d_theta * 1000) / 1000;

    // 计算每个状态的小数部分
    int x_pos_dec = (int) (x_pos_abs * 1000) % 1000;
    int d_x_dec = (int) (d_x_abs * 1000) % 1000;
    int yaw_dec = (int) (yaw_abs * 1000) % 1000;
    int d_yaw_dec = (int) (d_yaw_abs * 1000) % 1000;
    int theta_l_dec = (int) (theta_l_abs * 1000) % 1000;
    int d_theta_l_dec = (int) (d_theta_l_abs * 1000) % 1000;
    int theta_r_dec = (int) (theta_r_abs * 1000) % 1000;
    int d_theta_r_dec = (int) (d_theta_r_abs * 1000) % 1000;
    int theta_dec = (int) (theta_abs * 1000) % 1000;
    int d_theta_dec = (int) (d_theta_abs * 1000) % 1000;

    /* 后四列统一放在 State_Data_Print 的 CSV 行中，单位为度：
     * P1/P2raw 是达妙直接反馈；phi1/phi4 是加偏置后送入 VMC 的角。 */
    const float rad_to_mdeg = 180000.0f / PI;
    const int p1_raw_mdeg = (int)lroundf(
            (pClassicData->Motor_Data.L1_Motor_POS - 3.1415926f) * rad_to_mdeg);
    const int p2_raw_mdeg = (int)lroundf(
            (pClassicData->Motor_Data.L2_Motor_POS - 6.2831853f) * rad_to_mdeg);
    const int phi1_mdeg = (int)lroundf(pClassicData->Motor_Data.L1_Motor_POS * rad_to_mdeg);
    const int phi4_mdeg = (int)lroundf(pClassicData->Motor_Data.L2_Motor_POS * rad_to_mdeg);

    // 处理符号：当整数部分为0但原始值为负时，手动输出负号
    char x_pos_sign =
            (x_pos_int == 0 && pClassicData->States_Data.X_pos < 0.0f) ? '-' : ' ';
    char d_x_sign =
            (d_x_int == 0 && dx_err < 0.0f) ? '-' : ' ';
    char yaw_sign =
            (yaw_int == 0 && pClassicData->States_Data.yaw_err < 0.0f) ? '-' : ' ';
    char d_yaw_sign =
            (d_yaw_int == 0 && pClassicData->States_Data.d_yaw_err < 0.0f) ? '-' : ' ';
    char theta_l_sign =
            (theta_l_int == 0 && pClassicData->States_Data.theta_L < 0.0f)
                ? '-'
                : ' ';
    char d_theta_l_sign =
            (d_theta_l_int == 0 && pClassicData->States_Data.d_theta_L < 0.0f)
                ? '-'
                : ' ';
    char theta_r_sign =
            (theta_r_int == 0 && pClassicData->States_Data.theta_R < 0.0f)
                ? '-'
                : ' ';
    char d_theta_r_sign =
            (d_theta_r_int == 0 && pClassicData->States_Data.d_theta_R < 0.0f)
                ? '-'
                : ' ';
    char theta_sign =
            (theta_int == 0 && pClassicData->States_Data.theta < 0.0f) ? '-' : ' ';
    char d_theta_sign =
            (d_theta_int == 0 && pClassicData->States_Data.d_theta < 0.0f)
                ? '-'
                : ' ';

    // 打印输出，手动处理负号
    uart_print("%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%"
               "c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d\r\n",
               x_pos_sign, x_pos_int, x_pos_dec, d_x_sign, d_x_int, d_x_dec,
               yaw_sign, yaw_int, yaw_dec, d_yaw_sign, d_yaw_int, d_yaw_dec,
               theta_l_sign, theta_l_int, theta_l_dec, d_theta_l_sign,
               d_theta_l_int, d_theta_l_dec, theta_r_sign, theta_r_int,
               theta_r_dec, d_theta_r_sign, d_theta_r_int, d_theta_r_dec,
               theta_sign, theta_int, theta_dec, d_theta_sign, d_theta_int,
               d_theta_dec,
               p1_raw_mdeg < 0 ? '-' : ' ', abs(p1_raw_mdeg) / 1000, abs(p1_raw_mdeg) % 1000,
               p2_raw_mdeg < 0 ? '-' : ' ', abs(p2_raw_mdeg) / 1000, abs(p2_raw_mdeg) % 1000,
               phi1_mdeg < 0 ? '-' : ' ', abs(phi1_mdeg) / 1000, abs(phi1_mdeg) % 1000,
               phi4_mdeg < 0 ? '-' : ' ', abs(phi4_mdeg) / 1000, abs(phi4_mdeg) % 1000);
}

// 固定腿长 K 的状态反馈计算；输出限幅在 VMC 后的 Controler_Limit() 中执行。
#if 0 /* LQR implementation moved to APP/src/app_K_value.cpp. */
static void LQR_K_CalculateWithGain(Classic_Data_t *pClassicData,
                                     K_Fixed_Leg_t gain) {
    /* K 矩阵对应 [x, dx, yaw, dyaw, thetaL, dthetaL,
     * thetaR, dthetaR, pitch, dpitch] 十维状态。 */
    /* 仅限制送入 K 的位置误差，状态记录保留原值。 */
    const float x_pos_lqr = fminf(fmaxf(pClassicData->States_Data.X_pos,
                                        -0.30f), 0.30f);
    float states[10] = {
        x_pos_lqr,
        pClassicData->States_Data.d_x_err,
        pClassicData->States_Data.yaw_err,
        pClassicData->States_Data.d_yaw_err,
        pClassicData->States_Data.theta_L - theta_L_ref,
        pClassicData->States_Data.d_theta_L - d_theta_L_ref,
        pClassicData->States_Data.theta_R - theta_R_ref,
        pClassicData->States_Data.d_theta_R - d_theta_R_ref,
        -(pClassicData->States_Data.theta - theta_ref),
        -(pClassicData->States_Data.d_theta - d_theta_ref)
    };
    float control[4] = {0.0f};
    arm_matrix_instance_f32 K = {4, 10, &gain[0][0]};
    arm_matrix_instance_f32 state = {10, 1, states};
    arm_matrix_instance_f32 output = {4, 1, control};
    arm_mat_mult_f32(&K, &state, &output);

    pClassicData->Contronller_Data.Fw_L = control[0];
    pClassicData->Contronller_Data.Fw_R = control[1];
    pClassicData->Contronller_Data.Tp_L = control[2];
    pClassicData->Contronller_Data.Tp_R = control[3];
}
#endif

#if 0 /* Output limiter moved to APP/src/app_control_limit.cpp. */
void Controler_Limit(Classic_Data_t *pClassicData) {
  // Fw 限幅：±6.0 N（轮子力，不经过 VMC）
  if (pClassicData->Contronller_Data.Fw_L > 6.0f)
    pClassicData->Contronller_Data.Fw_L = 6.0f;
  if (pClassicData->Contronller_Data.Fw_L < -6.0f)
    pClassicData->Contronller_Data.Fw_L = -6.0f;
  if (pClassicData->Contronller_Data.Fw_R > 6.0f)
    pClassicData->Contronller_Data.Fw_R = 6.0f;
  if (pClassicData->Contronller_Data.Fw_R < -6.0f)
    pClassicData->Contronller_Data.Fw_R = -6.0f;
  // T1/T2 限幅：±20.5 N·m
  if (pClassicData->Contronller_Data.T1_L > 20.5f)
    pClassicData->Contronller_Data.T1_L = 20.5f;
  if (pClassicData->Contronller_Data.T1_L < -20.5f)
    pClassicData->Contronller_Data.T1_L = -20.5f;
  if (pClassicData->Contronller_Data.T1_R > 20.5f)
    pClassicData->Contronller_Data.T1_R = 20.5f;
  if (pClassicData->Contronller_Data.T1_R < -20.5f)
    pClassicData->Contronller_Data.T1_R = -20.5f;
  if (pClassicData->Contronller_Data.T2_L > 20.5f)
    pClassicData->Contronller_Data.T2_L = 20.5f;
  if (pClassicData->Contronller_Data.T2_L < -20.5f)
    pClassicData->Contronller_Data.T2_L = -20.5f;
  if (pClassicData->Contronller_Data.T2_R > 20.5f)
    pClassicData->Contronller_Data.T2_R = 20.5f;
  if (pClassicData->Contronller_Data.T2_R < -20.5f)
    pClassicData->Contronller_Data.T2_R = -20.5f;
}
#endif
