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
#include "app_leg_control.hpp"
#include "app_step_climb_demo.hpp"
#include "app_stand_state.hpp"
#include "app_chassis_position_hold.hpp"
#include "app_chassis_yaw_state.hpp"

// 定义参考值（根据实际需求调整）
float theta_ref = 0.0f; // 期望整体角度
float d_theta_ref = 0.0f; // 期望整体角速度
float d_theta_L_ref = 0.0f; // 期望左腿角速度
float d_theta_R_ref = 0.0f; // 期望右腿角速度
float theta_L_ref = 0.0f; // 由左右各自腿长目标表实时更新，默认 0 rad
float theta_R_ref = 0.0f; // 由左右各自腿长目标表实时更新，默认 0 rad

static constexpr float kStateDerivativeFilterAlpha = 0.2f;

/* 仅限制遥控前后速度参考的变化率，不限制 KF 反馈或 LQR 输出。
 * 推杆加速较缓，松杆与反向时优先快速回零，避免操作手给出速度阶跃。 */
static constexpr float kRemoteXVelocityAccelLimitMps2 = 1.5f;
static constexpr float kRemoteXVelocityDecelLimitMps2 = 8.0f;

float angle_to_rad(float degrees);

void LEG_Data_Print(Classic_Data_t *pClassicData);

void Motor_Data_Print(Classic_Data_t *pClassicData);

void IMU_Data_Print(Classic_Data_t *pClassicData);

void State_Data_Print(Classic_Data_t *pClassicData);

/* 健康诊断：放在安全早退之前，只打印状态，不参与放行或输出决策。 */
static void ControlHealth_Data_Print(bool imu_snapshot_updated,
                                     bool remote_snapshot_updated,
                                     bool motor_snapshot_updated,
                                     bool motor_feedback_fault,
                                     bool hip_driver_fault,
                                     const Motor_ControlSnapshot_t &motor_snapshot);

/* VMC 正运动学输出的车身系虚拟腿角（后摆正）与 IMU pitch（低头正）
 * 合成为 LQR 使用的地面系腿状态（前摆正、后摆负）。 */
static void VMC_ConvertLegFeedbackToWorldState(Classic_Data_t *data,
                                                float cal_dt);

static float Chassis_RateLimitRemoteXVelocity(float requested_velocity_m_s,
                                               float previous_velocity_m_s,
                                               float dt_s) {
    if (!isfinite(requested_velocity_m_s) ||
        !isfinite(previous_velocity_m_s) || !isfinite(dt_s) || dt_s <= 0.0f) {
        return 0.0f;
    }

    const bool same_direction =
            (requested_velocity_m_s * previous_velocity_m_s) >= 0.0f;
    const bool increasing_speed = same_direction &&
            (fabsf(requested_velocity_m_s) > fabsf(previous_velocity_m_s));
    const float limit_mps2 = increasing_speed
            ? kRemoteXVelocityAccelLimitMps2
            : kRemoteXVelocityDecelLimitMps2;
    const float max_step_m_s = limit_mps2 * dt_s;
    const float delta_m_s = requested_velocity_m_s - previous_velocity_m_s;

    if (delta_m_s > max_step_m_s) {
        return previous_velocity_m_s + max_step_m_s;
    }
    if (delta_m_s < -max_step_m_s) {
        return previous_velocity_m_s - max_step_m_s;
    }
    return requested_velocity_m_s;
}

/* FireWater CSV：目标L0、左L0、右L0、左误差、右误差、左F0、右F0。 */
void LegLengthPID_Data_Print(Classic_Data_t *pClassicData);

/* FireWater CSV：LQR 十维误差 + 双轮转速原始反馈 + 双 Fw 输出，共 14 列：
 * X, dX, yaw, dyaw, thetaL, dthetaL, thetaR, dthetaR, -pitch, -dpitch,
 * wheelVEL_L/R(轮轴rad/s原始), Fw_L/R(N)。
 * 后四列用于判断尖峰时刻是"反馈先动"还是"输出先动"、左右是否对称。 */
void LQR_State_Data_Print(Classic_Data_t *pClassicData);

/* FireWater CSV，专门检查 KF 输入、输出与进入 K 的纵向两列，顺序见函数实现。 */
void Kalman_Data_Print(Classic_Data_t *pClassicData, float cal_dt);

typedef struct {
    bool motor_feedback_fault;
    bool hip_driver_fault;
} MotorFeedbackHealth_t;

/* 初始化六台底盘电机，并按原有启动顺序发送一次零扭矩。 */
static void ChassisMotors_InitAndSendZero(void) {
    DM_Motor_Init(&DM_8009P1, DM_MOTOR_MODEL_8009, 0x051, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_8009P2, DM_MOTOR_MODEL_8009, 0x052, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_8009P3, DM_MOTOR_MODEL_8009, 0x053, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_8009P4, DM_MOTOR_MODEL_8009, 0x054, DM_MOTOR_MODE_MIT, &hfdcan1);
    DM_Motor_Init(&DM_3519L, DM_MOTOR_MODEL_3519, 0x055, DM_MOTOR_MODE_MIT, &hfdcan2);
    DM_Motor_Init(&DM_3519R, DM_MOTOR_MODEL_3519, 0x056, DM_MOTOR_MODE_MIT, &hfdcan2);

    DM_Motor_MIT_Control(&DM_8009P1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    DM_Motor_MIT_Control(&DM_8009P2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    DM_Motor_MIT_Control(&DM_8009P4, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    DM_Motor_MIT_Control(&DM_8009P3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    DM_Motor_MIT_Control(&DM_3519L, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    DM_Motor_MIT_Control(&DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/* 仅供电机反馈超时/驱动故障恢复使用。
 * 先向当前受反馈状态机监管、且已初始化的电机下发零命令，再重发使能帧。
 * 正常配置覆盖四髋和两轮毂；四髋接线调试仅覆盖已接的 P1~P4。 */
static void ChassisMotors_RecoverFromFeedbackFault(void) {
    static DM_Motor_t *const motors[] = {
        &DM_8009P1, &DM_8009P2, &DM_8009P3, &DM_8009P4,
        &DM_3519L, &DM_3519R,
    };
    /* 正常模式只恢复四关节和两轮毂；3507/4310属于上装。 */
    uint32_t monitored_count = 6U;
#if DM_FOUR_HIP_COMMISSIONING_BYPASS
    /* 四髋接线调试只触及已接的 P1~P4，绝不向未接电机反复广播使能。 */
    monitored_count = 4U;
#endif
    for (uint32_t i = 0; i < monitored_count; ++i) {
        if (motors[i]->ID != 0U) {
            if (motors[i]->MODE == DM_MOTOR_MODE_MIT) {
                DM_Motor_MIT_Control(motors[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            } else if (motors[i]->MODE == DM_MOTOR_MODE_SPEED) {
                DM_Motor_Speed_Control(motors[i], 0.0f);
            }
        }
    }
    for (uint32_t i = 0; i < monitored_count; ++i) {
        if (motors[i]->ID != 0U) {
            DM_Motor_Enable(motors[i]);
        }
    }
}

static MotorFeedbackHealth_t MotorFeedback_EvaluateHealth(
        const Motor_ControlSnapshot_t &snapshot) {
    MotorFeedbackHealth_t health = {};
    for (uint32_t i = 0; i < 4U; ++i) {
        if (snapshot.driver_fault_state[i] != 0U) {
            health.hip_driver_fault = true;
            break;
        }
    }

    bool chassis_driver_fault = health.hip_driver_fault;
    for (uint32_t i = 4U; i < 6U && !chassis_driver_fault; ++i) {
        if (snapshot.driver_fault_state[i] != 0U) {
            chassis_driver_fault = true;
        }
    }
#if DM_FOUR_HIP_COMMISSIONING_BYPASS
    health.motor_feedback_fault =
            (dm_motor_feedback_timeout_flag != 0U) || health.hip_driver_fault;
#else
    health.motor_feedback_fault =
            (dm_motor_feedback_timeout_flag != 0U) || chassis_driver_fault;
#endif
    return health;
}

static bool ControlInputs_HaveTimeout(void) {
    return (imu_data_timeout_flag != 0U) || (remote_data_timeout_flag != 0U);
}

static void ChassisControl_ClearOutputs(Classic_Data_t *data) {
    data->Contronller_Data.Fw_L = 0.0f;
    data->Contronller_Data.Fw_R = 0.0f;
    data->Contronller_Data.Tp_L = 0.0f;
    data->Contronller_Data.Tp_R = 0.0f;
    data->Contronller_Data.T1_L = 0.0f;
    data->Contronller_Data.T2_L = 0.0f;
    data->Contronller_Data.T1_R = 0.0f;
    data->Contronller_Data.T2_R = 0.0f;
}

static void VMC_ConvertLegFeedbackToWorldState(Classic_Data_t *data,
                                                float cal_dt) {
    /* phi0 是车身系“后摆正”。合成为地面系后 theta_L/R 为“前摆正、
     * 后摆负”，进入 LQR 时再由调用处按 current - reference 构造误差。 */
    data->States_Data.theta_L =
            +1.5708f - data->Leg_Data.phi_0_L - data->IMU_Data.pitch;
    data->States_Data.d_theta_L = -data->Leg_Data.delta_phi_0_L -
                                  data->IMU_Data.delta_pitch;
    data->States_Data.theta_R =
            +1.5708f - data->Leg_Data.phi_0_R - data->IMU_Data.pitch;
    data->States_Data.d_theta_R = -data->Leg_Data.delta_phi_0_R -
                                  data->IMU_Data.delta_pitch;
    data->States_Data.theta = data->IMU_Data.pitch;
    data->States_Data.d_theta = data->IMU_Data.delta_pitch;

    /* theta_L / theta_R 二阶导：后向差分 + 一阶低通，使用定时器 dt。 */
    const float inv_dt = 1.0f / cal_dt;
    const float raw_dd_theta_L =
            (data->States_Data.d_theta_L - data->States_Data.d_last_d_theta_L) * inv_dt;
    data->States_Data.dd_theta_L =
            kStateDerivativeFilterAlpha * raw_dd_theta_L +
            (1.0f - kStateDerivativeFilterAlpha) * data->States_Data.dd_theta_L;
    data->States_Data.d_last_d_theta_L = data->States_Data.d_theta_L;

    const float raw_dd_theta_R =
            (data->States_Data.d_theta_R - data->States_Data.d_last_d_theta_R) * inv_dt;
    data->States_Data.dd_theta_R =
            kStateDerivativeFilterAlpha * raw_dd_theta_R +
            (1.0f - kStateDerivativeFilterAlpha) * data->States_Data.dd_theta_R;
    data->States_Data.d_last_d_theta_R = data->States_Data.d_theta_R;
}

void CaculateTask(void *pvParameters) {
    VMC_Kinematics_t vmc_kinematics;
    Classic_Data_t *pClassicData = (Classic_Data_t *) pvParameters;
    LegControl_Init();
    StandState_Init();
    StepClimb_Init();
    ChassisMotors_InitAndSendZero();
    
    WheelKalman_Init(&Wheel_Kalman_L, 0.001f, 0.0f, 0.0f);
    WheelKalman_Init(&Wheel_Kalman_R, 0.001f, 0.0f, 0.0f);
    ChassisPositionHold_Init(0.0f);

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
    float shaped_target_x_vel = 0.0f;
    while (1) {
        IMU_ControlSnapshot_t received_imu;
        Remote_ControlSnapshot_t received_remote;
        Motor_ControlSnapshot_t received_motor;
        bool imu_snapshot_updated = false;
        bool remote_snapshot_updated = false;
        bool motor_snapshot_updated = false;

        if ((IMU_ControlQueueHandle != NULL) &&
            (xQueueReceive(IMU_ControlQueueHandle, &received_imu, 0) == pdPASS)) {
            imu_snapshot = received_imu;
            imu_snapshot_updated = true;
        }
        if ((Remote_ControlQueueHandle != NULL) &&
            (xQueueReceive(Remote_ControlQueueHandle, &received_remote, 0) == pdPASS)) {
            remote_snapshot = received_remote;
            remote_snapshot_updated = true;
        }
        if ((Motor_ControlQueueHandle != NULL) &&
            (xQueueReceive(Motor_ControlQueueHandle, &received_motor, 0) == pdPASS)) {
            motor_snapshot = received_motor;
            motor_snapshot_updated = true;
        }

        /* Classic_Data 从这里起由 Calculate 持有并使用本周期输入快照。 */
        pClassicData->IMU_Data = imu_snapshot.data;
        pClassicData->Target_Data.Target_X_vel = remote_snapshot.target_x_vel;
        pClassicData->Target_Data.Target_Y_vel = remote_snapshot.target_y_vel;
        pClassicData->Target_Data.Target_yaw_vel = remote_snapshot.target_yaw_vel;
        pClassicData->Target_Data.Target_Gimbal_Pitch_vel =
                remote_snapshot.target_gimbal_pitch_vel;
        pClassicData->Target_Data.Target_Gyro_Speed = remote_snapshot.target_gyro_speed;
        pClassicData->Target_Data.gyro_mode_enable = remote_snapshot.gyro_mode_enable;
        pClassicData->Contronller_Data.total_enable = remote_snapshot.total_enable;
        /*
         * 对前向命令先做统一死区，再供速度环与位置保持共同使用。只在
         * PositionHold 内判断会造成 X_pos 已锁定、d_x_err 却继续追逐
         * 遥控器/转向耦合残余速度，从而松杆后缓慢爬行。
         */
        float effective_target_x_vel = pClassicData->Target_Data.Target_X_vel;
        if (fabsf(effective_target_x_vel) <
            CHASSIS_POSITION_HOLD_VELOCITY_EPS_M_S) {
            effective_target_x_vel = 0.0f;
        }
        pClassicData->Target_Data.Target_X_vel = effective_target_x_vel;

        /* 总使能关闭时，无论本轮随后等到完整反馈或超时，
         * F0 状态都不能保留上一拍的值。 */
        if (pClassicData->Contronller_Data.total_enable == 0U) {
            pClassicData->Contronller_Data.F0_L = 0.0f;
            pClassicData->Contronller_Data.F0_R = 0.0f;
        }
        /* 可选任务心跳调试串口输出，默认关闭。 */
        // static TickType_t xLastAlivePrint = 0;
        // TickType_t xAliveNow = xTaskGetTickCount();
        // if ((xAliveNow - xLastAlivePrint) > pdMS_TO_TICKS(1000)) {
        //     uart_print_blocking("CAL_ALIVE\r\n");
        //     xLastAlivePrint = xAliveNow;
        // }
        /* 固定 1 ms 调度：不等待反馈事件。正常版本中，八电机反馈超时、
         * 任一驱动状态码故障、IMU 超时或遥控超时都会令底盘输出清零。
         * 四髋接线调试配置只将电机健康范围收窄为 P1~P4；这四台的超时/故障
         * 仍会清零，IMU、遥控以及遥控总使能的保护不变。 */
        const MotorFeedbackHealth_t motor_health =
                MotorFeedback_EvaluateHealth(motor_snapshot);
        ControlHealth_Data_Print(imu_snapshot_updated, remote_snapshot_updated,
                                 motor_snapshot_updated, motor_health.motor_feedback_fault,
                                 motor_health.hip_driver_fault, motor_snapshot);
        if (motor_health.motor_feedback_fault || ControlInputs_HaveTimeout()) {
            /* 同一段电机故障只尝试恢复一次，不能在 1 ms 控制周期反复刷使能帧。 */
            if (motor_health.motor_feedback_fault && !motor_fault_reenable_sent) {
                ChassisMotors_RecoverFromFeedbackFault();
                motor_fault_reenable_sent = true;
            }
            shaped_target_x_vel = 0.0f;
            ChassisControl_ClearOutputs(pClassicData);
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

        /* 速度参考在 Calculate 内整形，RemoteTask 仍只负责遥控原始数据
         * 的死区和比例换算。总使能关闭时清零内部状态，避免再次使能时
         * 沿用关机前的速度目标。 */
        if (pClassicData->Contronller_Data.total_enable == 0U) {
            shaped_target_x_vel = 0.0f;
        } else {
            shaped_target_x_vel = Chassis_RateLimitRemoteXVelocity(
                    effective_target_x_vel, shaped_target_x_vel, cal_dt);
        }
        pClassicData->Target_Data.Target_X_vel = shaped_target_x_vel;
        
        /* CAN 电机反馈解算已完成安装坐标、零位和轮毂减速比换算；本周期
         * 只取同一份队列快照，不重复整理，后续 VMC/KF 使用的值不变。 */
        pClassicData->Motor_Data = motor_snapshot.chassis_motor_data;

        /* 本轮四个髋电机反馈已经就绪。先完成正运动学并写入 Leg_Data，
         * 后面的 Kalman、起身判断、LQR、PID 与补偿都只使用本轮同一套腿状态。 */
        VMC_UpdateLegFeedback(pClassicData, &vmc_kinematics, cal_dt);

        VMC_ConvertLegFeedbackToWorldState(pClassicData, cal_dt);
        //二阶导腿角加速度使用于离地检测
        
        /* Kalman 的控制输入必须是带方向的前向加速度；accel_h 是幅值，
         * 会在静止/转向时持续向位置状态注入正向偏置。 */
        /* KF 只使用 VMC 车身系腿摆角（后摆正）、腿长、轮端速度和 IMU
         * 前向加速度；不使用车身 pitch 或 pitch 角速度。 */
        WheelKalman_Update(&Wheel_Kalman_L, pClassicData->IMU_Data.accel_x, cal_dt,
                           pClassicData->Leg_Data.phi_0_L,
                           pClassicData->Leg_Data.delta_phi_0_L,
                           pClassicData->Leg_Data.L0_L, pClassicData->Leg_Data.d_L0_L,
                           pClassicData->Motor_Data.Left_Wheel_Motor_VEL);
        WheelKalman_Update(&Wheel_Kalman_R, pClassicData->IMU_Data.accel_x, cal_dt,
                           pClassicData->Leg_Data.phi_0_R,
                           pClassicData->Leg_Data.delta_phi_0_R,
                           pClassicData->Leg_Data.L0_R, pClassicData->Leg_Data.d_L0_R,
                           pClassicData->Motor_Data.Right_Wheel_Motor_VEL);
        // ====== 起身状态机（左右腿共用一个状态）======
        StandState_Update(cal_dt,
                          pClassicData->States_Data.theta_L,
                          pClassicData->States_Data.theta_R);
        const bool total_enable = (remote_snapshot.total_enable != 0U);
        const bool lqr_switch_enabled = (remote_snapshot.lqr_enable != 0U);
#if STEP_CLIMB_CONTROL_ENABLE
        const bool step_detection_enabled = (remote_snapshot.step_climb_enable != 0U);
        const StepClimbCommand_t step_cmd = StepClimb_Update(
                cal_dt, step_detection_enabled, total_enable, lqr_switch_enabled,
                StandState_IsUp(),
                pClassicData->Leg_Data.L0_L, pClassicData->Leg_Data.L0_R,
                pClassicData->States_Data.theta_L, pClassicData->States_Data.theta_R);
#else
        StepClimbCommand_t step_cmd = {};
        step_cmd.allow_lqr = true;
#endif
        if (step_cmd.force_not_up) {
            StandState_ForceNotUp();
        }
#if LEG_CONTROL_TEST_MODE
        const bool stand_up = true;
#else
        const bool stand_up = StandState_IsUp();
#endif

        // ====== LQR + 腿长 PID + roll 差分补偿（phi PID 当前关闭） ======
        /* LQR 仅在遥控器开关打开时运行；关闭时清零，避免沿用旧输出。 */
        pClassicData->Contronller_Data.Fw_L = 0.0f;
        pClassicData->Contronller_Data.Fw_R = 0.0f;
        pClassicData->Contronller_Data.Tp_L = 0.0f;
        pClassicData->Contronller_Data.Tp_R = 0.0f;

        /* 左右腿按各自实际 L0 查询实车定义下的角度目标（rad）。 */
        theta_L_ref = LegAngleTarget_GetTheta(pClassicData->Leg_Data.L0_L);
        theta_R_ref = LegAngleTarget_GetTheta(pClassicData->Leg_Data.L0_R);

        const bool lqr_enabled = !LEG_CONTROL_TEST_MODE && lqr_switch_enabled &&
                                 step_cmd.allow_lqr;
        const bool leg_pid_enabled = total_enable;
#if GROUND_DETECTION_ENABLE
        /* 离地检测只在正常起身工作角内运行。角度仅作为运行门限；真正的
         * 离地判据仍是左右腿各自的法向力 Fn 及原有连续确认时间。 */
        const bool ground_detection_active =
                total_enable && lqr_enabled && stand_up &&
                (fabsf(pClassicData->States_Data.theta_L) <
                 STAND_UP_ENTER_ANGLE_ABS_RAD) &&
                (fabsf(pClassicData->States_Data.theta_R) <
                 STAND_UP_ENTER_ANGLE_ABS_RAD);
        const bool airborne_control_active =
                ground_detection_active &&
                (GroundDetection_IsAirborne() != 0U);
#else
        const bool airborne_control_active = false;
#endif
        const bool position_hold_enable =
                leg_pid_enabled && lqr_enabled && stand_up &&
                !airborne_control_active;
        ChassisLongitudinalState_Update(
                pClassicData, Wheel_Kalman_L.x[0], Wheel_Kalman_R.x[0],
                Wheel_Kalman_L.x[1], Wheel_Kalman_R.x[1],
                position_hold_enable);
        ChassisYawState_Update(pClassicData, position_hold_enable);

        /* LQR 仅由遥控成品开关 lqr_enable 控制，并用左右实际腿长均值查表。 */
        if (lqr_enabled) {
            const float average_leg_length =
                    (pClassicData->Leg_Data.L0_L + pClassicData->Leg_Data.L0_R) * 0.5f;
            const float scheduled_leg_length = average_leg_length;
            K_Fixed_Leg_t scheduled_gain;
            LQR_K_GetInterpolated(scheduled_leg_length, scheduled_gain);
            LQR_K_CalculateWithGain(pClassicData, scheduled_gain,
                                    theta_ref, d_theta_ref,
                                    theta_L_ref, d_theta_L_ref,
                                    theta_R_ref, d_theta_R_ref);
        }
        /* LQR 自己负责不经 VMC 的轮端虚拟力 Fw 限幅。 */
        LQR_LimitWheelForces(pClassicData);
        /* 总使能且已起身后叠加双腿重力补偿；起身前不叠加。
         * 总使能关闭时 ControlTask 向全部电机发送零力矩。 */
        float gravity_tp_l = 0.0f;
        float gravity_tp_r = 0.0f;
#if LEG_GRAVITY_COMPENSATION_ENABLE
#if LEG_CONTROL_TEST_MODE
        if (leg_pid_enabled) {
            gravity_tp_r = LegGravity_GetTp(pClassicData->Leg_Data.L0_R,
                                            pClassicData->States_Data.theta_R);
        }
#else
        if (leg_pid_enabled && stand_up) {
            gravity_tp_l = LegGravity_GetTp(pClassicData->Leg_Data.L0_L,
                                            pClassicData->States_Data.theta_L);
            gravity_tp_r = LegGravity_GetTp(pClassicData->Leg_Data.L0_R,
                                            pClassicData->States_Data.theta_R);
        }
#endif
#endif

#if LEG_CONTROL_TEST_MODE
        const bool override_leg_target = false;
        const float override_leg_target_m = 0.0f;
        const bool swing_pid_enable = false;
        const float swing_target_rad = 0.0f;
#else
        /* 离地是并行控制覆盖：不关闭 LQR 或任何补偿，只将腿长目标设为
         * 0.350 m。该状态来自上一拍在起身工作角内完成确认的 Fn 结果。 */
        const bool override_leg_target =
                airborne_control_active || step_cmd.override_leg_length;
        const float override_leg_target_m = airborne_control_active
                ? GROUND_DETECTION_LEG_TARGET_M : step_cmd.leg_length_target_m;
        const bool swing_pid_enable = step_cmd.swing_pid_enable;
        const float swing_target_rad = step_cmd.swing_target_rad;
#endif
        LegControl_Update(pClassicData, cal_dt, stand_up, leg_pid_enabled,
                          remote_snapshot.leg_length_channel,
                          override_leg_target, override_leg_target_m,
                          swing_pid_enable, swing_target_rad);

        /* F0 已在 LegControl 内限幅；Tp 是 VMC 的虚拟输入，在 VMC 模块限幅。 */
        VMC_LimitVirtualPitchTorque(pClassicData, 4.0f);
        // ====== VMC 力矩分配 ======
        VMC_MapForceToJointTorques(
                pClassicData, &vmc_kinematics,
                pClassicData->Contronller_Data.F0_L,
                pClassicData->Contronller_Data.F0_R,
                pClassicData->Contronller_Data.Tp_L + gravity_tp_l,
                pClassicData->Contronller_Data.Tp_R + gravity_tp_r);

        /* VMC 后只限髋关节实际扭矩；轮力 Fw 不属于 VMC。 */
        VMC_LimitJointTorques(pClassicData);
#if LEG_CONTROL_TEST_MODE
        pClassicData->Contronller_Data.T1_L = 0.0f;
        pClassicData->Contronller_Data.T2_L = 0.0f;
#endif

        /* 离地检测使用本轮最终 F0 与真正送入 VMC 的 Tp。
         * GroundDetection_Update 内部只剔除弹簧内部抵消项，并完成滤波、
         * 迟滞及连续拍确认；不再用上一轮命令或重复叠加重力补偿。 */
#if GROUND_DETECTION_ENABLE
        if (ground_detection_active) {
            const float fn_left = GroundDetection_Update(
                    GD_SIDE_LEFT,
                    pClassicData->Contronller_Data.F0_L,
                    pClassicData->Contronller_Data.Tp_L + gravity_tp_l,
                    pClassicData->Leg_Data.L0_L,
                    pClassicData->Leg_Data.d_L0_L,
                    pClassicData->Leg_Data.dd_L0_L,
                    pClassicData->IMU_Data.accel_v,
                    pClassicData->States_Data.theta_L,
                    pClassicData->States_Data.d_theta_L,
                    pClassicData->States_Data.dd_theta_L);
            const float fn_right = GroundDetection_Update(
                    GD_SIDE_RIGHT,
                    pClassicData->Contronller_Data.F0_R,
                    pClassicData->Contronller_Data.Tp_R + gravity_tp_r,
                    pClassicData->Leg_Data.L0_R,
                    pClassicData->Leg_Data.d_L0_R,
                    pClassicData->Leg_Data.dd_L0_R,
                    pClassicData->IMU_Data.accel_v,
                    pClassicData->States_Data.theta_R,
                    pClassicData->States_Data.d_theta_R,
                    pClassicData->States_Data.dd_theta_R);
            (void)fn_left;
            (void)fn_right;
        } else {
            /* 工作角外不运行、不累计确认时间，也不保留旧离地状态。 */
            GroundDetection_Reset();
        }

        /* 双腿均确认离地时只禁止轮毂扭矩输出。LQR 本身继续计算，腿长 PID、
         * 气弹簧补偿、腿重力补偿及两套 roll 补偿均保持原运行条件。 */
        if (ground_detection_active && GroundDetection_IsAirborne()) {
            pClassicData->Contronller_Data.Fw_L = 0.0f;
            pClassicData->Contronller_Data.Fw_R = 0.0f;
        }
#endif

        {
            /* 原控制周期末尾调试选择：一次只启用一种，避免不同字段混在同一 CSV 流。 */
            static TickType_t xLastPrintTick = 0;
            TickType_t xNow = xTaskGetTickCount();
            if ((xNow - xLastPrintTick) > pdMS_TO_TICKS(100)) {
                /* 调试打印选择（100ms 一帧，FireWater 纯 CSV，一次只开一个）：
                 * Motor_Data_Print       - 8 列：四髋电机角度(rad)×4 + 速度(rad/s)×2 + 左右轮速
                 * LEG_Data_Print         - 8 列：L0_L,L0_R,dL0_L,dL0_R,phi0_L,phi0_R,dphi0_L,dphi0_R
                 *                         （VMC车身系腿角：后摆正，不是地面系 theta）
                 * LegLengthPID_Data_Print- 5 列：目标腿长,左L0,右L0,左F0,右F0（m,m,m,N,N）
                 *                         （腿长环跟踪与输出力）
                 * IMU_Data_Print         - 6 列：roll,pitch,yaw,droll,dpitch,dyaw
                 *                         （已完成原始HI14轴映射：pitch低头正，yaw左转正）
                 * State_Data_Print       - 14 列：10 个 LQR 状态 + P1/P2raw/phi1/phi4 关节角(度)
                 *                         （全状态总览，角度类为度）
                 * LQR_State_Data_Print   - 14 列：X,dX,yaw,dyaw,thetaL,dthetaL,thetaR,dthetaR,
                 *                         -pitch,-dpitch,轮速L,轮速R,Fw_L,Fw_R
                 *                         （LQR 误差 + 双轮原始反馈 + 双轮输出力；诊断速度环用）
                 * Kalman_Data_Print    - 21 列：KF 实际输入、左右最终 p/v、
                 *                         平均 p/v、K 实际使用的位置/速度两列；
                 *                         具体列顺序见 Kalman_Data_Print 注释。
                 * */
                // Motor_Data_Print(pClassicData);          // 四髋电机角度/速度 + 左右轮速
                // LEG_Data_Print(pClassicData);            // 腿长/腿角/腿速
                // LegLengthPID_Data_Print(pClassicData);   // 腿长PID跟踪
                // IMU_Data_Print(pClassicData);            // IMU 姿态与加速度
                // State_Data_Print(pClassicData);          // LQR 全状态 + 关节角
                // LQR_State_Data_Print(pClassicData);       // LQR 误差 + 轮速 + Fw（速度环诊断）
                 Kalman_Data_Print(pClassicData, cal_dt);  // KF 输入/输出 + K 实际纵向两列
                xLastPrintTick = xNow;
            }

        }
        xEventGroupSetBits(ControlEventGroup, Calculate_OK_BIT);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}

/**
 * @brief 将角度（度）转换为弧度
 * @param degrees 输入角度值（度）
 * @return float 转换后的弧度值
 */
float angle_to_rad(float degrees) { return degrees * (PI / 180.0f); }

static void ControlHealth_Data_Print(bool imu_snapshot_updated,
                                     bool remote_snapshot_updated,
                                     bool motor_snapshot_updated,
                                     bool motor_feedback_fault,
                                     bool hip_driver_fault,
                                     const Motor_ControlSnapshot_t &motor_snapshot) {
    static TickType_t last_print_tick = 0;
    const TickType_t now = xTaskGetTickCount();
    if ((now - last_print_tick) < pdMS_TO_TICKS(200)) return;
    last_print_tick = now;
    /* 电机顺序：P1,P2,P3,P4,3519L,3519R,3507,4310。
     * st=驱动器反馈高四位原始状态码；sw=软件故障位。
     * last_* 是最近一条非法帧/驱动状态故障记录。 */
    uart_print("FAULT,st=%X/%X/%X/%X/%X/%X/%X/%X,"
               "sw=%u/%u/%u/%u/%u/%u/%u/%u,"
               "to=%u/%u/%u,mf=%u,hf=%u,q=%u/%u/%u,"
               "last=%lu/%03X/%u/%X/%u,"
               "txf=%lu/%lu/%lu,bo=%lu/%lu/%lu,"
               "ep=%lu/%lu/%lu,ew=%lu/%lu/%lu,rst=%lu/%lu/%lu\r\n",
               (unsigned)motor_snapshot.data[0].err,
               (unsigned)motor_snapshot.data[1].err,
               (unsigned)motor_snapshot.data[2].err,
               (unsigned)motor_snapshot.data[3].err,
               (unsigned)motor_snapshot.data[4].err,
               (unsigned)motor_snapshot.data[5].err,
               (unsigned)motor_snapshot.data[6].err,
               (unsigned)motor_snapshot.data[7].err,
               (unsigned)motor_snapshot.driver_fault_state[0],
               (unsigned)motor_snapshot.driver_fault_state[1],
               (unsigned)motor_snapshot.driver_fault_state[2],
               (unsigned)motor_snapshot.driver_fault_state[3],
               (unsigned)motor_snapshot.driver_fault_state[4],
               (unsigned)motor_snapshot.driver_fault_state[5],
               (unsigned)motor_snapshot.driver_fault_state[6],
               (unsigned)motor_snapshot.driver_fault_state[7],
               (unsigned)imu_data_timeout_flag,
               (unsigned)remote_data_timeout_flag,
               (unsigned)dm_motor_feedback_timeout_flag,
               (unsigned)motor_feedback_fault,
               (unsigned)hip_driver_fault,
               (unsigned)imu_snapshot_updated,
               (unsigned)remote_snapshot_updated,
               (unsigned)motor_snapshot_updated,
               (unsigned long)dm_motor_fault_record.count,
               (unsigned)dm_motor_fault_record.feedback_id,
               (unsigned)dm_motor_fault_record.motor_id,
               (unsigned)dm_motor_fault_record.status,
               (unsigned)dm_motor_fault_record.reason,
               (unsigned long)fdcan1_tx_fifo_full_count,
               (unsigned long)fdcan2_tx_fifo_full_count,
               (unsigned long)fdcan3_tx_fifo_full_count,
               (unsigned long)fdcan1_bus_off_count,
               (unsigned long)fdcan2_bus_off_count,
               (unsigned long)fdcan3_bus_off_count,
               (unsigned long)fdcan1_error_passive_count,
               (unsigned long)fdcan2_error_passive_count,
               (unsigned long)fdcan3_error_passive_count,
               (unsigned long)fdcan1_error_warning_count,
               (unsigned long)fdcan2_error_warning_count,
               (unsigned long)fdcan3_error_warning_count,
               (unsigned long)fdcan1_restart_count,
               (unsigned long)fdcan2_restart_count,
               (unsigned long)fdcan3_restart_count);
}

void LEG_Data_Print(Classic_Data_t *pClassicData) {
    /* FireWater 纯 CSV，依次为：
     * L0_L,L0_R,dL0_L,dL0_R,phi0_L,phi0_R,dphi0_L,dphi0_R。
     * phi0/dphi0 是 VMC 车身系量，后摆为正，尚未合成地面系 theta。
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
    /* FireWater: 纯 CSV，依次为 roll,pitch,yaw,droll,dpitch,dyaw。
     * 这是安装轴映射后的控制数据，不是 HI14 原始字段：pitch/ dpitch
     * 为低头正；yaw/dyaw 不取反，实车左转正。 */
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
    /* FireWater 纯 CSV：目标L0、左L0、右L0、左F0、右F0；单位 m,m,m,N,N。 */
    const float target = pClassicData->Target_Data.Target_LegLong;
    const float values[5] = {
        target,
        pClassicData->Leg_Data.L0_L,
        pClassicData->Leg_Data.L0_R,
        pClassicData->Contronller_Data.F0_L,
        pClassicData->Contronller_Data.F0_R,
    };
    int milli[5];
    for (uint32_t index = 0; index < 5U; ++index)
        milli[index] = (int)lroundf(values[index] * 1000.0f);

    uart_print("%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d\r\n",
               milli[0] < 0 ? '-' : ' ', abs(milli[0]) / 1000, abs(milli[0]) % 1000,
               milli[1] < 0 ? '-' : ' ', abs(milli[1]) / 1000, abs(milli[1]) % 1000,
               milli[2] < 0 ? '-' : ' ', abs(milli[2]) / 1000, abs(milli[2]) % 1000,
               milli[3] < 0 ? '-' : ' ', abs(milli[3]) / 1000, abs(milli[3]) % 1000,
               milli[4] < 0 ? '-' : ' ', abs(milli[4]) / 1000, abs(milli[4]) % 1000);
}

void LQR_State_Data_Print(Classic_Data_t *pClassicData) {
    /* 与 LQR_K_CalculateWithGain 内的 state[10] 完全一致的构造：
     * 只读状态、不改变任何控制量。 */
    const float x_pos_lqr = fminf(fmaxf(pClassicData->States_Data.X_pos, -1.00f), 1.00f);
    const float values[14] = {
        x_pos_lqr,
        pClassicData->States_Data.d_x_err,
        pClassicData->States_Data.yaw_err,
        pClassicData->States_Data.d_yaw_err,
        pClassicData->States_Data.theta_L - theta_L_ref,
        pClassicData->States_Data.d_theta_L - d_theta_L_ref,
        pClassicData->States_Data.theta_R - theta_R_ref,
        pClassicData->States_Data.d_theta_R - d_theta_R_ref,
        -(pClassicData->States_Data.theta - theta_ref),
        -(pClassicData->States_Data.d_theta - d_theta_ref),
        pClassicData->Motor_Data.Left_Wheel_Motor_VEL,
        pClassicData->Motor_Data.Right_Wheel_Motor_VEL,
        pClassicData->Contronller_Data.Fw_L,
        pClassicData->Contronller_Data.Fw_R,
    };
    int milli[14];
    for (uint32_t index = 0; index < 14U; ++index)
        milli[index] = (int)lroundf(values[index] * 1000.0f);

    uart_print("%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,"
               "%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d,"
               "%c%d.%03d,%c%d.%03d,%c%d.%03d,%c%d.%03d\r\n",
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

void Kalman_Data_Print(Classic_Data_t *pClassicData, float cal_dt) {
    /* FireWater 纯 CSV，所有值均为“已经实际送入/取自 KF”的数值，按列为：
     *  0 phiKF_L, 1 dphiKF_L, 2 L_L, 3 dL_L, 4 wheelVelKF_L,
     *  5 phiKF_R, 6 dphiKF_R, 7 L_R, 8 dL_R, 9 wheelVelKF_R,
     * 10 pKF_L,11 vKF_L,12 pKF_R,13 vKF_R,
     * 14 pKF_avg,15 vKF_avg,
     * 16 wheelLinear_L,17 wheelLinear_R,18 wheelLinear_avg,
     * 19 X_pos(K 实际使用的位移误差),20 d_x_err(K 实际使用的速度误差)。
     * phiKF/dphiKF 是 VMC 车身系后摆正，KF 不使用 pitch 数据。
     * wheelLinear 是仅用轮端角速度乘轮半径得到的线速度，不含腿项或 KF。 */
    const float p_kf_avg = (Wheel_Kalman_L.x[0] + Wheel_Kalman_R.x[0]) * 0.5f;
    const float v_kf_avg = (Wheel_Kalman_L.x[1] + Wheel_Kalman_R.x[1]) * 0.5f;
    const float wheel_linear_l =
            pClassicData->Motor_Data.Left_Wheel_Motor_VEL * WHEEL_RADIUS;
    const float wheel_linear_r =
            pClassicData->Motor_Data.Right_Wheel_Motor_VEL * WHEEL_RADIUS;
    const float values[21] = {
        pClassicData->Leg_Data.phi_0_L,
        pClassicData->Leg_Data.delta_phi_0_L,
        pClassicData->Leg_Data.L0_L,
        pClassicData->Leg_Data.d_L0_L,
        pClassicData->Motor_Data.Left_Wheel_Motor_VEL,
        pClassicData->Leg_Data.phi_0_R,
        pClassicData->Leg_Data.delta_phi_0_R,
        pClassicData->Leg_Data.L0_R,
        pClassicData->Leg_Data.d_L0_R,
        pClassicData->Motor_Data.Right_Wheel_Motor_VEL,
        Wheel_Kalman_L.x[0],
        Wheel_Kalman_L.x[1],
        Wheel_Kalman_R.x[0],
        Wheel_Kalman_R.x[1],
        p_kf_avg,
        v_kf_avg,
        wheel_linear_l,
        wheel_linear_r,
        (wheel_linear_l + wheel_linear_r) * 0.5f,
        pClassicData->States_Data.X_pos,
        pClassicData->States_Data.d_x_err,
    };

    char line[512];
    size_t used = 0U;
    for (uint32_t index = 0; index < 21U; ++index) {
        const int milli = (int)lroundf(values[index] * 1000.0f);
        const int written = snprintf(line + used, sizeof(line) - used,
                                     "%c%d.%03d%s",
                                     milli < 0 ? '-' : ' ', abs(milli) / 1000,
                                     abs(milli) % 1000,
                                     (index == 20U) ? "\r\n" : ",");
        if (written < 0 || (size_t)written >= (sizeof(line) - used)) {
            return;  // 行缓冲不足时宁可丢弃本帧，避免输出截断 CSV
        }
        used += (size_t)written;
    }
    uart_print("%s", line);
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
