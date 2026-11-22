#ifndef APP_DATA_TASK_HPP
#define APP_DATA_TASK_HPP
#include "FreeRTOS.h"
#include "app_Task.hpp"
#include "SBUS.h"
#include "main.h"
#include "app_motor.h"
#include "bsp_UART.h"

typedef struct {
    /* 控制层 IMU 姿态：pitch/delta_pitch 为低头正；yaw/delta_yaw 保持
     * 原始 HI14 方向（实车左转正）。原始 HI14 pitch 抬头正的翻转在
     * app_INSTask.cpp 已完成，以下字段不再做“误差取反”。 */
    float roll;
    float delta_roll;
    float pitch;
    float delta_pitch;
    float yaw;
    float delta_yaw;
    float last_roll;
    float last_pitch;
    float last_yaw;
    float gyro[3];
    float accel[3];
    float accel_v;
    float accel_x;
    float accel_h;
} IMU_Data_t;

typedef struct {
    float L1_Motor_POS;
    float L1_Motor_VEL;
    float L2_Motor_POS;
    float L2_Motor_VEL;
    float R1_Motor_POS;
    float R1_Motor_VEL;
    float R2_Motor_POS;
    float R2_Motor_VEL;
    float Left_Wheel_Motor_VEL;
    float Right_Wheel_Motor_VEL;
} Motor_Data_t;

typedef struct {
    float Target_X_vel;
    float Target_Y_vel;              // CH3 横移目标，当前底盘无横移执行器，未接入控制
    float Target_yaw_vel;
    float Target_Gimbal_Pitch_vel;   // CH1 云台俯仰速度预留，未接入控制
    float Target_Gyro_Speed;         // CH4 小陀螺转速预留，未接入控制
    float Target_LegLong;
    uint8_t gyro_mode_enable;        // CH8 小陀螺模式预留，未接入控制
} Target_Data_t;

/* Complete, by-value producer-to-Calculate snapshots. */
typedef struct {
    IMU_Data_t data;
} IMU_ControlSnapshot_t;

typedef struct {
    /* RemoteTask 已完成摇杆/旋钮死区和比例换算；Calculate 直接消费。 */
    float target_yaw_vel;             // CH0：右杆左右
    float target_gimbal_pitch_vel;    // CH1：右杆前后，预留
    float target_x_vel;               // CH2：左杆前后
    float target_y_vel;               // CH3：左杆左右，预留
    float target_gyro_speed;          // CH4：小陀螺旋钮，预留
    int16_t leg_length_channel;       // CH5：腿长旋钮，保留给腿控标尺换算
    uint8_t step_climb_enable;        // CH6
    uint8_t lqr_enable;               // CH7
    uint8_t gyro_mode_enable;         // CH8，预留
    uint8_t total_enable;             // CH9
} Remote_ControlSnapshot_t;

#define MOTOR_CONTROL_SNAPSHOT_COUNT 8U
typedef struct {
    DM_Motor_Data_t data[MOTOR_CONTROL_SNAPSHOT_COUNT];
    uint8_t driver_fault_state[MOTOR_CONTROL_SNAPSHOT_COUNT];
    uint8_t driver_fault_flag;
    /* 六台底盘电机已按底盘统一坐标整理的反馈。原始八电机 data 仍保留，
     * 供故障诊断及上装电机使用；Calculate 不再重复做底盘反馈映射。 */
    Motor_Data_t chassis_motor_data;
} Motor_ControlSnapshot_t;

typedef struct {
    IMU_Data_t IMU_Data;
    Motor_Data_t Motor_Data;

    struct {
        /* VMC 正运动学虚拟腿角（车身系）：后摆为正。仅用于 VMC/诊断；
         * 它不是 LQR 的地面系 theta。 */
        float phi_0_L;
        /* 两髋电机 CAN 速度按对称五连杆关系平均后的原始腿摆角速度，
         * 仅用于腿摆观测器输入与诊断，不直接进入控制。 */
        float raw_delta_phi_0_L;
        float delta_phi_0_L;
        float phi_0_R;
        float raw_delta_phi_0_R;
        float delta_phi_0_R;
        float theta_L;
        float delta_theta_L;
        float theta_R;
        float delta_theta_R;
        float d_L0_L;       // L0_L 一阶导 (m/s)
        float d_L0_R;       // L0_R 一阶导 (m/s)
        float d_last_L0_L;  // L0_L 上一周期值（内部状态）
        float d_last_L0_R;  // L0_R 上一周期值（内部状态）
        float dd_L0_L;      // L0_L 二阶导 (m/s²)
        float dd_L0_R;      // L0_R 二阶导 (m/s²)
        float L0_L;
        float L0_R;
    } Leg_Data;

    struct {
        float F0_L;
        float F0_R;
        float T1_L;
        float T2_L;
        float T1_R;
        float T2_R;
        float Fw_L;
        float Fw_R;
        float Tp_L;
        float Tp_R;
        /* 拨弹 3507 的输出轴速度指令，rad/s；由拨弹控制逻辑写入。 */
        float Feeder_Speed_Command_Rad_S;
        uint8_t total_enable;
    } Contronller_Data;

    struct {
        /* 供 LQR/Kalman 的估计状态。theta_L/R 是地面系腿角：前摆为正、
         * 后摆为负；theta/d_theta 是固件 IMU pitch：低头为正。 */
        float X_pos;
        float d_X;
        float theta_L;
        float d_theta_L;
        float theta_R;
        float d_theta_R;
        float dd_theta_L;   // theta_L 二阶导 (rad/s²)
        float dd_theta_R;   // theta_R 二阶导 (rad/s²)
        float d_last_d_theta_L, d_last_d_theta_R;  // 上一周期角速度（内部状态）
        float theta;
        float d_theta;
        float yaw;
        float d_yaw;
        float d_x_err;
        float yaw_err;
        float d_yaw_err;
    } States_Data;

    Target_Data_t Target_Data;
} Classic_Data_t;

extern Classic_Data_t Classic_Data;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif


#endif
