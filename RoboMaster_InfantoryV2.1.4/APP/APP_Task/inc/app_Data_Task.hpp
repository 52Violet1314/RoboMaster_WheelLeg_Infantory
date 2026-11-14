#ifndef APP_DATA_TASK_HPP
#define APP_DATA_TASK_HPP
#include "FreeRTOS.h"
#include "app_Task.hpp"
#include "SBUS.h"
#include "main.h"
#include "app_motor.h"
#include "bsp_UART.h"

typedef struct {
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
    float Target_yaw_vel;
    float Target_LegLong;
} Target_Data_t;

/* Complete, by-value producer-to-Calculate snapshots. */
typedef struct {
    IMU_Data_t data;
} IMU_ControlSnapshot_t;

typedef struct {
    SBUS_Data_t sbus_data;
    float target_x_vel;
    float target_yaw_vel;
} Remote_ControlSnapshot_t;

#define MOTOR_CONTROL_SNAPSHOT_COUNT 8U
typedef struct {
    DM_Motor_Data_t data[MOTOR_CONTROL_SNAPSHOT_COUNT];
    uint8_t driver_fault_state[MOTOR_CONTROL_SNAPSHOT_COUNT];
    uint8_t driver_fault_flag;
} Motor_ControlSnapshot_t;

typedef struct {
    IMU_Data_t IMU_Data;
    Motor_Data_t Motor_Data;

    struct {
        float phi_0_L;
        float delta_phi_0_L;
        float phi_0_R;
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
        uint8_t total_enable;
    } Contronller_Data;

    struct {
        float X_pos;
        float X_pos_Err;
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
