#ifndef APP_DATA_TASK_HPP
#define APP_DATA_TASK_HPP
#include "FreeRTOS.h"
#include "app_Task.hpp"
#include "SBUS.h"
#include "main.h"
#include "app_motor.h"


typedef struct {
    struct {
        float roll;
        float delta_roll;
        float pitch;
        float delta_pitch;
        float yaw;
        float delta_yaw;
        float last_roll;
        float last_pitch;
        float last_yaw;

        /* 机体角速度 (rad/s)，陀螺仪原始三轴输出 */
        float gyro[3];

        /* 机体加速度 (m/s²)，含重力分量，机体系下未旋转 */
        float accel[3];

        /* 世界系竖直加速度 (m/s²)，向上为正，已去除重力 */
        float accel_v;

        /* 世界系水平加速度幅值 (m/s²) */
        float accel_h;
    } IMU_Data;

    struct {
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
    } Motor_Data;

    struct {
        int imu_time;
        int leg_data_time;
        int last_imu_time;
        int last_leg_data_time;
    } Data_Time;

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
    } Contronller_Data;

    struct {
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

    struct {
        float Target_X_vel;
        float Target_yaw_vel;
        float Target_LegLong;
    } Target_Data;
} Classic_Data_t;

extern Classic_Data_t Classic_Data;


#ifdef __cplusplus
extern "C" {
#endif

void app_Data_Task(void *pvParameters);
#ifdef __cplusplus
}
#endif


#endif