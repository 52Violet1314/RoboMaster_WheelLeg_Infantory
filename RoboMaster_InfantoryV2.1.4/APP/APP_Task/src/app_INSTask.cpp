#include "app_Data_Task.hpp"
#include "CalculateTask.hpp"
#include "bsp_UART.h"
#include "portmacro.h"
#include "semphr.h"
#include "arm_math.h"
#include "tim.h"
#include "usart.h"
#include "main.h"
#include "app_motor.h"
#include "tim.h"
#include "Hipnuv_hi14.h"
#define GRAVITY 9.81f  // 重力加速度 (m/s²)

void app_INSTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Classic_Data_t *ClassData = (Classic_Data_t *)pvParameters;
    uint32_t ulNotificationValue = 0;
    xEventGroupSetBits(ControlEventGroup,Control_OK_BIT);
    while (1) {
    xTaskNotifyWait(0, 0, &ulNotificationValue, portMAX_DELAY);
    bool new_data = false;
    if (hi14_rx_size > 0) {
        for (uint16_t i = 0; i < hi14_rx_size; i++)
        {
            int ret = hipnuc_input(&Hipnuc_HI14, hi14_uart_rx_buf[i]);
            if (ret == 1)
            {
                /* HI14 输出的 roll/pitch/yaw 单位为度，需转换为弧度 */
                ClassData->IMU_Data.roll  = angle_to_rad(Hipnuc_HI14.hi14data.roll);
                /* HI14 原始陀螺仪单位为 deg/s，统一存为 rad/s。 */
                const float deg_to_rad = 0.01745329251994329577f;
                ClassData->IMU_Data.gyro[0] = Hipnuc_HI14.hi14data.gyr[0] * deg_to_rad;
                ClassData->IMU_Data.gyro[1] = Hipnuc_HI14.hi14data.gyr[1] * deg_to_rad;
                ClassData->IMU_Data.gyro[2] = Hipnuc_HI14.hi14data.gyr[2] * deg_to_rad;
                /* HI14 加速度 (m/s²) 直接存储 */
                ClassData->IMU_Data.accel[0] = Hipnuc_HI14.hi14data.acc[0];
                ClassData->IMU_Data.accel[1] = Hipnuc_HI14.hi14data.acc[1];
                ClassData->IMU_Data.accel[2] = Hipnuc_HI14.hi14data.acc[2];
                ClassData->IMU_Data.pitch = -1.0f * angle_to_rad(Hipnuc_HI14.hi14data.pitch);
                ClassData->IMU_Data.yaw   = angle_to_rad(Hipnuc_HI14.hi14data.yaw);
                new_data = true;
            }
        }
        hi14_rx_size = 0;  // 处理完清零，防止下一轮重复解析旧数据导致 delta 衰减
    }
    if (!new_data) {
        continue;  // 未解析到有效帧，等下一包
    }
    ClassData->Data_Time.imu_time = __HAL_TIM_GET_COUNTER(&htim5);
    /*
     * HI14 安装坐标：X 右、Y 前、Z 上。
     * 与当前欧拉角处理保持一致：roll 不变、pitch = -raw_pitch、yaw 不变。
     * 因此欧拉角速度映射为：roll<-gyro_y，pitch<- -gyro_x，yaw<-gyro_z。
     */
    ClassData->IMU_Data.delta_roll = ClassData->IMU_Data.gyro[1];
    ClassData->IMU_Data.delta_pitch = -ClassData->IMU_Data.gyro[0];
    ClassData->IMU_Data.delta_yaw = ClassData->IMU_Data.gyro[2];

    /* 保持历史角度字段同步。 */
    ClassData->IMU_Data.last_roll = ClassData->IMU_Data.roll;
    ClassData->IMU_Data.last_pitch = ClassData->IMU_Data.pitch;
    ClassData->IMU_Data.last_yaw = ClassData->IMU_Data.yaw;

    // ================================================================
    // 向量旋转：机体加速度 → 世界系竖直/水平加速度
    // 利用 pitch / roll 姿态角，通过旋转矩阵将机体系加速度
    // 投影到世界系（重力方向为竖直），分离竖直与水平分量。
    //
    // 旋转矩阵 R = Ry(pitch) · Rx(roll)
    //   world = R · body
    //
    // world_z 方向与重力同向（世界系 Z 向下，NED 约定）
    // 去除重力后得到运动加速度：
    //   accel_v = 竖直运动加速度（正值 = 向下/重力方向）
    //   accel_h = 水平运动加速度幅值
    // ================================================================
    {
        float sp = arm_sin_f32(ClassData->IMU_Data.pitch);
        float cp = arm_cos_f32(ClassData->IMU_Data.pitch);
        float sr = arm_sin_f32(ClassData->IMU_Data.roll);
        float cr = arm_cos_f32(ClassData->IMU_Data.roll);

        float ax = ClassData->IMU_Data.accel[0];
        float ay = ClassData->IMU_Data.accel[1];
        float az = ClassData->IMU_Data.accel[2];

        /* world = Ry(pitch) * Rx(roll) * body */
        float wx =  cp * ax + sp * sr * ay + sp * cr * az;
        float wy =              cr * ay -      sr * az;
        float wz = -sp * ax + cp * sr * ay + cp * cr * az;
        
        float sqart = wx * wx + wy * wy;

        ClassData->IMU_Data.accel_v = wz - GRAVITY;         /* 竖直运动加速度，正值向下 */
        ClassData->IMU_Data.accel_x = wx;
        arm_sqrt_f32(sqart, &ClassData->IMU_Data.accel_h); /* 水平加速度幅值 */
    }

    ClassData->Data_Time.last_imu_time = ClassData->Data_Time.imu_time;
    htim3.Instance->CCR4 = 1;
    xEventGroupSetBits(DataGroup,IMU_DATA_READY_BIT);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}
