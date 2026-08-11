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
#include "app_ins_cal.h"
#include "tim.h"
#include "Hipnuv_hi14.h"
#define a 0.4f  // 一阶低通滤波（roll/pitch）
#define a_yaw 0.05f  // yaw 单独加强滤波，抑制噪声抖动

#define GRAVITY 9.81f  // 重力加速度 (m/s²)

void IMU_Temp_Control(float Target_Temp);

void app_INSTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Classic_Data_t *ClassData = (Classic_Data_t *)pvParameters;
    uint32_t ulNotificationValue = 0;
    xEventGroupSetBits(ControlEventGroup,Control_OK_BIT);
    while (1) {
#ifdef USE_HIPNUC_HI14
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
                /* HI14 陀螺仪角速度 (rad/s) 直接存储 */
                ClassData->IMU_Data.gyro[0] = Hipnuc_HI14.hi14data.gyr[0];
                ClassData->IMU_Data.gyro[1] = Hipnuc_HI14.hi14data.gyr[1];
                ClassData->IMU_Data.gyro[2] = Hipnuc_HI14.hi14data.gyr[2];
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
#else
    xEventGroupWaitBits(ControlEventGroup, Control_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    INS_Task();
    ClassData->IMU_Data.roll = angle_to_rad(INS.Roll);
    /* INS 陀螺仪角速度 (rad/s) */
    ClassData->IMU_Data.gyro[0] = INS.Gyro[0];
    ClassData->IMU_Data.gyro[1] = INS.Gyro[1];
    ClassData->IMU_Data.gyro[2] = INS.Gyro[2];
    /* INS 加速度 (m/s²) — 使用原始含重力测量值 */
    ClassData->IMU_Data.accel[0] = INS.Accel[0];
    ClassData->IMU_Data.accel[1] = INS.Accel[1];
    ClassData->IMU_Data.accel[2] = INS.Accel[2];
    ClassData->IMU_Data.pitch = angle_to_rad(INS.Pitch);
    ClassData->IMU_Data.yaw = angle_to_rad(INS.Yaw);
#endif

    /* 保存当前 accel，HI14 路径下在 new_data 分支已填 */
    /* INS 路径下在上方已填，此处统一使用 */

    ClassData->Data_Time.imu_time = __HAL_TIM_GET_COUNTER(&htim5);
    float time_diff = (ClassData->Data_Time.imu_time -
                    ClassData->Data_Time.last_imu_time) *
                   1e-6f;
    if (time_diff > 1e-6f) { // 确保时间差大于0
      // 处理 roll 跳变
      float roll_change = ClassData->IMU_Data.roll - ClassData->IMU_Data.last_roll;
      if (roll_change > 3.14159f)
          roll_change -= 6.28318f;
      else if (roll_change < -3.14159f)
          roll_change += 6.28318f;
      ClassData->IMU_Data.delta_roll =
          a * roll_change / time_diff +
          (1 - a) * ClassData->IMU_Data.delta_roll;
      ClassData->IMU_Data.last_roll = ClassData->IMU_Data.roll;

      // 处理 pitch 跳变
      float pitch_change = ClassData->IMU_Data.pitch - ClassData->IMU_Data.last_pitch;
      if (pitch_change > 3.14159f)
          pitch_change -= 6.28318f;
      else if (pitch_change < -3.14159f)
          pitch_change += 6.28318f;
      ClassData->IMU_Data.delta_pitch =
          a * pitch_change / time_diff +
          (1 - a) * ClassData->IMU_Data.delta_pitch;
      ClassData->IMU_Data.last_pitch = ClassData->IMU_Data.pitch;

      // 处理 yaw 跳变
      float yaw_change = ClassData->IMU_Data.yaw - ClassData->IMU_Data.last_yaw;
      if (yaw_change > 3.14159f)
          yaw_change -= 6.28318f;
      else if (yaw_change < -3.14159f)
          yaw_change += 6.28318f;
      ClassData->IMU_Data.delta_yaw =
          a_yaw * yaw_change / time_diff + (1 - a_yaw) * ClassData->IMU_Data.delta_yaw;
      ClassData->IMU_Data.last_yaw = ClassData->IMU_Data.yaw;
    }
      else {
      ClassData->IMU_Data.delta_roll = 0.0f;
      ClassData->IMU_Data.delta_pitch = 0.0f;
      ClassData->IMU_Data.delta_yaw = 0.0f;
    }

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
        arm_sqrt_f32(sqart, &ClassData->IMU_Data.accel_h); /* 水平加速度幅值 */
    }

    ClassData->Data_Time.last_imu_time = ClassData->Data_Time.imu_time;
    #ifdef USE_HIPNUC_HI14
    htim3.Instance->CCR4 = 1;
    #else
    IMU_Temp_Control(40.0f);
    #endif
    xEventGroupSetBits(DataGroup,IMU_DATA_READY_BIT);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

float err = 0.0f, err_l = 0.0f, err_ll = 0.0f;

void IMU_Temp_Control(float Target_Temp) {
  const uint16_t Kp = 10, Ki = 0, Kd = 0, MAX_OUT = 800;
  uint16_t out = 0;
  err_ll = err_l;
  err_l = err;
  err = Target_Temp - BMI088.Temperature;
  out = (uint16_t)(Kp * err + Ki * (err + err_l + err_ll) + Kd * (err - err_l));
  if (out > MAX_OUT)
    out = MAX_OUT;
  if (out < 0)
    out = 0;
  // uart_print("IMU_Temp_Control:%d\r\n", out);
  htim3.Instance->CCR4 = (uint16_t)out;
}