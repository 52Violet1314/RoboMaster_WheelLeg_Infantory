#include "app_Remote_Task.hpp"
#include "app_Data_Task.hpp"
#include "CalculateTask.hpp"
#include "FreeRTOS.h"
#include "app_motor.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "bsp_UART.h"
#include <cstdint>

float Gimbal_Yaw_Vel = 0.0f;

void app_Remote_Task(void *pvParameters) {
    DM_Motor_Init(&DM_3507, DM_MOTOR_MODEL_3507, 0x057, DM_MOTOR_MODE_SPEED, &hfdcan2);
    DM_Motor_Init(&DM_4310, DM_MOTOR_MODEL_J4310, 0x058, DM_MOTOR_MODE_SPEED, &hfdcan2);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Remote_ControlSnapshot_t remote_snapshot = {};
    (void)pvParameters;
    while (1) {
        /* Like the gimbal InputTask, poll the raw ring buffer on this task's
         * own cycle.  No complete frame means SBUS_Data stays at its last
         * successfully parsed value. */
        bool remote_updated = false;
        while (SBUS_Frame_Parse_From_Buffer(&sbus_ring_buffer, &SBUS_Data) != 0U) {
            remote_updated = true;
        }
        if(SBUS_Data.Channel[9] < -100)
        {
            DM_Motor_MIT_Control(&DM_8009P1, 0.0f, 0.0f, 0.0f, 0.0f, 0); 
            DM_Motor_MIT_Control(&DM_8009P2, 0.0f, 0.0f, 0.0f, 0.0f, 0); 
            DM_Motor_MIT_Control(&DM_8009P4, 0.0f, 0.0f, 0.0f, 0.0f, 0); 
            DM_Motor_MIT_Control(&DM_8009P3, 0.0f, 0.0f, 0.0f, 0.0f, 0); 
            DM_Motor_MIT_Control(&DM_3519L, 0.0f, 0.0f, 0.0f, 0.0f, 0); 
            DM_Motor_MIT_Control(&DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f, 0); 
        }
        /* 底盘遥控死区：CH0 转向（右杆左右）、CH2 前后（左杆前后）。
         * 实测排除法结论：CH1=右杆前后（用过），CH3=左杆左右（用过），
         * 剩下左杆前后只能是 CH2。
         * 云台 pitch 在云台板也走 CH2，本板不消费。 */
        const int16_t chassis_yaw_channel =
                (SBUS_Data.Channel[0] > 5 || SBUS_Data.Channel[0] < -5)
                    ? SBUS_Data.Channel[0] : 0;
        const int16_t chassis_x_channel =
                (SBUS_Data.Channel[2] > 5 || SBUS_Data.Channel[2] < -5)
                    ? SBUS_Data.Channel[2] : 0;
        const float target_yaw_vel = chassis_yaw_channel / 783.0f * 4.0f;
        const float target_x_vel = chassis_x_channel / 783.0f * 4.0f - target_yaw_vel * 0.240f;
        /* 云台 yaw 速度不再由本板解析：左杆前后已让给底盘前后。
         * 此处保持云台 yaw 电机静止，等待后续云台板联调再接入。 */
        Gimbal_Yaw_Vel = 0.0f;
        if(SBUS_Data.Channel[4] > 100)
        {
            DM_Motor_Speed_Control(&DM_3507, 3.14f);
        }
        else
        {
            DM_Motor_Speed_Control(&DM_3507, 0.0f); 
        }
        DM_Motor_Speed_Control(&DM_4310, Gimbal_Yaw_Vel);
        if (remote_updated && (Remote_ControlQueueHandle != NULL)) {
            remote_snapshot.sbus_data = SBUS_Data;
            remote_snapshot.target_x_vel = target_x_vel;
            remote_snapshot.target_yaw_vel = target_yaw_vel;
            if (xQueueOverwrite(Remote_ControlQueueHandle, &remote_snapshot) == pdPASS) {
                DataTimeout_MarkRemoteValid();
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
