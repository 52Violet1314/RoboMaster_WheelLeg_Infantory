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
float Gimbal_Pitch_Vel = 0.0f;

void app_Remote_Task(void *pvParameters) {
    DM_Motor_Init(&DM_3507, 0x057, DM_MOTOR_MODE_SPEED, &hfdcan2);
    DM_Motor_Init(&DM_4310, 0x058, DM_MOTOR_MODE_SPEED, &hfdcan2);
    vTaskDelay(pdMS_TO_TICKS(1));
    DM_Motor_Init(&DM_3519_Gimbal_Pitch, 0x011, DM_MOTOR_MODE_SPEED, &hfdcan3);
    vTaskDelay(pdMS_TO_TICKS(1));
    DM_Motor_Init(&DM_3519_Shoot_L, 0x012, DM_MOTOR_MODE_SPEED, &hfdcan3);
    vTaskDelay(pdMS_TO_TICKS(1));
    DM_Motor_Init(&DM_3519_Shoot_R, 0x013, DM_MOTOR_MODE_SPEED, &hfdcan3);
    uint8_t Ammo_Push_Flag = 0;
    while (1) {
        Classic_Data_t *Classic_Data = (Classic_Data_t *) pvParameters;
        /* Like the gimbal InputTask, poll the raw ring buffer on this task's
         * own cycle.  No complete frame means SBUS_Data stays at its last
         * successfully parsed value. */
        while (SBUS_Frame_Parse_From_Buffer(&sbus_ring_buffer, &SBUS_Data) != 0U) {
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
        /* 底盘遥控死区：CH0 转向、CH1 前后在 ±5 原始值内视为零。 */
        const int16_t chassis_yaw_channel =
                (SBUS_Data.Channel[0] > 5 || SBUS_Data.Channel[0] < -5)
                    ? SBUS_Data.Channel[0] : 0;
        const int16_t chassis_x_channel =
                (SBUS_Data.Channel[1] > 5 || SBUS_Data.Channel[1] < -5)
                    ? SBUS_Data.Channel[1] : 0;
        Classic_Data->Target_Data.Target_yaw_vel = chassis_yaw_channel / 783.0f * 4.0f;
        Classic_Data->Target_Data.Target_X_vel =  chassis_x_channel / 783.0f * 4.0f - Classic_Data->Target_Data.Target_yaw_vel*0.240f;
        if(SBUS_Data.Channel[2] > 10 || SBUS_Data.Channel[2] < -10)
        {
            Gimbal_Pitch_Vel = SBUS_Data.Channel[2] / 783.0f * 4.0f;
        }
        else
        {
            Gimbal_Pitch_Vel = 0.0f;
        }
        if(SBUS_Data.Channel[3] > 10 || SBUS_Data.Channel[3] < -10)
        {
            Gimbal_Yaw_Vel = SBUS_Data.Channel[3] / 783.0f * -4.71f;
        }
        else
        {
            Gimbal_Yaw_Vel = 0.0f;
        }
        if(SBUS_Data.Channel[4] > 100)
        {
            DM_Motor_Speed_Control(&DM_3519_Shoot_L, -18.84f);
            DM_Motor_Speed_Control(&DM_3519_Shoot_R, 18.84f);
            if(Ammo_Push_Flag < 10)
            {
                Ammo_Push_Flag ++;
            }
            else if(Ammo_Push_Flag >= 10)
            {
                DM_Motor_Speed_Control(&DM_3507, 3.14f);
            }
        }
        else
        {
            DM_Motor_Speed_Control(&DM_3519_Shoot_L, 0.0f);
            DM_Motor_Speed_Control(&DM_3519_Shoot_R, 0.0f);
            DM_Motor_Speed_Control(&DM_3507, 0.0f); 
             Ammo_Push_Flag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        DM_Motor_Speed_Control(&DM_3519_Gimbal_Pitch, Gimbal_Pitch_Vel);
        vTaskDelay(pdMS_TO_TICKS(1));
        DM_Motor_Speed_Control(&DM_4310, Gimbal_Yaw_Vel);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
