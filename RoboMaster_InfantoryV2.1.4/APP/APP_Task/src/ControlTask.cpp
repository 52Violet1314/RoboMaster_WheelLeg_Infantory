#include "ControlTask.hpp"
#include "CalculateTask.hpp"
#include "FreeRTOS.h"
#include "SBUS.h"
#include "WS2812.h"
#include "app_Task.hpp"
#include "app_main.hpp"
#include "app_motor.h"
#include "bsp_UART.h"
#include "fdcan.h"
#include "main.h"
#include "queue.h"
#include "stm32h7xx_hal_tim.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include <cstddef>

namespace {
/* LQR 的前两路输入实际是轮端力矩 Twl/Twr (N·m)，Fw_L/R 沿用旧字段名。
 * 当前机械标定：wheel_speed = motor_speed * 19.2 / 15.76；理想传动下
 * motor_torque = wheel_torque * 19.2 / 15.76。轮径只用于速度换算，不参与这里。 */
constexpr float kWheelTorqueToMotorTorque = 19.2f / 15.76f;
} // namespace

void MotorControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Classic_Data_t *Classic_Data = (Classic_Data_t *) pvParameters;
    while (1) {
        xEventGroupWaitBits(ControlEventGroup, Calculate_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
        if (Classic_Data->Contronller_Data.total_enable == 0U) {
            DM_Motor_MIT_Control(&DM_8009P1, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_8009P2, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_8009P4, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_8009P3, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_3519L, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_Speed_Control(&DM_3507, 0.0f);
        } else {
            DM_Motor_MIT_Control(&DM_8009P1, 0.0f, 0.0f, 0.0f, 0.0f,
                                 Classic_Data->Contronller_Data.T1_L);
            DM_Motor_MIT_Control(&DM_8009P2, 0.0f, 0.0f, 0.0f, 0.0f,
                                 Classic_Data->Contronller_Data.T2_L);
            DM_Motor_MIT_Control(&DM_8009P4, 0.0f, 0.0f, 0.0f, 0.0f,
                                 -Classic_Data->Contronller_Data.T1_R);
            DM_Motor_MIT_Control(&DM_8009P3, 0.0f, 0.0f, 0.0f, 0.0f,
                                 -Classic_Data->Contronller_Data.T2_R);
            DM_Motor_MIT_Control(
                &DM_3519L, 0.0f, 0.0f, 0.0f, 0.0f,
                Classic_Data->Contronller_Data.Fw_L * kWheelTorqueToMotorTorque);
            DM_Motor_MIT_Control(
                &DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f,
                -(Classic_Data->Contronller_Data.Fw_R * kWheelTorqueToMotorTorque));
            DM_Motor_Speed_Control(
                &DM_3507,
                Classic_Data->Contronller_Data.Feeder_Speed_Command_Rad_S);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
