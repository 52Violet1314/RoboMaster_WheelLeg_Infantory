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
/* 轮速反馈与轮毂力输出共用同一套轮端机械参数。
 * Fw 是 LQR 给出的轮地切向力(N)：先乘轮半径得到轮端扭矩，再除减速比
 * 得到 MIT 接口所需的电机轴扭矩(N·m)。 */
constexpr float kWheelRadiusM = 0.0575f;
constexpr float kWheelReductionRatio = 17.0f;
constexpr float kWheelForceToMotorTorque = kWheelRadiusM / kWheelReductionRatio;
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
                Classic_Data->Contronller_Data.Fw_L * kWheelForceToMotorTorque);
            DM_Motor_MIT_Control(
                &DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f,
                -(Classic_Data->Contronller_Data.Fw_R * kWheelForceToMotorTorque));
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
