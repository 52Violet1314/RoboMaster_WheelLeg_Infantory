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
                ((Classic_Data->Contronller_Data.Fw_L / 15.8f) * 19.2f));
            DM_Motor_MIT_Control(
                &DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f,
                -((Classic_Data->Contronller_Data.Fw_R / 15.8f) * 19.2f));
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
