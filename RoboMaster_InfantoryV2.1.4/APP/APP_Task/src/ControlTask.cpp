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

int controltimer = 0;
int tasktime = 0;
int last_tasktime = 0;

void MotorControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Classic_Data_t *Classic_Data = (Classic_Data_t *) pvParameters;
    while (1) {
        // tasktime = __HAL_TIM_GET_COUNTER(&htim5) - last_tasktime;
        // last_tasktime = __HAL_TIM_GET_COUNTER(&htim5);a
        xEventGroupWaitBits(ControlEventGroup, Calculate_OK_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
        // uart_print("enter ControlTask\r\n");
        // uart_print("ControlState: %d\r\n", ControlState);
        if (SBUS_Data.Channel[9] < 100) {
            // DM_Motor_Enable(&DM_8009P1);
            // DM_Motor_Enable(&DM_8009P2);
            // DM_Motor_Enable(&DM_8009P3);
            // DM_Motor_Enable(&DM_8009P4);
            // DM_Motor_Enable(&DM_3519L);
            // DM_Motor_Enable(&DM_3519R);
            DM_Motor_MIT_Control(&DM_8009P1, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_8009P2, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_8009P4, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_8009P3, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_3519L, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f, 0);
        } else {
            // DM_Motor_Enable(&DM_8009P1);
            // DM_Motor_Enable(&DM_8009P2);
            // DM_Motor_Enable(&DM_8009P3);
            // DM_Motor_Enable(&DM_8009P4);
            // DM_Motor_Enable(&DM_3519L);
            // DM_Motor_Enable(&DM_3519R);
            // DM_Motor_MIT_Control(&DM_8009P1, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            // DM_Motor_MIT_Control(&DM_8009P2, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            // DM_Motor_MIT_Control(&DM_8009P4, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            // DM_Motor_MIT_Control(&DM_8009P3, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(&DM_8009P1, 0.0f, 0.0f, 0.0f, 0.0f,
                                 ((Classic_Data->Contronller_Data.T1_L * 17.0f) / 19.0f));
            DM_Motor_MIT_Control(&DM_8009P2, 0.0f, 0.0f, 0.0f, 0.0f,
                                 ((Classic_Data->Contronller_Data.T2_L * 17.0f) / 19.0f));
            DM_Motor_MIT_Control(&DM_8009P4, 0.0f, 0.0f, 0.0f, 0.0f,
                                 ((-Classic_Data->Contronller_Data.T1_R * 17.0f) / 19.0f));
            DM_Motor_MIT_Control(&DM_8009P3, 0.0f, 0.0f, 0.0f, 0.0f,
                                 ((-Classic_Data->Contronller_Data.T2_R * 17.0f) / 19.0f));
            // DM_Motor_MIT_Control(&DM_3519L, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            // DM_Motor_MIT_Control(&DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f, 0);
            DM_Motor_MIT_Control(
                &DM_3519L, 0.0f, 0.0f, 0.0f, 0.0f,
                ((Classic_Data->Contronller_Data.Fw_L / 15.8f) * 19.2f));
            DM_Motor_MIT_Control(
                &DM_3519R, 0.0f, 0.0f, 0.0f, 0.0f,
                -((Classic_Data->Contronller_Data.Fw_R / 15.8f) * 19.2f));
        }
        // uart_print("[%d]Cont\r\n", __HAL_TIM_GET_COUNTER(&htim5));
        xEventGroupSetBits(ControlEventGroup, Control_OK_BIT);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}