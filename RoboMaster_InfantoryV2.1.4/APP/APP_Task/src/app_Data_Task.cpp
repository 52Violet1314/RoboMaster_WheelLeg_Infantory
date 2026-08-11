#include "app_Data_Task.hpp"
#include "bsp_UART.h"
#include "projdefs.h"
#include "usart.h"
#include "main.h"
#include "app_motor.h"
#include "tim.h"
Classic_Data_t Classic_Data = {0};

// app_Data_Task 已合并到 CaculateTask，此函数保留为空桩以防链接引用
void app_Data_Task(void *pvParameters) {
    (void)pvParameters;
    vTaskDelete(NULL);
}

void Motor_Data_Process(Classic_Data_t *pClassicData) {
    pClassicData->Motor_Data.L1_Motor_POS =
            (DM_8009P1.data.position_rad / 3.50f * 3.14f) + 3.14f;
    pClassicData->Motor_Data.L2_Motor_POS =
            (DM_8009P2.data.position_rad / 3.50f * 3.14f) + 6.28f;
    pClassicData->Motor_Data.R1_Motor_POS =
            -(DM_8009P4.data.position_rad / 3.50f * 3.14f) - 3.14f;
    pClassicData->Motor_Data.R2_Motor_POS =
            -(DM_8009P3.data.position_rad / 3.50f * 3.14f);
    pClassicData->Motor_Data.L1_Motor_VEL = DM_8009P1.data.velocity_rad_s;
    pClassicData->Motor_Data.L2_Motor_VEL = DM_8009P2.data.velocity_rad_s;
    pClassicData->Motor_Data.R1_Motor_VEL = DM_8009P4.data.velocity_rad_s;
    pClassicData->Motor_Data.R2_Motor_VEL = DM_8009P3.data.velocity_rad_s;
    pClassicData->Motor_Data.Left_Wheel_Motor_VEL =
            -(DM_3519L.data.velocity_rad_s / 15.8f * 19.2f + 0.048f);
    pClassicData->Motor_Data.Right_Wheel_Motor_VEL =
            (DM_3519R.data.velocity_rad_s / 15.8f * 19.2f + 0.048f);
}
