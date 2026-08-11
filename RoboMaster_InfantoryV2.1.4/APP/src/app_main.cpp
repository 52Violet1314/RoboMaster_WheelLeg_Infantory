#include "app_main.hpp"
#include "app_motor.h"
#include "bsp_UART.h"
#include "bsp_fdcan.h"
#include "fdcan.h"
#include "stm32h7xx_hal.h"
#include <cstdio>
#include "DMIMU.h"
#include "BMI088driver.h"
#include "app_ins_cal.h"
#include "stm32h7xx_hal_tim.h"
#include "app_motor.h"

void app_main(void) {
  uart_print_blocking("Running to app_main\r\n");
  bsp_fdcan_set_baud(&hfdcan1, CAN_FD_BRS, CAN_BR_5M);
  bsp_fdcan_set_baud(&hfdcan2, CAN_FD_BRS, CAN_BR_5M);
  bsp_fdcan_set_baud(&hfdcan3, CAN_FD_BRS, CAN_BR_2M);
  // Task_Init 必须在 bsp_can_init 之前调用：确保 DataGroup 等事件组
  // 在 FDCAN 中断使能前已创建，避免 ISR 中访问 NULL 句柄触发 configASSERT 死循环
  Task_Init();
  bsp_can_init();
  uart_print_blocking("CanInited\r\n");
  htim3.Instance->CCR4 = 1;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  vTaskStartScheduler();
}