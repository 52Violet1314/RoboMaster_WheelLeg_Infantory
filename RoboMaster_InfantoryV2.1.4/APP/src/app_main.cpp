#include "app_main.hpp"
#include "app_motor.h"
#include "bsp_UART.h"
#include "bsp_fdcan.h"
#include "fdcan.h"
#include "stm32h7xx_hal.h"
#include <cstdio>
#include "stm32h7xx_hal_tim.h"
#include "app_motor.h"

void app_main(void) {
  uart_print_blocking("Running to app_main\r\n");
  /* 初始化 UART DMA 打印信号量，避免任务启动后 uart_print() 没有发送完成同步。 */
  bsp_UART_Init();
  // Task_Init 必须在 bsp_can_init 之前调用：确保任务和通信对象已创建。
  // 在 FDCAN 中断使能前已创建，避免 ISR 中访问 NULL 句柄触发 configASSERT 死循环
  Task_Init();
  bsp_can_init();
  uart_print_blocking("CanInited\r\n");
  vTaskStartScheduler();
}
