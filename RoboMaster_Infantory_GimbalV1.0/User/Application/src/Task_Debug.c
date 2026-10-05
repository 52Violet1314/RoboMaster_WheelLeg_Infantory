#include "Task.h"
#include "Debug.h"
#include "Pid.h"
#include "cmsis_os2.h"
#include "usart.h"
#include <stdio.h>

/* 强符号覆盖 Core/syscalls.c 的弱 _write。仅 DebugTask 初始化后允许向 USART1
 * 输出，避免调度器启动早期的库输出意外占用调试串口。禁止从 ISR 调用 printf。 */
static volatile uint8_t s_debug_console_ready;

int _write(int file, char *ptr, int len)
{
  (void)file;
  if ((s_debug_console_ready == 0U) || (ptr == NULL) || (len <= 0)) return 0;
  return (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len,
                            DEBUG_UART_TX_TIMEOUT_MS) == HAL_OK) ? len : 0;
}

void Debug_ConsoleInit(void)
{
  s_debug_console_ready = 1U;
  /* 关闭 stdio 缓冲，printf 每次调用立即送往 USART1。浮点格式由链接选项
   * -Wl,-u,_printf_float 启用，不能只靠本文件的 _write 实现。 */
  (void)setvbuf(stdout, NULL, _IONBF, 0);
}

/* FireWater 临时曲线输出：
 * channels[0] = 当前模式下送入速度环的目标 Pitch 角速度(rad/s)
 * channels[1] = HI91 gyr[X] 的实际 Pitch 角速度(rad/s)
 * 仅只读 PID 诊断快照；后续不需要曲线时删除本循环内的三行即可。 */
void Task_DebugTask(void *argument)
{
  (void)argument;
  Debug_ConsoleInit();
  for (;;) {
    Pitch_Control_Diagnostic_t diagnostic;
    Pitch_Control_GetDiagnostic(&diagnostic);
    printf("channels:%.6f,%.6f\r\n",
           (double)diagnostic.velocity_reference_rad_s,
           (double)diagnostic.measured_velocity_rad_s);
    osDelay(TASK_DEBUG_PERIOD_MS);
  }
}
