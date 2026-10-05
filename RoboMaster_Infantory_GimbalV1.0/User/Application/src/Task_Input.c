#include "Task.h"
#include "cmsis_os2.h"
#include "Remote.h"
#include "Vision.h"
#include "Refree.h"
#include "Interrupt.h"

/* 输入任务只编排阶段；每一种链路的协议状态机都封装在各自通信模块内。 */
static void input_service_reception(void)
{
  /* 仅处理 UART DMA 异常后的非中断重启请求，不做协议或业务解释。 */
  Communication_Uart_Service();
}

static void input_parse_remote_stage(void)
{
  /* SBUS 重同步与原始通道解码，输出仅供 ModeTask 消费的原始快照。 */
  Remote_Process();
}

static void input_parse_vision_stage(void)
{
  /* 视觉帧同步、CRC 与原始目标字段解包。 */
  Vision_Process();
}

static void input_parse_referee_stage(void)
{
  /* 裁判链路帧同步、CRC、键鼠/自定义载荷原始字段解包。 */
  Refree_Process();
}

void Task_InputTask(void *argument)
{
  (void)argument;
  Remote_Init();
  for (;;) {
    input_service_reception();
    input_parse_remote_stage();
    input_parse_vision_stage();
    input_parse_referee_stage();
    osDelay(TASK_INPUT_PERIOD_MS);
  }
}
