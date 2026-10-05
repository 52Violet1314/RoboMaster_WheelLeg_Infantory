#ifndef USER_APPLICATION_DEBUG_H
#define USER_APPLICATION_DEBUG_H

#define TASK_DEBUG_PERIOD_MS            (10U)
#define DEBUG_UART_TX_TIMEOUT_MS        (100U)

/** 初始化 USART1 printf 输出；DebugTask 启动时调用一次。 */
void Debug_ConsoleInit(void);
/** DebugTask 主循环。 */
void Task_DebugTask(void *argument);

#endif
