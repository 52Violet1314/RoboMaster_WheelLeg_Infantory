#include "Task.h"
#include "cmsis_os2.h"
#include "Can_Motor.h"

void Task_CanTask(void *argument)
{
  (void)argument;
  Can_Motor_Init();
  for (;;) {
    Can_Motor_Process();
    osDelay(TASK_CAN_PERIOD_MS);
  }
}
