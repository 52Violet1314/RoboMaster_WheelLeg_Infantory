#include "Task.h"
#include "Imu.h"
#include "Hipnuc_Command.h"
#include "tim.h"

/* 仅由 ImuTask 写入；GimbalTask 只读此状态决定是否零输出。 */
static volatile uint32_t s_imu_fault_flags;
static volatile Input_Link_Diagnostic_t s_imu_link_diagnostic;

#if (HIPNUC_FIRST_USE_CONFIG_ON_BOOT != 0U)
/**
 * @brief 首次装机时向 HiPNUC 写入本机器人的固定配置。
 * @details 仅由 HIPNUC_FIRST_USE_CONFIG_ON_BOOT 临时启用一次，且在 USART2 DMA
 *          接收启动前运行。这样 IMU 的 ASCII 回复不会混入 HI91 二进制解析器。
 *          本函数不包含地磁校准：九轴校准必须由人工在整机固定、所有电机停机后完成。
 */
static void imu_apply_first_use_configuration(void)
{
  /* 暂停数据流，便于配置阶段的串口通信；本工程 USART2 已固定为 921600、8N1。 */
  (void)Hipnuc_Command_SetLogEnabled(0U);
  (void)Hipnuc_Command_SetUrfr(134U); /* 模块 X 左、Y 后、Z 上。 */
  (void)Hipnuc_Command_SetWorldCoordinate(HIPNUC_COORD_ENU); /* 世界系：东、北、天。 */
  (void)Hipnuc_Command_SetAttitudeMode(HIPNUC_ATT_MODE_AHRS_9_AXIS); /* 屏蔽壳下使用九轴。 */
  (void)Hipnuc_Command_SetHi91PeriodMs(1U); /* 1 ms 一帧，即 1 kHz。 */
  (void)Hipnuc_Command_Save(); /* 将上述长期配置写入 IMU。 */
  (void)Hipnuc_Command_Reboot();
  osDelay(HIPNUC_REBOOT_WAIT_MS); /* 等待 IMU 重启完成后再挂接 DMA。 */
}
#endif

void Task_IMU_Task(void *argument)
{
  (void)argument;
  Imu_Init();
#if (HIPNUC_FIRST_USE_CONFIG_ON_BOOT != 0U)
  imu_apply_first_use_configuration();
#endif
  Imu_Usart2_Start_Receive();
  for (;;) {
    Imu_Task_Process();
    const uint32_t last_sample_us = Imu_GetLastSampleTimestampUs();
    const uint32_t now_us = TIM7_GetTimestampUs();
    const uint32_t age_us = (last_sample_us == 0U) ? UINT32_MAX :
        (uint32_t)(now_us - last_sample_us);
    const uint8_t timeout_active = (last_sample_us == 0U ||
        age_us > (IMU_TASK_SAMPLE_TIMEOUT_MS * 1000U)) ? 1U : 0U;
    if (timeout_active != s_imu_link_diagnostic.timeout_active) {
      if (timeout_active != 0U)
        s_imu_link_diagnostic.timeout_enter_count++;
      else
        s_imu_link_diagnostic.timeout_recover_count++;
      s_imu_link_diagnostic.timeout_active = timeout_active;
    }
    s_imu_link_diagnostic.age_ms = (age_us == UINT32_MAX) ? UINT32_MAX :
        (age_us / 1000U);
    s_imu_link_diagnostic.valid_frame_count = Imu_GetValidSampleCount();
    s_imu_fault_flags = (timeout_active != 0U) ?
        IMU_FAULT_SAMPLE_TIMEOUT : IMU_FAULT_NONE;
    /* 即使当前没有新 DMA 块也必须让出 CPU，防止高优先级 IMU 任务空转。 */
    osDelay(TASK_IMU_PERIOD_MS);
  }
}

uint32_t Task_GetImuFaultFlags(void)
{
  return s_imu_fault_flags;
}

void Task_GetImuLinkDiagnostic(Input_Link_Diagnostic_t *diagnostic)
{
  if (diagnostic == NULL) return;
  *diagnostic = s_imu_link_diagnostic;
}
