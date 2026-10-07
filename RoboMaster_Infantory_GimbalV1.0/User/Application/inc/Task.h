#ifndef USER_APPLICATION_TASK_H
#define USER_APPLICATION_TASK_H
#include <stdint.h>

/* 正常 Pitch PID、ESO 和 MIT 控制帧均为 1 ms。下列 3 ms 节拍只用于
 * 摩擦轮命令及 Pitch 异常/恢复期间的零扭矩安全帧，避免恢复流量过密。 */
#define TASK_IMU_PERIOD_MS      (1U)
#define TASK_INPUT_PERIOD_MS    (1U)
#define TASK_CAN_PERIOD_MS      (1U)
#define TASK_MODE_PERIOD_MS     (1U)
#define TASK_GIMBAL_PERIOD_MS   (1U)
#define GIMBAL_MOTOR_COMMAND_PERIOD_MS (3U)

/* Pitch 机械允许行程，单位 rad。GimbalTask 用它限制手操继续向外的速度，
 * 并夹紧视觉/键鼠的绝对角目标；始终允许向安全区反向运动。 */
#define GIMBAL_PITCH_LIMIT_UPPER_RAD    (0.4000000f)
#define GIMBAL_PITCH_LIMIT_LOWER_RAD   (-0.2500000f)

/* 由 RTOS 任务包装函数调用的应用任务统一接口。 */
typedef enum {
  GIMBAL_FAULT_NONE          = 0U,
  GIMBAL_FAULT_IMU_TIMEOUT   = 1U << 0,
  GIMBAL_FAULT_COMMAND_LOST  = 1U << 1,
  GIMBAL_FAULT_MOTOR_TIMEOUT = 1U << 2,
  GIMBAL_FAULT_CAN_OFFLINE   = 1U << 3,
  GIMBAL_FAULT_MOTOR_DRIVER  = 1U << 4,
  GIMBAL_FAULT_LEFT_TIMEOUT  = 1U << 5,
  GIMBAL_FAULT_RIGHT_TIMEOUT = 1U << 6,
  /* 左右摩擦轮故障只拦截自身，不参与 Pitch 的停机判定。 */
  GIMBAL_FAULT_LEFT_DRIVER   = 1U << 8,
  GIMBAL_FAULT_RIGHT_DRIVER  = 1U << 9
} Gimbal_Fault_Flag_t;

typedef enum {
  MODE_FAULT_NONE             = 0U,
  MODE_FAULT_REMOTE_TIMEOUT   = 1U << 0,
  MODE_FAULT_KEYBOARD_TIMEOUT = 1U << 1,
  MODE_FAULT_VISION_TIMEOUT   = 1U << 2,
  MODE_FAULT_COMMAND_LOST     = 1U << 3
} Mode_Fault_Flag_t;
typedef enum {
  IMU_FAULT_NONE           = 0U,
  IMU_FAULT_SAMPLE_TIMEOUT = 1U << 0
} Imu_Fault_Flag_t;

/* 仅供低频串口诊断的输入链路统计。计数只在“进入/退出超时”边沿加一，
 * 不参与任何控制、保护或恢复决策。 */
typedef struct {
  uint32_t timeout_enter_count;
  uint32_t timeout_recover_count;
  uint32_t command_lost_enter_count;
  uint32_t command_lost_recover_count;
  uint32_t age_ms;
  uint32_t valid_frame_count;
  uint8_t timeout_active;
  uint8_t failsafe_active;
  uint8_t command_valid;
  uint8_t command_source;
} Input_Link_Diagnostic_t;

/** USART1 串口调试任务入口。 */
void Task_DebugTask(void *argument);
void Task_IMU_Task(void *argument);
void Task_InputTask(void *argument);
void Task_CanTask(void *argument);
void Task_ModeTask(void *argument);
void Task_Gimbal_Task(void *argument);
/** ModeTask/ImuTask 各自维护的健康状态；GimbalTask 只读取并执行输出保护。 */
uint32_t Task_GetModeFaultFlags(void);
uint32_t Task_GetImuFaultFlags(void);
void Task_GetImuLinkDiagnostic(Input_Link_Diagnostic_t *diagnostic);
void Task_GetModeInputDiagnostic(Input_Link_Diagnostic_t *diagnostic);
#endif
