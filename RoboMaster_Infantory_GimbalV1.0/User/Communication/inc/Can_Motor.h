#ifndef USER_COMMUNICATION_CAN_MOTOR_H
#define USER_COMMUNICATION_CAN_MOTOR_H
#include "fdcan.h"
#include "cmsis_os2.h"
#include <stdint.h>

/* 电机控制帧标准 ID（ESC_ID）。 */
#define DM_MOTOR_PITCH_ID      0x003U
#define DM_MOTOR_LEFT_ID       0x001U
#define DM_MOTOR_RIGHT_ID      0x002U

/* 电机反馈帧标准 ID（MST_ID）。 */
#define DM_MOTOR_PITCH_FB_ID   0x013U
#define DM_MOTOR_LEFT_FB_ID    0x011U
#define DM_MOTOR_RIGHT_FB_ID   0x012U

/* 三台 CAN 电机总开关。
 * 0：台架测试模式。不启动 FDCAN1、不发送任何电机使能/控制帧，也不执行
 *    空总线的错误恢复；云台任务会将 CAN 判为离线并保持零输出。
 * 1：启动实际电机通信。当前用于 Pitch 手操调试；CAN 反馈、IMU、命令和
 *    机械角度任一保护条件不满足时，云台任务仍会发送零扭矩并停止摩擦轮。 */
#define CAN_MOTOR_COMMUNICATION_ENABLE (1U)

/* 左右摩擦轮是否实际安装在本条 CAN 总线。
 * 设为 0 时，该轮的速度、零速、Enable、Disable、超时与自动恢复全部屏蔽；
 * ISR 仍可统计 FIFO 原始帧，便于后续接回时排查。当前台架只接 Pitch，故均关闭。 */
#define CAN_MOTOR_LEFT_PRESENT  (0U)
#define CAN_MOTOR_RIGHT_PRESENT (0U)

//控制ID模式偏移
#define DM_MODE_MIT            0x000U
#define DM_MODE_SPEED          0x200U

/* ============================================================
 * DM-S3519-1EC 运行输出上限
 *
 * 三台电机均为 DM-S3519-1EC。下列数值用于本机发送指令前的硬限幅：
 * - 峰值扭矩 1.0 Nm：MIT 扭矩给定绝不越过该值；
 * - 命令最大速度 30 rad/s：这是本电机驱动当前 VMAX 参数及官方例程
 *   所使用的可编码速度边界，低于机械空载转速，故作为实际发送上限。
 *
 * 注意：协议映射范围 PMAX/VMAX/TMAX 为 12.5/30/10，TMAX=10 仅用于
 * 12 位 CAN 字段的数值编码，不能改成 7.8，否则驱动器解码比例错误。
 * ============================================================ */
#define DM_S3519_PEAK_TORQUE_NM          (1.0f)
#define DM_S3519_MAX_COMMAND_SPEED_RAD_S (30.0f)

/* CanTask 中相邻恢复帧的发送间隔；恢复状态机不阻塞、不调用 osDelay。 */
#define CAN_MOTOR_INITIAL_FRAME_INTERVAL_MS (3U)
/* 任一电机故障失能未获 s=0 确认时的 Disable 重试周期；所有异常安全控制帧
 * 统一按 3 ms 发送，正常闭环才使用 1 ms。 */
#define CAN_MOTOR_DISABLE_RETRY_INTERVAL_MS (3U)

/* ISR -> CanTask 的三电机最新反馈快照：仅 1 个元素，绝不积压旧数据。 */
#define CAN_MOTOR_RX_QUEUE_DEPTH (1U)

/* 自动恢复开关：
 * 1：对超时电机、以及确认 s=0 的任一电机轮流发送使能帧，3 ms 后发送该电机零指令；
 * 0：不发送自动使能帧，FDCAN 仍接收并维护在线状态，业务任务仍会发零指令。
 * 当前值为 1，适合已确认机械、驱动器和急停均正常后的通信恢复测试。 */
#define CAN_MOTOR_TIMEOUT_AUTO_REENABLE (1U)

/* 达妙 CAN/MIT 协议编码范围：仅供打包和反馈解包使用，不代表运行安全值。 */
#define DM_S3519_PROTOCOL_PMAX_RAD       (12.5f)
#define DM_S3519_PROTOCOL_VMAX_RAD_S     (30.0f)
#define DM_S3519_PROTOCOL_TMAX_NM        (10.0f)

/* 三台电机的回帧在线阈值。时间戳只在 CanTask 消费 ISR 事件时更新，
 * GimbalTask 不再自行维护电机时间戳。 */
#define CAN_MOTOR_LEFT_FEEDBACK_TIMEOUT_MS  (1000U)
#define CAN_MOTOR_RIGHT_FEEDBACK_TIMEOUT_MS (1000U)
#define CAN_MOTOR_PITCH_FEEDBACK_TIMEOUT_MS (1000U)

/* Pitch 电机一帧完整反馈的解析结果。Pitch 姿态闭环仍以 IMU 为准，
 * 故这些量暂不直接进入 PID；保留它们用于电机状态、调试和后续诊断。 */
typedef struct {
  uint8_t motor_id;       /* 反馈 data[0] 低四位。 */
  uint8_t driver_status;  /* 反馈 data[0] 高四位；0=失能，1=使能。 */
  uint16_t position_raw;
  uint16_t velocity_raw;
  uint16_t torque_raw;
  float position_rad;
  float velocity_rad_s;
  float torque_nm;
  float mos_temperature_c;
  float coil_temperature_c;
  uint32_t timestamp_ms;  /* CanTask 完整解析此帧的本地时刻。 */
  uint32_t frame_count;
} DM_Motor_Feedback_t;

/* ISR -> CanTask 的三电机原始反馈快照。
 * 每次入队均携带三个电机最近的原始帧、DLC 和序号；CanTask 根据 sequence
 * 判断哪台有新帧，并在任务上下文完成全部有效性校验。 */
typedef struct {
  uint8_t data[3][8];
  uint8_t length[3];
  uint32_t sequence[3];
} Can_Motor_Rx_Snapshot_t;

/* 仅供 DebugTask 查看的 Pitch 链路诊断，不能参与控制决策。 */
typedef struct {
  uint32_t pitch_enable_attempts;
  uint32_t pitch_enable_enqueue_failures;
  uint32_t pitch_safe_zero_count;
  uint32_t pitch_recovery_zero_count;
  uint8_t last_pitch_zero_source;
  uint32_t pitch_invalid_feedback_count;
  uint8_t last_pitch_feedback_length;
  uint8_t last_pitch_feedback_motor_id;
  uint32_t left_invalid_feedback_count;
  uint32_t right_invalid_feedback_count;
  uint8_t last_left_feedback_length;
  uint8_t last_left_feedback_motor_id;
  uint8_t last_right_feedback_length;
  uint8_t last_right_feedback_motor_id;
  uint32_t left_feedback_count;
  uint32_t right_feedback_count;
  uint32_t pitch_feedback_count;
  uint8_t left_driver_status;
  uint8_t right_driver_status;
  uint8_t pitch_driver_status;
} Can_Motor_Diagnostic_t;

/* CanTask 发给 Pitch 的零 MIT 来源；GimbalTask 的故障位另行打印。 */
typedef enum {
  CAN_MOTOR_PITCH_ZERO_NONE = 0U,
  CAN_MOTOR_PITCH_ZERO_INITIAL_START,
  CAN_MOTOR_PITCH_ZERO_BUS_OFF_RECOVERY,
  CAN_MOTOR_PITCH_ZERO_ENABLE_RECOVERY
} Can_Motor_Pitch_Zero_Source_t;

typedef struct {
  float pmax_rad;
  float vmax_rad_s;
  float tmax_nm;
  float kp_min;
  float kp_max;
  float kd_min;
  float kd_max;
} DM_Motor_PhysicalConfig_t;

typedef struct {
  uint16_t id;          /* ESC_ID，控制帧基础 ID */
  uint16_t mode;
  FDCAN_HandleTypeDef *hcan;
  DM_Motor_PhysicalConfig_t config;
} DM_Motor_t;

typedef enum {
  CAN_MOTOR_LINK_INIT = 0U,
  /* CanTask 正在按启动节拍依次发送使能和零输出帧；GimbalTask 不得插帧。 */
  CAN_MOTOR_LINK_INITIALIZING,
  /* FDCAN 已启动，已发送使能和零指令，等待 Pitch 的 0x13 反馈确认链路。 */
  CAN_MOTOR_LINK_WAIT_PITCH_FEEDBACK,
  CAN_MOTOR_LINK_ONLINE
} Can_Motor_Link_State_t;

typedef enum {
  DM_MOTOR_LEFT = 0U,
  DM_MOTOR_RIGHT,
  DM_MOTOR_PITCH
} DM_Motor_Index_t;

typedef enum {
  CAN_MOTOR_HEALTH_NONE               = 0U,
  CAN_MOTOR_HEALTH_LEFT_TIMEOUT       = 1U << 0,
  CAN_MOTOR_HEALTH_RIGHT_TIMEOUT      = 1U << 1,
  CAN_MOTOR_HEALTH_PITCH_TIMEOUT      = 1U << 2,
  CAN_MOTOR_HEALTH_LEFT_DRIVER_FAULT  = 1U << 3,
  CAN_MOTOR_HEALTH_RIGHT_DRIVER_FAULT = 1U << 4,
  CAN_MOTOR_HEALTH_PITCH_DRIVER_FAULT = 1U << 5,
  CAN_MOTOR_HEALTH_BUS_OFF            = 1U << 6,
  CAN_MOTOR_HEALTH_LEFT_DATA_FAULT    = 1U << 7,
  CAN_MOTOR_HEALTH_RIGHT_DATA_FAULT   = 1U << 8,
  CAN_MOTOR_HEALTH_PITCH_DATA_FAULT   = 1U << 9
} Can_Motor_Health_Flag_t;

void Can_Motor_Init(void);
void Can_Motor_Process(void);
/** @brief 仅由 FDCAN Bus-Off ISR 调用：请求 CanTask 最小化 Stop/Start 恢复。 */
void Can_Motor_RequestBusOffRecovery(FDCAN_HandleTypeDef *hfdcan);
Can_Motor_Link_State_t Can_Motor_GetLinkState(void);
/** CanTask 维护的三电机在线/驱动器/总线健康状态；GimbalTask 仅读取此状态。 */
uint32_t Can_Motor_GetHealthFlags(void);
/* 仅当 Pitch 已收到有效 s=1 回帧且无自身通信/驱动故障时返回 1。
 * GimbalTask 必须以此作为 PID 扭矩输出的唯一准入条件。 */
uint8_t Can_Motor_IsPitchCommandReady(void);
/** 读取 CanTask 保存的 Pitch 完整反馈快照。仅任务上下文可调用，不可在 ISR 调用。 */
uint8_t Can_Motor_GetPitchFeedback(DM_Motor_Feedback_t *feedback);
void Can_Motor_GetDiagnostic(Can_Motor_Diagnostic_t *diagnostic);
/** FDCAN ISR 入口：只收集三台原始帧并投递最新快照；不做有效性业务判断。 */
void Can_Motor_OnFeedbackFrame(DM_Motor_Index_t index, const uint8_t data[8],
                               uint8_t data_length);
/** Pitch 专用控制接口，保证内部电机对象不暴露到模块外。 */
uint8_t Can_Motor_PitchMIT(float kp, float kd, float position_rad,
                           float velocity_rad_s, float torque_nm);
float Can_Motor_PitchTorqueLimit(void);
/** 两个摩擦轮的固定转速控制接口；不使用数值反馈，但使用 CanTask 在线状态。 */
uint8_t Can_Motor_FrictionSpeed(float left_rad_s, float right_rad_s);
uint8_t Can_Motor_LeftFrictionSpeed(float velocity_rad_s);
uint8_t Can_Motor_RightFrictionSpeed(float velocity_rad_s);
/* ISR -> CanTask 的三电机反馈快照队列，长度由 CAN_MOTOR_RX_QUEUE_DEPTH 定义。 */
extern osMessageQueueId_t Can_Motor_Rx_QueueHandle;
void DM_Motor_Enable(DM_Motor_t *motor);
void DM_Motor_Disable(DM_Motor_t *motor);
uint8_t DM_Motor_Speed_Control(DM_Motor_t *motor, float velocity_rad_s);
uint8_t DM_Motor_MIT_Control(DM_Motor_t *motor, float kp, float kd,
                             float position_rad, float velocity_rad_s,
                             float torque_nm);
#endif
