#include "Can_Motor.h"
#include "Can_Filter.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static DM_Motor_t s_motor_left;
static DM_Motor_t s_motor_right;
static DM_Motor_t s_motor_pitch;

/* 将正常浮点指令夹到对称绝对值上限；NaN 由调用点单独拒绝。 */
static float clamp_symmetric(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static DM_Motor_t *motor_from_index(DM_Motor_Index_t index)
{
  switch (index) {
  case DM_MOTOR_LEFT: return &s_motor_left;
  case DM_MOTOR_RIGHT: return &s_motor_right;
  case DM_MOTOR_PITCH: return &s_motor_pitch;
  default: return NULL;
  }
}

static uint8_t can_motor_is_present(DM_Motor_Index_t index)
{
  switch (index) {
  case DM_MOTOR_LEFT: return CAN_MOTOR_LEFT_PRESENT;
  case DM_MOTOR_RIGHT: return CAN_MOTOR_RIGHT_PRESENT;
  case DM_MOTOR_PITCH: return 1U;
  default: return 0U;
  }
}

static uint8_t float_to_uint(float value, float min, float max,
                             uint8_t bits, uint16_t *result)
{
  const float span = max - min;
  if (result == NULL || value != value || value < min || value > max || span <= 0.0f)
    return 1U;
  *result = (uint16_t)((value - min) * ((float)((1UL << bits) - 1UL)) / span);
  return 0U;
}

typedef enum {
  CAN_MOTOR_STATE_WAIT_FEEDBACK = 0U,
  CAN_MOTOR_STATE_ONLINE,
  /* 已收到状态=0 的有效反馈，但驱动尚未使能。 */
  CAN_MOTOR_STATE_DISABLED,
  CAN_MOTOR_STATE_TIMEOUT,
  CAN_MOTOR_STATE_DRIVER_FAULT
} Can_Motor_State_t;

typedef struct {
  uint32_t last_rx_tick_ms;
  uint32_t frame_count;
  uint8_t last_driver_status;
  Can_Motor_State_t state;
} Can_Motor_Runtime_t;

/* ISR 仅置位/入队；CanTask 是三电机在线状态机的唯一写入者。 */
static volatile uint8_t s_fdcan1_bus_off_pending;
/* FDCAN ISR 只置位；CanTask 原子读取并清除。bit0/1/2 对应左/右/Pitch。 */
static Can_Motor_Rx_Snapshot_t s_isr_rx_snapshot;
static uint32_t s_task_seen_sequence[3];
static volatile Can_Motor_Link_State_t s_can_motor_link_state = CAN_MOTOR_LINK_INIT;
static volatile uint32_t s_can_motor_health_flags;
static Can_Motor_Runtime_t s_motor_runtime[3];
static volatile Can_Motor_Diagnostic_t s_can_motor_diagnostic;
/* 只有 Pitch 保存完整数值反馈；左右摩擦轮只维护收到帧/状态/超时。 */
static DM_Motor_Feedback_t s_pitch_feedback;
static int8_t s_recovery_motor_index = -1;
/* 下一次超时恢复从哪台开始查找。即使某台始终掉线，也不能占住其他电机的
 * 使能/零指令恢复机会；每成功选中一台便推进到其后一台。 */
static uint8_t s_recovery_next_motor_index;
static uint8_t s_recovery_send_zero;
static uint32_t s_recovery_last_tx_tick;
/* 任一电机自身错误或格式错误后，必须先由该电机有效 s=0 回帧确认 Disable
 * 生效，才允许清错并进入 Enable -> 零指令恢复。三台互不影响。 */
static uint8_t s_disable_pending[3];
static uint8_t s_disable_next_motor_index;
static uint32_t s_disable_last_tx_tick;

#if (CAN_MOTOR_COMMUNICATION_ENABLE != 0U)
/* 不改变过滤器、FIFO 和时序，只打开已经配置好的中断。 */
static HAL_StatusTypeDef can_motor_enable_notifications(void)
{
  return HAL_FDCAN_ActivateNotification(&hfdcan1,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
      FDCAN_IT_BUS_OFF,
      0U);
}

/* 达妙反馈首字节高四位是“状态”而不是“非零即故障”。
 * 说明书定义 0=失能、1=使能；未定义的状态码也按故障处理，避免未知状态下继续运动。 */
static uint8_t dm_motor_status_is_fault(uint8_t status)
{
  switch (status) {
  case 0x0U:
  case 0x1U:
    return 0U;
  case 0x5U: /* 读出传感器错误 */
  case 0x6U: /* 读取电机参数错误 */
  case 0x8U: /* 超压 */
  case 0x9U: /* 欠压 */
  case 0xAU: /* 过电流 */
  case 0xBU: /* MOS 过温 */
  case 0xCU: /* 线圈过温 */
  case 0xDU: /* 通讯丢失 */
  case 0xEU: /* 过载 */
  default:
    return 1U;
  }
}

static uint32_t can_motor_timeout_flag(DM_Motor_Index_t index)
{
  return 1UL << (uint32_t)index;
}

static uint32_t can_motor_driver_fault_flag(DM_Motor_Index_t index)
{
  return 1UL << ((uint32_t)index + 3U);
}

static uint32_t can_motor_data_fault_flag(DM_Motor_Index_t index)
{
  switch (index) {
  case DM_MOTOR_LEFT: return CAN_MOTOR_HEALTH_LEFT_DATA_FAULT;
  case DM_MOTOR_RIGHT: return CAN_MOTOR_HEALTH_RIGHT_DATA_FAULT;
  case DM_MOTOR_PITCH: return CAN_MOTOR_HEALTH_PITCH_DATA_FAULT;
  default: return CAN_MOTOR_HEALTH_NONE;
  }
}

static uint32_t can_motor_timeout_ms(DM_Motor_Index_t index)
{
  switch (index) {
  case DM_MOTOR_LEFT: return CAN_MOTOR_LEFT_FEEDBACK_TIMEOUT_MS;
  case DM_MOTOR_RIGHT: return CAN_MOTOR_RIGHT_FEEDBACK_TIMEOUT_MS;
  case DM_MOTOR_PITCH: return CAN_MOTOR_PITCH_FEEDBACK_TIMEOUT_MS;
  default: return 0U;
  }
}

/* 仅开机调用一次：配置过滤器并启动 FDCAN1。 */
static HAL_StatusTypeDef can_motor_initial_start_fdcan1(void)
{
  Can_Filter_Init();
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) return HAL_ERROR;
  if (can_motor_enable_notifications() != HAL_OK) return HAL_ERROR;
  return HAL_OK;
}

/* 启动和 CAN Bus-Off 恢复后只发送零目标，不自动使能。若电机在 MCU 复位前
 * 已保持使能，此帧仍将其置入安全零输出。自动使能由下方超时恢复状态机受宏控制。 */
static void can_motor_send_safe_zero_output(Can_Motor_Pitch_Zero_Source_t source)
{
#if (CAN_MOTOR_LEFT_PRESENT != 0U)
  (void)DM_Motor_Speed_Control(&s_motor_left, 0.0f);
#endif
#if (CAN_MOTOR_RIGHT_PRESENT != 0U)
  (void)DM_Motor_Speed_Control(&s_motor_right, 0.0f);
#endif
  (void)Can_Motor_PitchMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  s_can_motor_diagnostic.pitch_safe_zero_count++;
  s_can_motor_diagnostic.last_pitch_zero_source = (uint8_t)source;
}

#if (CAN_MOTOR_TIMEOUT_AUTO_REENABLE != 0U)
static void can_motor_send_zero_for_index(DM_Motor_Index_t index)
{
  switch (index) {
  case DM_MOTOR_LEFT:
    (void)DM_Motor_Speed_Control(&s_motor_left, 0.0f);
    break;
  case DM_MOTOR_RIGHT:
    (void)DM_Motor_Speed_Control(&s_motor_right, 0.0f);
    break;
  case DM_MOTOR_PITCH:
    (void)Can_Motor_PitchMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    s_can_motor_diagnostic.pitch_recovery_zero_count++;
    s_can_motor_diagnostic.last_pitch_zero_source =
        (uint8_t)CAN_MOTOR_PITCH_ZERO_ENABLE_RECOVERY;
    break;
  default:
    break;
  }
}

/* 超时或任一电机明确回报“失能”时，均进入同一受控使能流程。
 * 驱动/格式错误必须先经 Disable -> s=0 确认 -> 清错，之后才允许恢复。 */
static uint8_t can_motor_needs_enable_recovery(DM_Motor_Index_t index)
{
  const uint32_t timeout_flag = can_motor_timeout_flag(index);
  const uint32_t driver_flag = can_motor_driver_fault_flag(index);
  const uint32_t data_flag = can_motor_data_fault_flag(index);
  if (can_motor_is_present(index) == 0U) return 0U;
  /* 每台电机只等待自己的 Disable 确认；其他两台仍可独立恢复。 */
  if (s_disable_pending[index] != 0U) return 0U;
  if ((s_can_motor_health_flags & driver_flag) != 0U) return 0U;
  if ((s_can_motor_health_flags & data_flag) != 0U) return 0U;
  if ((s_can_motor_health_flags & timeout_flag) != 0U) return 1U;
  return (s_motor_runtime[index].state == CAN_MOTOR_STATE_DISABLED) ? 1U : 0U;
}
#endif

/* 仅在 CAN_MOTOR_TIMEOUT_AUTO_REENABLE=1 时运行。一次只处理一台电机：
 * 相邻使能/零指令至少间隔 3 ms，三台电机仍一帧一帧轮询，绝不在同一拍堆帧。 */
static void can_motor_process_timeout_recovery(uint32_t now)
{
#if (CAN_MOTOR_TIMEOUT_AUTO_REENABLE != 0U)
  if ((uint32_t)(now - s_recovery_last_tx_tick) <
      CAN_MOTOR_INITIAL_FRAME_INTERVAL_MS) return;

  if (s_recovery_motor_index < 0) {
    /* 轮询三台电机，不按固定左/右/Pitch 优先级反复选第一台。 */
    for (uint8_t offset = 0U; offset < 3U; ++offset) {
      const uint8_t candidate = (uint8_t)((s_recovery_next_motor_index + offset) % 3U);
      if (can_motor_needs_enable_recovery((DM_Motor_Index_t)candidate) != 0U) {
        s_recovery_motor_index = (int8_t)candidate;
        s_recovery_next_motor_index = (uint8_t)((candidate + 1U) % 3U);
        s_recovery_send_zero = 0U;
        break;
      }
    }
    if (s_recovery_motor_index < 0) return;
  }

  const DM_Motor_Index_t index = (DM_Motor_Index_t)s_recovery_motor_index;
  if (can_motor_needs_enable_recovery(index) == 0U) {
    s_recovery_motor_index = -1;
    return;
  }
  if (s_recovery_send_zero == 0U) {
    DM_Motor_Enable(motor_from_index(index));
    s_recovery_send_zero = 1U;
  } else {
    can_motor_send_zero_for_index(index);
    s_recovery_send_zero = 0U;
    s_recovery_motor_index = -1;
  }
  s_recovery_last_tx_tick = now;
#else
  (void)now;
#endif
}
#endif

static void can_motor_process_disable(uint32_t now)
{
  if ((uint32_t)(now - s_disable_last_tx_tick) <
      CAN_MOTOR_DISABLE_RETRY_INTERVAL_MS) return;
  for (uint8_t offset = 0U; offset < 3U; ++offset) {
    const uint8_t candidate = (uint8_t)((s_disable_next_motor_index + offset) % 3U);
    if (s_disable_pending[candidate] == 0U) continue;
    DM_Motor_Disable(motor_from_index((DM_Motor_Index_t)candidate));
    s_disable_next_motor_index = (uint8_t)((candidate + 1U) % 3U);
    s_disable_last_tx_tick = now;
    return;
  }
}

static void can_motor_request_disable(DM_Motor_Index_t index, uint32_t now)
{
  if (index > DM_MOTOR_PITCH) return;
  if (s_disable_pending[index] == 0U) {
    s_disable_pending[index] = 1U;
    /* 首帧可在本次 CanTask 发送；随后所有 Disable 统一以 3 ms 节拍轮询。 */
    s_disable_last_tx_tick = now - CAN_MOTOR_DISABLE_RETRY_INTERVAL_MS;
  }
}

/**
 * @brief 根据电机基础 ID 和模式偏移计算实际控制帧 ID。
 * @param motor 电机对象，使用其中的基础 ESC ID。
 * @param mode  模式偏移：MIT/位置速度/速度分别为 0x000/0x100/0x200。
 * @return 11 位标准 CAN 控制帧 ID。
 */
static uint16_t control_id(const DM_Motor_t *motor, uint16_t mode)
{
  return (uint16_t)(motor->id + mode);
}

/**
 * @brief 使用当前工程的 FDCAN1 发送一帧标准数据帧。
 * @param hcan   FDCAN 外设句柄。
 * @param id     11 位标准 CAN ID。
 * @param data   数据缓冲区。
 * @param length 数据长度；本协议控制帧为 4 或 8 字节。
 * @return 0 表示发送成功，1 表示发送队列加入失败。
 */
static uint8_t dm_send(FDCAN_HandleTypeDef *hcan, uint16_t id,
                       const uint8_t *data, uint32_t length)
{
#if (CAN_MOTOR_COMMUNICATION_ENABLE == 0U)
  /* 台架测试时禁止一切 CAN 电机发送；即使云台安全路径反复请求零输出，
   * 也绝不触碰未启动的 FDCAN 外设。 */
  (void)hcan;
  (void)id;
  (void)data;
  (void)length;
  return 1U;
#else
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = (length <= 8U) ? length : FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  /* 达妙电机与上位机均使用经典 CAN 标准帧，仲裁速率为 1 Mbps。
   * 即使负载只有 8 字节，也不得发送 CAN-FD/BRS 帧，否则电机不会确认。 */
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;
  return (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, (uint8_t *)data) == HAL_OK) ? 0U : 1U;
#endif
}

/**
 * @brief 初始化三个达妙电机对象，并启动 FDCAN1 接收。
 * @details 设置左右摩擦轮和 Pitch 的 ESC_ID、MST_ID、控制模式及映射范围，
 *          随后配置过滤器、启动 FDCAN1，并打开 FIFO0/FIFO1 新消息中断。
 */
void Can_Motor_Init(void)
{
  memset(&s_motor_left, 0, sizeof(s_motor_left));
  memset(&s_motor_right, 0, sizeof(s_motor_right));
  memset(&s_motor_pitch, 0, sizeof(s_motor_pitch));

  s_motor_left.id = DM_MOTOR_LEFT_ID;
  s_motor_left.mode = DM_MODE_SPEED;
  s_motor_right.id = DM_MOTOR_RIGHT_ID;
  s_motor_right.mode = DM_MODE_SPEED;
  s_motor_pitch.id = DM_MOTOR_PITCH_ID;
  s_motor_pitch.mode = DM_MODE_MIT;
  s_motor_left.hcan = &hfdcan1;
  s_motor_right.hcan = &hfdcan1;
  s_motor_pitch.hcan = &hfdcan1;

  /* 三台均为 DM-S3519。此处是 CAN 协议编码范围，运行安全限幅在各发送
   * 函数内使用 DM_S3519_PEAK_TORQUE_NM / MAX_COMMAND_SPEED 宏执行。 */
  s_motor_left.config = (DM_Motor_PhysicalConfig_t){
      DM_S3519_PROTOCOL_PMAX_RAD,
      DM_S3519_PROTOCOL_VMAX_RAD_S,
      DM_S3519_PROTOCOL_TMAX_NM,
      0.0f, 500.0f, 0.0f, 5.0f};
  s_motor_right.config = s_motor_left.config;
  s_motor_pitch.config = s_motor_left.config;

  s_fdcan1_bus_off_pending = 0U;
  memset(&s_isr_rx_snapshot, 0, sizeof(s_isr_rx_snapshot));
  memset(s_task_seen_sequence, 0, sizeof(s_task_seen_sequence));
  memset(s_motor_runtime, 0, sizeof(s_motor_runtime));
  memset((void *)&s_can_motor_diagnostic, 0, sizeof(s_can_motor_diagnostic));
  memset(&s_pitch_feedback, 0, sizeof(s_pitch_feedback));
  for (uint8_t i = 0U; i < 3U; ++i)
    s_motor_runtime[i].state = CAN_MOTOR_STATE_WAIT_FEEDBACK;
  /* 只对实际安装的电机建立反馈超时；未装的摩擦轮不产生保护或恢复流量。 */
  s_can_motor_health_flags = CAN_MOTOR_HEALTH_PITCH_TIMEOUT;
#if (CAN_MOTOR_LEFT_PRESENT != 0U)
  s_can_motor_health_flags |= CAN_MOTOR_HEALTH_LEFT_TIMEOUT;
#endif
#if (CAN_MOTOR_RIGHT_PRESENT != 0U)
  s_can_motor_health_flags |= CAN_MOTOR_HEALTH_RIGHT_TIMEOUT;
#endif
  s_recovery_motor_index = -1;
  s_recovery_next_motor_index = 0U;
  s_recovery_send_zero = 0U;
  s_recovery_last_tx_tick = HAL_GetTick();
  memset(s_disable_pending, 0, sizeof(s_disable_pending));
  s_disable_next_motor_index = 0U;
  s_disable_last_tx_tick = HAL_GetTick();
#if (CAN_MOTOR_COMMUNICATION_ENABLE != 0U)
  if (can_motor_initial_start_fdcan1() != HAL_OK) {
    s_can_motor_link_state = CAN_MOTOR_LINK_INIT;
  } else {
    s_can_motor_link_state = CAN_MOTOR_LINK_INITIALIZING;
    can_motor_send_safe_zero_output(CAN_MOTOR_PITCH_ZERO_INITIAL_START);
    /* 默认不自动使能。收到 Pitch 0x13 事件前，链路维持等待状态；三台的在线
     * 状态由 Can_Motor_Process 中各自的时间戳判断。 */
    s_can_motor_link_state = CAN_MOTOR_LINK_WAIT_PITCH_FEEDBACK;
  }
#else
  /* 只测遥控等非电机功能时，FDCAN1 保持未启动状态。这样没有接电机也不会因
   * 无 ACK 进入错误中断和反复恢复。GimbalTask 会据此判定 CAN 离线并停机。 */
  s_can_motor_link_state = CAN_MOTOR_LINK_INIT;
#endif
}

/**
 * @brief 按电机当前模式发送使能命令。
 * @param motor 要使能的电机对象。
 */
void DM_Motor_Enable(DM_Motor_t *motor)
{
  static const uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
  const uint8_t status = dm_send(motor->hcan, control_id(motor, motor->mode), data, 8U);
  if (motor == &s_motor_pitch) {
    s_can_motor_diagnostic.pitch_enable_attempts++;
    if (status != 0U) s_can_motor_diagnostic.pitch_enable_enqueue_failures++;
  }
}

/**
 * @brief 发送速度模式控制帧。
 * @param motor          目标电机对象。
 * @param velocity_rad_s 输出轴目标速度，单位 rad/s。
 * @return 0 表示发送成功，1 表示发送失败。
 */
uint8_t DM_Motor_Speed_Control(DM_Motor_t *motor, float velocity_rad_s)
{
  if (motor == NULL || velocity_rad_s != velocity_rad_s) return 1U;
  velocity_rad_s = clamp_symmetric(velocity_rad_s,
                                   DM_S3519_MAX_COMMAND_SPEED_RAD_S);
  return dm_send(motor->hcan, control_id(motor, DM_MODE_SPEED),
                 (uint8_t *)&velocity_rad_s, 4U);
}

void DM_Motor_Disable(DM_Motor_t *motor)
{
  static const uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFDU};
  if (motor != NULL) (void)dm_send(motor->hcan, control_id(motor, motor->mode), data, 8U);
}

/**
 * @brief 发送 MIT 模式控制帧。
 * @param motor          目标电机对象，提供 PMAX/VMAX/TMAX 映射上限。
 * @param kp             位置比例增益，范围 [0, 500]。
 * @param kd             位置微分增益，范围 [0, 5]。
 * @param position_rad   目标位置，单位 rad。
 * @param velocity_rad_s 目标速度，单位 rad/s。
 * @param torque_nm      转矩前馈，单位 Nm。
 * @return 0 表示发送成功，1 表示发送失败。
 */
uint8_t DM_Motor_MIT_Control(DM_Motor_t *motor, float kp, float kd,
                             float position_rad, float velocity_rad_s,
                             float torque_nm)
{
  uint8_t data[8];
  uint16_t p, v, t, kp_u, kd_u;
  if (motor == NULL || velocity_rad_s != velocity_rad_s || torque_nm != torque_nm ||
      position_rad != position_rad || kp != kp || kd != kd ||
      position_rad < -motor->config.pmax_rad || position_rad > motor->config.pmax_rad ||
      kp < motor->config.kp_min || kp > motor->config.kp_max ||
      kd < motor->config.kd_min || kd > motor->config.kd_max)
    return 1U;
  velocity_rad_s = clamp_symmetric(velocity_rad_s,
                                   DM_S3519_MAX_COMMAND_SPEED_RAD_S);
  torque_nm = clamp_symmetric(torque_nm, DM_S3519_PEAK_TORQUE_NM);
  if (
      float_to_uint(position_rad, -motor->config.pmax_rad, motor->config.pmax_rad, 16U, &p) ||
      float_to_uint(velocity_rad_s, -motor->config.vmax_rad_s, motor->config.vmax_rad_s, 12U, &v) ||
      float_to_uint(torque_nm, -motor->config.tmax_nm, motor->config.tmax_nm, 12U, &t) ||
      float_to_uint(kp, motor->config.kp_min, motor->config.kp_max, 12U, &kp_u) ||
      float_to_uint(kd, motor->config.kd_min, motor->config.kd_max, 12U, &kd_u)) return 1U;
  data[0] = (uint8_t)(p >> 8); data[1] = (uint8_t)p;
  data[2] = (uint8_t)(v >> 4);
  data[3] = (uint8_t)((v & 0x0FU) << 4) | (uint8_t)(kp_u >> 8);
  data[4] = (uint8_t)kp_u;
  data[5] = (uint8_t)(kd_u >> 4);
  data[6] = (uint8_t)((kd_u & 0x0FU) << 4) | (uint8_t)(t >> 8);
  data[7] = (uint8_t)t;
  return dm_send(motor->hcan, control_id(motor, DM_MODE_MIT), data, 8U);
}

/* 达妙反馈编码与 MIT 指令使用同一组 PMAX/VMAX/TMAX 映射范围。
 * 该函数只在 CanTask 调用，绝不进入 FDCAN 中断。 */
static float uint_to_float(uint16_t value, float min, float max, uint16_t max_raw)
{
  return ((float)value * (max - min) / (float)max_raw) + min;
}

/* 完整解包一帧达妙反馈。反馈布局：
 * data[0]=状态[7:4] + 电机 ID[3:0]；data[1..2]=位置 16 位；
 * data[3..4]=速度 12 位；data[4..5]=扭矩 12 位；data[6]=MOS 温度；
 * data[7]=线圈温度。 */
static void can_motor_decode_pitch_feedback(const uint8_t data[8],
                                            uint32_t task_tick_ms)
{
  const DM_Motor_PhysicalConfig_t *config = &s_motor_pitch.config;
  DM_Motor_Feedback_t *feedback = &s_pitch_feedback;
  const uint16_t position_raw = ((uint16_t)data[1] << 8) | data[2];
  const uint16_t velocity_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
  const uint16_t torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];

  feedback->motor_id = data[0] & 0x0FU;
  feedback->driver_status = data[0] >> 4;
  feedback->position_raw = position_raw;
  feedback->velocity_raw = velocity_raw;
  feedback->torque_raw = torque_raw;
  feedback->position_rad = uint_to_float(position_raw,
                                          -config->pmax_rad,
                                          config->pmax_rad,
                                          65535U);
  feedback->velocity_rad_s = uint_to_float(velocity_raw,
                                            -config->vmax_rad_s,
                                            config->vmax_rad_s,
                                            4095U);
  feedback->torque_nm = uint_to_float(torque_raw,
                                      -config->tmax_nm,
                                      config->tmax_nm,
                                      4095U);
  feedback->mos_temperature_c = (float)data[6];
  feedback->coil_temperature_c = (float)data[7];
  /* 统一由 CanTask 在消费事件时读取 HAL 时基；CAN ISR 不读取时间戳。 */
  feedback->timestamp_ms = task_tick_ms;
  feedback->frame_count++;
}

/** CanTask 消费一条 ISR 反馈事件；这是三台电机时间戳和驱动器状态的唯一写入点。 */
static void can_motor_consume_rx_event(DM_Motor_Index_t index, uint8_t driver_status,
                                       uint32_t task_tick_ms)
{
  if (index > DM_MOTOR_PITCH) return;
  Can_Motor_Runtime_t *runtime = &s_motor_runtime[index];
  const uint32_t timeout_flag = can_motor_timeout_flag(index);
  const uint32_t driver_flag = can_motor_driver_fault_flag(index);

  runtime->last_rx_tick_ms = task_tick_ms;
  runtime->frame_count++;
  runtime->last_driver_status = driver_status;
  s_can_motor_health_flags &= ~timeout_flag;
  if (index == DM_MOTOR_LEFT) {
    s_can_motor_diagnostic.left_feedback_count++;
    s_can_motor_diagnostic.left_driver_status = driver_status;
  } else if (index == DM_MOTOR_RIGHT) {
    s_can_motor_diagnostic.right_feedback_count++;
    s_can_motor_diagnostic.right_driver_status = driver_status;
  } else {
    s_can_motor_diagnostic.pitch_feedback_count++;
    s_can_motor_diagnostic.pitch_driver_status = driver_status;
  }

  /* 仅本电机的有效 s=0 可以确认其 Disable 已生效。确认后清本机的驱动/格式
   * 故障，下一轮恢复状态机才会发送 Enable；不能由其他电机的反馈越权清错。 */
  if (driver_status == 0U && s_disable_pending[index] != 0U) {
    s_disable_pending[index] = 0U;
    s_can_motor_health_flags &= ~(driver_flag | can_motor_data_fault_flag(index));
  }
  if (dm_motor_status_is_fault(driver_status) != 0U) {
    s_can_motor_health_flags |= driver_flag;
    runtime->state = CAN_MOTOR_STATE_DRIVER_FAULT;
    /* 三台均只由自身错误进入 Disable 等待；其他电机和输入模块不参与。 */
    can_motor_request_disable(index, task_tick_ms);
  } else if ((s_can_motor_health_flags & driver_flag) == 0U) {
    runtime->state = (driver_status == 0U) ? CAN_MOTOR_STATE_DISABLED :
                                             CAN_MOTOR_STATE_ONLINE;
  }
}

/** CanTask 统一检查三台电机回帧时间；未收到首帧也从上电起视为超时。 */
static void can_motor_update_timeouts(uint32_t now)
{
  for (uint8_t i = 0U; i < 3U; ++i) {
    const DM_Motor_Index_t index = (DM_Motor_Index_t)i;
    Can_Motor_Runtime_t *runtime = &s_motor_runtime[i];
    const uint32_t timeout_flag = can_motor_timeout_flag(index);
    const uint32_t driver_flag = can_motor_driver_fault_flag(index);
    const uint32_t data_flag = can_motor_data_fault_flag(index);
    if (can_motor_is_present(index) == 0U) {
      s_can_motor_health_flags &= ~(timeout_flag | driver_flag | data_flag);
      continue;
    }
    const uint32_t timeout_ms = can_motor_timeout_ms(index);
    const uint8_t timed_out = (runtime->frame_count == 0U) ||
        ((uint32_t)(now - runtime->last_rx_tick_ms) > timeout_ms);

    if (timed_out != 0U) {
      s_can_motor_health_flags |= timeout_flag;
      if ((s_can_motor_health_flags & driver_flag) == 0U)
        runtime->state = CAN_MOTOR_STATE_TIMEOUT;
    } else {
      s_can_motor_health_flags &= ~timeout_flag;
      if ((s_can_motor_health_flags & driver_flag) == 0U &&
          runtime->state != CAN_MOTOR_STATE_DISABLED)
        runtime->state = CAN_MOTOR_STATE_ONLINE;
    }
  }
}

/** @brief CAN 电机任务执行入口：消费反馈事件、统一判超时，并进行受控恢复。 */
void Can_Motor_Process(void)
{
#if (CAN_MOTOR_COMMUNICATION_ENABLE != 0U)
  const uint32_t now = HAL_GetTick();
  Can_Motor_Rx_Snapshot_t snapshot;
  if (Can_Motor_Rx_QueueHandle != NULL &&
      osMessageQueueGet(Can_Motor_Rx_QueueHandle, &snapshot, NULL, 0U) == osOK) {
    static const uint8_t expected_id[3] = {DM_MOTOR_LEFT_ID, DM_MOTOR_RIGHT_ID, DM_MOTOR_PITCH_ID};
    for (uint8_t i = 0U; i < 3U; ++i) {
      if (snapshot.sequence[i] == s_task_seen_sequence[i]) continue;
      s_task_seen_sequence[i] = snapshot.sequence[i];
      if (snapshot.length[i] != 8U || (snapshot.data[i][0] & 0x0FU) != expected_id[i]) {
        if (i == DM_MOTOR_LEFT) {
          s_can_motor_diagnostic.left_invalid_feedback_count++;
          s_can_motor_diagnostic.last_left_feedback_length = snapshot.length[i];
          s_can_motor_diagnostic.last_left_feedback_motor_id = snapshot.data[i][0] & 0x0FU;
        } else if (i == DM_MOTOR_RIGHT) {
          s_can_motor_diagnostic.right_invalid_feedback_count++;
          s_can_motor_diagnostic.last_right_feedback_length = snapshot.length[i];
          s_can_motor_diagnostic.last_right_feedback_motor_id = snapshot.data[i][0] & 0x0FU;
        } else {
          s_can_motor_diagnostic.pitch_invalid_feedback_count++;
          s_can_motor_diagnostic.last_pitch_feedback_length = snapshot.length[i];
          s_can_motor_diagnostic.last_pitch_feedback_motor_id = snapshot.data[i][0] & 0x0FU;
        }
        /* 帧长或帧内电机 ID 不可信：只停本机，重发 Disable 至收到本机有效 s=0。 */
        s_can_motor_health_flags |= can_motor_data_fault_flag((DM_Motor_Index_t)i);
        can_motor_request_disable((DM_Motor_Index_t)i, now);
        continue;
      }
      can_motor_consume_rx_event((DM_Motor_Index_t)i, snapshot.data[i][0] >> 4, now);
      if (i == DM_MOTOR_PITCH) can_motor_decode_pitch_feedback(snapshot.data[i], now);
    }
  }

  /* Bus-Off 才 Stop/Start；不重配过滤器，也不重新调用 MX_FDCAN1_Init。 */
  if (s_fdcan1_bus_off_pending != 0U) {
    s_can_motor_health_flags |= CAN_MOTOR_HEALTH_BUS_OFF;
    s_can_motor_link_state = CAN_MOTOR_LINK_INIT;
    (void)HAL_FDCAN_Stop(&hfdcan1);
    if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK &&
        can_motor_enable_notifications() == HAL_OK) {
      s_fdcan1_bus_off_pending = 0U;
      s_can_motor_health_flags &= ~CAN_MOTOR_HEALTH_BUS_OFF;
      s_can_motor_link_state = CAN_MOTOR_LINK_INITIALIZING;
      can_motor_send_safe_zero_output(CAN_MOTOR_PITCH_ZERO_BUS_OFF_RECOVERY);
      s_can_motor_link_state = CAN_MOTOR_LINK_WAIT_PITCH_FEEDBACK;
    }
  }

  can_motor_update_timeouts(now);
  if (s_motor_runtime[DM_MOTOR_PITCH].state == CAN_MOTOR_STATE_ONLINE &&
      s_can_motor_link_state == CAN_MOTOR_LINK_WAIT_PITCH_FEEDBACK)
    s_can_motor_link_state = CAN_MOTOR_LINK_ONLINE;
  can_motor_process_disable(now);
  /* 任一电机 Disable 未确认时只禁止其自身 Enable；确认本机 s=0 后才允许恢复。 */
  can_motor_process_timeout_recovery(now);
#else
  s_can_motor_health_flags = CAN_MOTOR_HEALTH_BUS_OFF |
                               CAN_MOTOR_HEALTH_LEFT_TIMEOUT |
                               CAN_MOTOR_HEALTH_RIGHT_TIMEOUT |
                               CAN_MOTOR_HEALTH_PITCH_TIMEOUT;
#endif
}

void Can_Motor_RequestBusOffRecovery(FDCAN_HandleTypeDef *hfdcan)
{
#if (CAN_MOTOR_COMMUNICATION_ENABLE != 0U)
  if (hfdcan == &hfdcan1) s_fdcan1_bus_off_pending = 1U;
#else
  (void)hfdcan;
#endif
}

Can_Motor_Link_State_t Can_Motor_GetLinkState(void)
{
  return s_can_motor_link_state;
}

uint32_t Can_Motor_GetHealthFlags(void)
{
  return s_can_motor_health_flags;
}

uint8_t Can_Motor_IsPitchCommandReady(void)
{
  const uint32_t pitch_blocking_flags = CAN_MOTOR_HEALTH_PITCH_TIMEOUT |
      CAN_MOTOR_HEALTH_PITCH_DRIVER_FAULT |
      CAN_MOTOR_HEALTH_PITCH_DATA_FAULT |
      CAN_MOTOR_HEALTH_BUS_OFF;
  uint8_t ready;
  /* Pitch 运行状态仅由 CanTask 写入；临界区避免本任务和 GimbalTask 在
   * s=0/s=1 切换时看到彼此不一致的状态组合。 */
  portENTER_CRITICAL();
  ready = ((s_can_motor_health_flags & pitch_blocking_flags) == 0U &&
           s_motor_runtime[DM_MOTOR_PITCH].state == CAN_MOTOR_STATE_ONLINE) ? 1U : 0U;
  portEXIT_CRITICAL();
  return ready;
}

uint8_t Can_Motor_GetPitchFeedback(DM_Motor_Feedback_t *feedback)
{
  if (feedback == NULL) return 1U;

  /* Pitch 反馈由 CanTask 连续写入多个字段；此处复制完整快照时禁止任务切换，
   * 避免消费者拿到一半来自旧帧、一半来自新帧的数据。ISR 不会写此快照。 */
  portENTER_CRITICAL();
  *feedback = s_pitch_feedback;
  portEXIT_CRITICAL();
  return (feedback->frame_count == 0U) ? 1U : 0U;
}

void Can_Motor_GetDiagnostic(Can_Motor_Diagnostic_t *diagnostic)
{
  if (diagnostic == NULL) return;
  portENTER_CRITICAL();
  *diagnostic = s_can_motor_diagnostic;
  portEXIT_CRITICAL();
}

void Can_Motor_OnFeedbackFrame(DM_Motor_Index_t index, const uint8_t data[8],
                               uint8_t data_length)
{
  if (index > DM_MOTOR_PITCH || can_motor_is_present(index) == 0U ||
      data == NULL || Can_Motor_Rx_QueueHandle == NULL) return;
  memcpy(s_isr_rx_snapshot.data[index], data, 8U);
  s_isr_rx_snapshot.length[index] = data_length;
  s_isr_rx_snapshot.sequence[index]++;
  Can_Motor_Rx_Snapshot_t event = s_isr_rx_snapshot;
  if (osMessageQueuePut(Can_Motor_Rx_QueueHandle, &event, 0U, 0U) != osOK) {
    Can_Motor_Rx_Snapshot_t discarded;
    (void)osMessageQueueGet(Can_Motor_Rx_QueueHandle, &discarded, NULL, 0U);
    (void)osMessageQueuePut(Can_Motor_Rx_QueueHandle, &event, 0U, 0U);
  }
}

uint8_t Can_Motor_PitchMIT(float kp, float kd, float position_rad,
                           float velocity_rad_s, float torque_nm)
{
  return DM_Motor_MIT_Control(&s_motor_pitch, kp, kd, position_rad,
                              velocity_rad_s, torque_nm);
}

float Can_Motor_PitchTorqueLimit(void)
{
  /* 返回运行安全扭矩而非协议 TMAX；PID 因而不会产生超过电机峰值的输出。 */
  return DM_S3519_PEAK_TORQUE_NM;
}

uint8_t Can_Motor_FrictionSpeed(float left_rad_s, float right_rad_s)
{
  uint8_t status = 0U;
  status |= Can_Motor_LeftFrictionSpeed(left_rad_s);
  status |= Can_Motor_RightFrictionSpeed(right_rad_s);
  return status;
}

uint8_t Can_Motor_LeftFrictionSpeed(float velocity_rad_s)
{
  return DM_Motor_Speed_Control(&s_motor_left, velocity_rad_s);
}

uint8_t Can_Motor_RightFrictionSpeed(float velocity_rad_s)
{
  return DM_Motor_Speed_Control(&s_motor_right, velocity_rad_s);
}

