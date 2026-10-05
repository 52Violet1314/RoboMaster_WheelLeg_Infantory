#include "Task.h"
#include "Mode.h"
#include "cmsis_os2.h"
#include "Remote.h"
#include "Vision.h"
#include "Refree.h"
#include "main.h"

static volatile uint32_t s_mode_fault_flags;
static volatile Input_Link_Diagnostic_t s_mode_input_diagnostic;

uint32_t Task_GetModeFaultFlags(void)
{
  return s_mode_fault_flags;
}

void Task_GetModeInputDiagnostic(Input_Link_Diagnostic_t *diagnostic)
{
  if (diagnostic == NULL) return;
  *diagnostic = s_mode_input_diagnostic;
}

static int16_t mode_deadband(int16_t value)
{
  return (value > -MODE_STICK_DEADBAND && value < MODE_STICK_DEADBAND) ? 0 : value;
}

static float mode_clamp_abs(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static float mode_clamp_range(float value, float lower, float upper)
{
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

/* 将两个单向旋钮的前 48 个 SBUS 原始值作为死区，避免最小端抖动产生指令。 */
static float mode_map_knob(uint16_t raw, float minimum, float maximum)
{
  if (raw < MODE_KNOB_ACTIVE_MIN_RAW) return minimum;
  if (raw > REMOTE_SBUS_MAX_VALUE) raw = REMOTE_SBUS_MAX_VALUE;
  return minimum + ((float)(raw - MODE_KNOB_ACTIVE_MIN_RAW) /
                    MODE_KNOB_ACTIVE_SPAN_RAW) * (maximum - minimum);
}

/**
 * @brief 向一个“只保留最新帧”的消费队列发布完整 Mode 命令。
 * @note  队列满时丢弃旧帧；控制与调试消费者都只应读取最新指令，发布者绝不等待。
 */
static void mode_publish_latest(osMessageQueueId_t queue,
                                const Mode_Control_Command_t *command)
{
  Mode_Control_Command_t discarded;
  if (queue == NULL || command == NULL) return;
  if (osMessageQueuePut(queue, command, 0U, 0U) != osOK) {
    (void)osMessageQueueGet(queue, &discarded, NULL, 0U);
    (void)osMessageQueuePut(queue, command, 0U, 0U);
  }
}

/* 仅由 ModeTask 调用：把已校验的原始 SBUS 通道解释为统一 SI 执行命令。 */
static void mode_map_remote(const Remote_Data_t *raw, Mode_Control_Command_t *command)
{
  const int16_t ch1 = mode_deadband(raw->channel[0]);
  const int16_t ch2 = mode_deadband(raw->channel[1]);
  const int16_t ch3 = mode_deadband(raw->channel[2]);
  const int16_t ch4 = mode_deadband(raw->channel[3]);
  command->chassis_vy_m_s = (float)ch1 / MODE_STICK_FULL_SCALE * MODE_CHASSIS_MAX_VY_M_S;
  command->chassis_vx_m_s = (float)ch2 / MODE_STICK_FULL_SCALE * MODE_CHASSIS_MAX_VX_M_S;
  command->gimbal_pitch_rad_s = (float)ch3 / MODE_STICK_FULL_SCALE * PITCH_MAX_RATE_RAD_S;
  command->gimbal_yaw_rad_s = (float)ch4 / MODE_STICK_FULL_SCALE * MODE_GIMBAL_YAW_MAX_RAD_S;
  command->chassis_yaw_rad_s = mode_map_knob(raw->channel_raw[4], 0.0f,
                                              MODE_CHASSIS_YAW_MAX_RAD_S);
  command->leg_length_m = mode_map_knob(raw->channel_raw[5], MODE_LEG_LENGTH_MIN_M,
                                         MODE_LEG_LENGTH_MAX_M);
  command->friction_wheel_on = (raw->channel[6] > MODE_SWITCH_MID_THRESHOLD) ? 1U : 0U;
  command->vision_mode_on = (raw->channel[7] > MODE_SWITCH_MID_THRESHOLD) ? 1U : 0U;
  command->spin_mode_enable = (raw->channel[8] > MODE_SWC_MAX_THRESHOLD) ? 1U : 0U;
  command->total_enable = (raw->channel[9] > MODE_SWITCH_MID_THRESHOLD) ? 1U : 0U;
  command->leg_mode_enable = command->total_enable;
}

/* 仅由 ModeTask 调用：把完整键鼠原始状态解释为连续执行指令。长按/锁存
 * 状态机仍在本任务主循环中处理，不能由通信模块或业务执行任务抢占。 */
static void mode_map_keyboard(const Referee_VtmKeyboardMouse_t *raw,
                              Mode_Control_Command_t *command)
{
  float speed_scale = MODE_KEYBOARD_NORMAL_SCALE;
  if (raw->key_shift != 0U) speed_scale = MODE_KEYBOARD_FAST_SCALE;
  else if (raw->key_ctrl != 0U) speed_scale = MODE_KEYBOARD_SLOW_SCALE;
  command->chassis_vx_m_s = ((float)raw->key_w - (float)raw->key_s) *
      MODE_CHASSIS_MAX_VX_M_S * speed_scale;
  command->chassis_vy_m_s = ((float)raw->key_d - (float)raw->key_a) *
      MODE_CHASSIS_MAX_VY_M_S * speed_scale;
  /* 符号与 Hero 一致。鼠标是增量，云台会按 source_frame_count 积分一次。 */
  command->gimbal_pitch_rad_s = mode_clamp_abs((float)raw->mouse_y *
      MODE_MOUSE_XY_RAD_S_PER_COUNT, PITCH_MAX_RATE_RAD_S);
  command->gimbal_yaw_rad_s = mode_clamp_abs(-(float)raw->mouse_x *
      MODE_MOUSE_XY_RAD_S_PER_COUNT, MODE_GIMBAL_YAW_MAX_RAD_S);
}

void Task_ModeTask(void *argument)
{
  (void)argument;
  Remote_Mode_Input_t remote = {0};
  Referee_VtmInput_t referee = {0};
  VisionToGimbal_t vision = {0};
  uint16_t previous_keys = 0U;
  uint32_t previous_keyboard_frame_count = 0U;
  uint32_t right_button_press_tick = 0U;
  uint8_t previous_right_button = 0U;
  uint8_t right_long_press_handled = 0U;
  uint8_t keyboard_friction_latched = 0U;
  uint8_t keyboard_vision_latched = 0U;
  uint8_t keyboard_spin_mode_latched = 0U;
  float keyboard_chassis_yaw_rad_s = 0.0f;
  float keyboard_leg_length_m = MODE_LEG_LENGTH_MIN_M;
  uint32_t command_sequence = 0U;

  for (;;) {
    const uint32_t now = HAL_GetTick();
    uint8_t got_referee = 0U;
    while (osMessageQueueGet(Mode_Remote_QueueHandle, &remote, NULL, 0U) == osOK) {}
    while (osMessageQueueGet(Mode_Referee_QueueHandle, &referee, NULL, 0U) == osOK) got_referee = 1U;
    while (osMessageQueueGet(Mode_Vision_QueueHandle, &vision, NULL, 0U) == osOK) {}

    const uint8_t remote_fresh = (remote.data.frame_valid != 0U) &&
        (remote.data.failsafe == 0U) &&
        ((uint32_t)(now - remote.data.update_tick) <= MODE_REMOTE_INPUT_TIMEOUT_MS);
    const uint8_t keyboard_fresh = (referee.keyboard_mouse.frame_count != 0U) &&
        ((uint32_t)(now - referee.keyboard_mouse.update_tick) <= MODE_KEYBOARD_MOUSE_TIMEOUT_MS);
    const uint8_t remote_timeout_active = (remote_fresh == 0U) ? 1U : 0U;
    if (remote_timeout_active != s_mode_input_diagnostic.timeout_active) {
      if (remote_timeout_active != 0U)
        s_mode_input_diagnostic.timeout_enter_count++;
      else
        s_mode_input_diagnostic.timeout_recover_count++;
      s_mode_input_diagnostic.timeout_active = remote_timeout_active;
    }
    s_mode_input_diagnostic.age_ms = (remote.data.frame_valid == 0U) ? UINT32_MAX :
        (uint32_t)(now - remote.data.update_tick);
    s_mode_input_diagnostic.valid_frame_count = remote.data.frame_count;
    s_mode_input_diagnostic.failsafe_active = remote.data.failsafe;
    s_mode_fault_flags = ((remote_fresh == 0U) ? MODE_FAULT_REMOTE_TIMEOUT : 0U) |
        ((keyboard_fresh == 0U) ? MODE_FAULT_KEYBOARD_TIMEOUT : 0U);

    if (got_referee != 0U && keyboard_fresh != 0U &&
        referee.keyboard_mouse.frame_count != previous_keyboard_frame_count) {
      const uint16_t keys = referee.keyboard_mouse.keyboard_value;
      const uint16_t pressed = (uint16_t)(keys & (uint16_t)~previous_keys);
      /* 离散功能均只在按下沿切换一次。 */
      if ((pressed & REFEREE_KEY_F) != 0U)
        keyboard_friction_latched = (keyboard_friction_latched == 0U) ? 1U : 0U;
      if ((pressed & MODE_KEY_SPIN_MODE) != 0U)
        keyboard_spin_mode_latched = (keyboard_spin_mode_latched == 0U) ? 1U : 0U;
      if ((pressed & MODE_KEY_YAW_DECREASE) != 0U)
        keyboard_chassis_yaw_rad_s -= MODE_KEY_YAW_STEP_RAD_S;
      if ((pressed & MODE_KEY_YAW_INCREASE) != 0U)
        keyboard_chassis_yaw_rad_s += MODE_KEY_YAW_STEP_RAD_S;
      keyboard_chassis_yaw_rad_s = mode_clamp_range(keyboard_chassis_yaw_rad_s,
          0.0f, MODE_CHASSIS_YAW_MAX_RAD_S);
      if ((pressed & MODE_KEY_LEG_LENGTH_DECREASE) != 0U)
        keyboard_leg_length_m -= MODE_KEY_LEG_LENGTH_STEP_M;
      if ((pressed & MODE_KEY_LEG_LENGTH_INCREASE) != 0U)
        keyboard_leg_length_m += MODE_KEY_LEG_LENGTH_STEP_M;
      keyboard_leg_length_m = mode_clamp_range(keyboard_leg_length_m,
          MODE_LEG_LENGTH_MIN_M, MODE_LEG_LENGTH_MAX_M);
      if (referee.keyboard_mouse.right_button_down != 0U && previous_right_button == 0U) {
        right_button_press_tick = referee.keyboard_mouse.update_tick;
        right_long_press_handled = 0U;
      }
      /* 对齐 Hero 的 1 秒长按阈值；本工程将其做成开/关锁存，下一次独立
       * 长按可退出视觉模式。按住期间 handled 防止每帧反复翻转。 */
      if (referee.keyboard_mouse.right_button_down != 0U &&
          right_long_press_handled == 0U &&
          (uint32_t)(now - right_button_press_tick) >= MODE_MOUSE_LONG_PRESS_MS) {
        keyboard_vision_latched = (keyboard_vision_latched == 0U) ? 1U : 0U;
        right_long_press_handled = 1U;
      }
      previous_keys = keys;
      previous_right_button = referee.keyboard_mouse.right_button_down;
      previous_keyboard_frame_count = referee.keyboard_mouse.frame_count;
    }

    Mode_Control_Command_t command = {0};
    if (keyboard_fresh != 0U) {
      mode_map_keyboard(&referee.keyboard_mouse, &command);
      command.friction_wheel_on = keyboard_friction_latched;
      command.vision_mode_on = keyboard_vision_latched;
      command.spin_mode_enable = keyboard_spin_mode_latched;
      command.chassis_yaw_rad_s = keyboard_chassis_yaw_rad_s;
      command.leg_length_m = keyboard_leg_length_m;
      /* 键鼠不设额外业务按键总使能：完整有效帧本身即表示输入可用。R 等
       * 其他未映射按键仍完整保留在 Referee 原始快照，绝不在这里产生动作。 */
      command.total_enable = 1U;
      command.leg_mode_enable = command.total_enable;
      command.command_valid = 1U;
      command.source = MODE_CONTROL_SOURCE_KEYBOARD_MOUSE;
      command.update_tick = referee.keyboard_mouse.update_tick;
      command.source_frame_count = referee.keyboard_mouse.frame_count;
    } else if (remote_fresh != 0U) {
      mode_map_remote(&remote.data, &command);
      command.command_valid = command.total_enable;
      command.source = MODE_CONTROL_SOURCE_REMOTE;
      command.update_tick = remote.data.update_tick;
      command.source_frame_count = remote.data.frame_count;
    }
    if (command.command_valid != 0U && command.vision_mode_on != 0U) {
      if (vision.frame_count == 0U ||
          (uint32_t)(now - vision.update_tick) > MODE_VISION_INPUT_TIMEOUT_MS) {
        command.command_valid = 0U;
        s_mode_fault_flags |= MODE_FAULT_VISION_TIMEOUT;
      } else {
        command.vision_yaw_rad = vision.yaw_rad;
        command.vision_pitch_rad = vision.pitch_rad;
      }
    }
    if (command.command_valid == 0U)
      s_mode_fault_flags |= MODE_FAULT_COMMAND_LOST;
    const uint8_t command_lost_active = (command.command_valid == 0U) ? 1U : 0U;
    const uint8_t previous_command_lost =
        (s_mode_input_diagnostic.command_valid == 0U) ? 1U : 0U;
    if (command_lost_active != previous_command_lost) {
      if (command_lost_active != 0U)
        s_mode_input_diagnostic.command_lost_enter_count++;
      else
        s_mode_input_diagnostic.command_lost_recover_count++;
    }
    s_mode_input_diagnostic.command_valid = command.command_valid;
    s_mode_input_diagnostic.command_source = command.source;
    /* 此时间戳属于 ModeTask -> 执行端这条独立链路；不得复用上游遥控、图传或
     * 视觉的接收时间。云台据此检测 ModeTask 停止发布命令的故障。 */
    command.publish_tick = now;
    command.sequence = ++command_sequence;

    /* 向云台发送唯一的统一控制命令。 */
    mode_publish_latest(Gimbal_Control_QueueHandle, &command);
    osDelay(TASK_MODE_PERIOD_MS);
  }
}

