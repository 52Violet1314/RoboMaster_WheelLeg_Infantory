#include "Task.h"
#include "Mode.h"
#include "cmsis_os2.h"
#include "Can_Motor.h"
#include "Fric.h"
#include "Imu.h"
#include "Pid.h"

static void gimbal_send_pitch_zero_output(void)
{
  (void)Can_Motor_PitchMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void Task_Gimbal_Task(void *argument)
{
  (void)argument;
  Pitch_Control_Init();
  Imu_Pitch_Packet_t imu_pkt = {0};
  uint32_t previous_timestamp_us = 0U;
  uint8_t have_imu = 0U;
  Mode_Control_Command_t control_command = {0};
  uint32_t last_keyboard_frame_count = 0U;
  uint32_t last_keyboard_update_tick = 0U;
  float keyboard_pitch_target_rad = 0.0f;
  uint8_t keyboard_position_active = 0U;
  uint32_t last_motor_command_tick = 0U;
  uint8_t motor_command_has_been_sent = 0U;
  uint32_t last_left_friction_command_tick = 0U;
  uint32_t last_right_friction_command_tick = 0U;
  uint8_t left_friction_command_has_been_sent = 0U;
  uint8_t right_friction_command_has_been_sent = 0U;
  /* 0->1 边沿表示刚进入失能、故障、超时或恢复等待，需清一次 PID。 */
  uint8_t previous_pitch_output_inhibited = 1U;
  for (;;) {
    const uint32_t now = HAL_GetTick();
    while (osMessageQueueGet(Gimbal_Imu_QueueHandle, &imu_pkt, NULL, 0U) == osOK)
      have_imu = 1U;
    while (osMessageQueueGet(Gimbal_Control_QueueHandle, &control_command, NULL, 0U) == osOK) {}
    const uint32_t imu_faults = Task_GetImuFaultFlags();
    const uint32_t mode_faults = Task_GetModeFaultFlags();
    /* GimbalTask 不再计算任何输入时间差；ModeTask 已完成来源/视觉超时判定，
     * ImuTask 已完成 IMU 超时判定。这里仅检查是否已经收到首帧供算法使用。 */
    const uint8_t imu_ok = (have_imu != 0U) &&
        ((imu_faults & IMU_FAULT_SAMPLE_TIMEOUT) == 0U);
    const uint8_t command_ok = (control_command.command_valid != 0U) &&
        ((mode_faults & MODE_FAULT_COMMAND_LOST) == 0U);
    const uint8_t friction_input_ok = (control_command.total_enable != 0U) &&
        (command_ok != 0U);
    /* 电机反馈时间戳、驱动器状态和超时恢复均由 CanTask 管理。GimbalTask 只读取
     * 这份统一健康状态来决定哪个执行端必须写零。 */
    const uint32_t motor_health = Can_Motor_GetHealthFlags();
    uint32_t faults = GIMBAL_FAULT_NONE;
    if (imu_ok == 0U) faults |= GIMBAL_FAULT_IMU_TIMEOUT;
    if (command_ok == 0U) faults |= GIMBAL_FAULT_COMMAND_LOST;
    if ((motor_health & CAN_MOTOR_HEALTH_PITCH_TIMEOUT) != 0U)
      faults |= GIMBAL_FAULT_MOTOR_TIMEOUT;
    if ((motor_health & CAN_MOTOR_HEALTH_LEFT_TIMEOUT) != 0U)
      faults |= GIMBAL_FAULT_LEFT_TIMEOUT;
    if ((motor_health & CAN_MOTOR_HEALTH_RIGHT_TIMEOUT) != 0U)
      faults |= GIMBAL_FAULT_RIGHT_TIMEOUT;
    const Can_Motor_Link_State_t can_link_state = Can_Motor_GetLinkState();
    /* WAIT_PITCH_FEEDBACK 表示 FDCAN 已启动，但至少有一台电机尚未确认在线；
     * 各电机超时位会分别拦截 Pitch 或摩擦轮，不把它误判成整个 CAN 离线。 */
    if (can_link_state == CAN_MOTOR_LINK_INIT ||
        can_link_state == CAN_MOTOR_LINK_INITIALIZING ||
        (motor_health & CAN_MOTOR_HEALTH_BUS_OFF) != 0U)
      faults |= GIMBAL_FAULT_CAN_OFFLINE;
    /* Pitch 的驱动器状态/回帧格式错误才属于 Pitch 停机条件。左右摩擦轮
     * 各自记录并仅拦截自身，绝不能因一侧摩擦轮错误停掉 Pitch。 */
    if ((motor_health & (CAN_MOTOR_HEALTH_PITCH_DRIVER_FAULT |
                         CAN_MOTOR_HEALTH_PITCH_DATA_FAULT)) != 0U)
      faults |= GIMBAL_FAULT_MOTOR_DRIVER;
    if ((motor_health & (CAN_MOTOR_HEALTH_LEFT_DRIVER_FAULT |
                         CAN_MOTOR_HEALTH_LEFT_DATA_FAULT)) != 0U)
      faults |= GIMBAL_FAULT_LEFT_DRIVER;
    if ((motor_health & (CAN_MOTOR_HEALTH_RIGHT_DRIVER_FAULT |
                         CAN_MOTOR_HEALTH_RIGHT_DATA_FAULT)) != 0U)
      faults |= GIMBAL_FAULT_RIGHT_DRIVER;
    /* 输出隔离策略：Pitch 只由自身的超时/状态/格式错误停止；左右摩擦轮
     * 各自按反馈和驱动器状态放行。IMU、指令、CAN 总线故障才统一停机。 */
    const uint32_t pitch_stop_faults = faults &
        (GIMBAL_FAULT_IMU_TIMEOUT | GIMBAL_FAULT_COMMAND_LOST |
         GIMBAL_FAULT_MOTOR_TIMEOUT | GIMBAL_FAULT_CAN_OFFLINE |
         GIMBAL_FAULT_MOTOR_DRIVER);
    /* s=0 不是驱动器故障，但电机尚未确认使能。它与超时/故障一样禁止 PID，
     * 直到 CanTask 收到有效 s=1 回帧。 */
    const uint8_t pitch_command_ready = Can_Motor_IsPitchCommandReady();
    const uint8_t pitch_output_inhibited =
        (pitch_stop_faults != GIMBAL_FAULT_NONE ||
         pitch_command_ready == 0U) ? 1U : 0U;
    /* 摩擦轮不依赖 IMU 姿态；IMU 超时只停 Pitch。摩擦轮仍必须响应自身输入
     * 失效和 CAN 总线离线，且左右各自的反馈/故障只拦截本侧。 */
    const uint32_t friction_common_stop_faults = faults &
        (GIMBAL_FAULT_COMMAND_LOST | GIMBAL_FAULT_CAN_OFFLINE);
    const uint8_t left_friction_allowed =
        (friction_input_ok != 0U) &&
        (friction_common_stop_faults == GIMBAL_FAULT_NONE) &&
        ((faults & (GIMBAL_FAULT_LEFT_TIMEOUT |
                    GIMBAL_FAULT_LEFT_DRIVER)) == 0U);
    const uint8_t right_friction_allowed =
        (friction_input_ok != 0U) &&
        (friction_common_stop_faults == GIMBAL_FAULT_NONE) &&
        ((faults & (GIMBAL_FAULT_RIGHT_TIMEOUT |
                    GIMBAL_FAULT_RIGHT_DRIVER)) == 0U);

    /* 正常闭环的 PID/ESO/MIT 使用 1 ms 节拍；故障、失能和恢复等待时的零
     * 扭矩安全帧仍保持原 3 ms 节拍，不能因提升正常控制频率而增加安全恢复流量。 */
    const uint8_t normal_pitch_command_due = (motor_command_has_been_sent == 0U) ||
        ((uint32_t)(now - last_motor_command_tick) >= TASK_GIMBAL_PERIOD_MS);
    const uint8_t safety_pitch_zero_due = (motor_command_has_been_sent == 0U) ||
        ((uint32_t)(now - last_motor_command_tick) >= GIMBAL_MOTOR_COMMAND_PERIOD_MS);
    const uint32_t left_friction_period_ms = (left_friction_allowed != 0U) ?
        TASK_GIMBAL_PERIOD_MS : GIMBAL_MOTOR_COMMAND_PERIOD_MS;
    const uint32_t right_friction_period_ms = (right_friction_allowed != 0U) ?
        TASK_GIMBAL_PERIOD_MS : GIMBAL_MOTOR_COMMAND_PERIOD_MS;
    const uint8_t left_friction_command_due =
        (left_friction_command_has_been_sent == 0U) ||
        ((uint32_t)(now - last_left_friction_command_tick) >= left_friction_period_ms);
    const uint8_t right_friction_command_due =
        (right_friction_command_has_been_sent == 0U) ||
        ((uint32_t)(now - last_right_friction_command_tick) >= right_friction_period_ms);

    /* FDCAN 已启动即可给摩擦轮发命令；WAIT_PITCH_FEEDBACK 不再阻塞摩擦轮。 */
    const uint8_t friction_can_ready =
        (can_link_state != CAN_MOTOR_LINK_INIT) &&
        (can_link_state != CAN_MOTOR_LINK_INITIALIZING);
    if (friction_can_ready != 0U && left_friction_command_due != 0U) {
      Fric_Control_UpdateLeft(control_command.friction_wheel_on, left_friction_allowed);
      last_left_friction_command_tick = now;
      left_friction_command_has_been_sent = 1U;
    }
    if (friction_can_ready != 0U && right_friction_command_due != 0U) {
      Fric_Control_UpdateRight(control_command.friction_wheel_on, right_friction_allowed);
      last_right_friction_command_tick = now;
      right_friction_command_has_been_sent = 1U;
    }

    if (pitch_output_inhibited != 0U) {
      /* 失能、超时、故障或恢复中均只清一次 PID 状态并发送零 MIT。CanTask
       * 发送 Enable 后，必须先收到 s=1，才允许这里重新产生 PID 扭矩。 */
      if (previous_pitch_output_inhibited == 0U) {
        Pitch_Control_Reset();
        previous_timestamp_us = 0U;
        keyboard_position_active = 0U;
      }
      if (safety_pitch_zero_due != 0U) {
        gimbal_send_pitch_zero_output();
        last_motor_command_tick = now;
        motor_command_has_been_sent = 1U;
      }
      previous_pitch_output_inhibited = 1U;
      osDelay(TASK_GIMBAL_PERIOD_MS);
      continue;
    }

    previous_pitch_output_inhibited = 0U;
    /* 正常状态下每 1 ms 运行 PID/ESO 并发送 Pitch MIT 帧。 */
    if (normal_pitch_command_due == 0U) {
      osDelay(TASK_GIMBAL_PERIOD_MS);
      continue;
    }
    last_motor_command_tick = now;
    motor_command_has_been_sent = 1U;

    float dt = 0.001f;
    if (previous_timestamp_us != 0U) {
      dt = (float)(imu_pkt.timestamp_us - previous_timestamp_us) * 1e-6f;
      if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;
    }
    previous_timestamp_us = imu_pkt.timestamp_us;

    Pitch_Control_Mode_t mode = PITCH_CONTROL_MANUAL;
    float target = control_command.gimbal_pitch_rad_s;
    if (control_command.vision_mode_on != 0U) {
      mode = PITCH_CONTROL_VISION;
      target = control_command.vision_pitch_rad;
      keyboard_position_active = 0U;
    } else if (control_command.source == MODE_CONTROL_SOURCE_KEYBOARD_MOUSE) {
      /* Hero 的 mouse_x/y 是每一包的位移增量。ModeTask 虽以 1 kHz 发布
       * 最新命令，故只在 source_frame_count 变化时积分一次，绝不能每控制周期
       * 重复累计同一帧。首次进入以当前 IMU 姿态作目标，避免模式切换跳变。 */
      mode = PITCH_CONTROL_KEYBOARD_POSITION;
      if (keyboard_position_active == 0U) {
        keyboard_pitch_target_rad = imu_pkt.pitch_rad;
        keyboard_position_active = 1U;
        last_keyboard_frame_count = control_command.source_frame_count;
        last_keyboard_update_tick = control_command.update_tick;
      } else if (control_command.source_frame_count != last_keyboard_frame_count) {
        float input_dt = (float)(control_command.update_tick - last_keyboard_update_tick) * 0.001f;
        if (input_dt <= 0.0f || input_dt > 0.1f) input_dt = 0.001f;
        keyboard_pitch_target_rad += control_command.gimbal_pitch_rad_s * input_dt;
        last_keyboard_frame_count = control_command.source_frame_count;
        last_keyboard_update_tick = control_command.update_tick;
      }
      target = keyboard_pitch_target_rad;
    } else {
      keyboard_position_active = 0U;
    }
    /* 到机械上下限时，手操只禁止继续向外的目标速度，反向回退仍正常通过。
     * 视觉/键鼠的 target 是绝对角度，仍在进入 PID 前夹紧。 */
    if (mode == PITCH_CONTROL_MANUAL) {
      if ((imu_pkt.pitch_rad >= GIMBAL_PITCH_LIMIT_UPPER_RAD && target > 0.0f) ||
          (imu_pkt.pitch_rad <= GIMBAL_PITCH_LIMIT_LOWER_RAD && target < 0.0f))
        target = 0.0f;
    } else {
      if (target > GIMBAL_PITCH_LIMIT_UPPER_RAD)
        target = GIMBAL_PITCH_LIMIT_UPPER_RAD;
      else if (target < GIMBAL_PITCH_LIMIT_LOWER_RAD)
        target = GIMBAL_PITCH_LIMIT_LOWER_RAD;
      if (mode == PITCH_CONTROL_KEYBOARD_POSITION)
        keyboard_pitch_target_rad = target;
    }
    Pitch_Control_Update(dt, imu_pkt.pitch_rad, imu_pkt.gyro_pitch_rad_s, mode, target);
    osDelay(TASK_GIMBAL_PERIOD_MS);
  }
}
