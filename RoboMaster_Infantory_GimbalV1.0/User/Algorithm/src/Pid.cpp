/**
 ******************************************************************************
 * @file    Pid.cpp
 * @brief   Pitch 轴控制器
 *
 * 整体架构(对照参考工程 gimbal_task.c):
 *   1. 手操模式 (MANUAL)
 *        遥控器下发速度(rad/s) -> 独立速度 PID
 *        -> MIT 扭矩控制
 *        反馈: IMU pitch 角速度
 *
 *   2. 视觉自瞄模式 (VISION)
 *        视觉端下发目标 IMU pitch 绝对角(rad) -> S 曲线平滑(输出仍是 IMU 角)
 *        -> 独立角度 PID(反馈=IMU pitch)
 *        -> MIT 扭矩控制
 *
 * 安全保护:
 *   写电机前检查 IMU pitch 是否在 [lower, upper] 限幅内, 超限写 0 速度
 *
 * 模式切换:
 *   由状态机分派, 两套 PID 用不同结构体封装, 参数独立
 ******************************************************************************
 */
#include "Pid.h"
#include "s_curve.h"
#include "ESO.h"
#include <math.h>

/* Pid.cpp 是 C++ 单元, 而下列头文件由 C 编译, 必须 extern "C" 否则链接时名字被修饰 */
#ifdef __cplusplus
extern "C" {
#endif
#include "Can_Motor.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* ================================================================
 * 通用 PID 类实现
 * ================================================================ */
Pid::Pid() : kp_(0), ki_(0), kd_(0), limit_(0), integral_limit_(0),
             integral_(0), last_p_term_(0), last_i_term_(0),
             previous_measurement_(0), derivative_initialized_(0U) {}

void Pid::Init(float kp, float ki, float kd, float output_limit,
               float integral_limit) {
  kp_ = kp; ki_ = ki; kd_ = kd; limit_ = output_limit;
  integral_limit_ = integral_limit; Reset();
}

float Pid::Calculate(float reference, float measurement, float dt) {
  /* 误差 = 期望 - 测量 */
  const float error = reference - measurement;

  /* dt 合法性兜底: 防止除零与积分爆炸 */
  if (dt <= 0.0f) dt = 0.001f;

  /* 积分项直接保存为“已乘 Ki 的输出贡献”，因此 integral_limit_ 可以和
   * output_limit_ 使用同一物理单位。 */
  integral_ += ki_ * error * dt;
  if (integral_limit_ > 0.0f) {
    if (integral_ >  integral_limit_) integral_ =  integral_limit_;
    if (integral_ < -integral_limit_) integral_ = -integral_limit_;
  }

  /* 微分先行：仅对反馈测量值求导，取负号后与 d(error)/dt 的阻尼方向一致。
   * 新使能、模式切换或 Reset 后首拍不计算微分，避免历史值缺失导致扭矩突刺。 */
  float derivative = 0.0f;
  if (derivative_initialized_ != 0U)
    derivative = -(measurement - previous_measurement_) / dt;
  previous_measurement_ = measurement;
  derivative_initialized_ = 1U;

  /* PID 合成 + 输出限幅 */
  last_p_term_ = kp_ * error;
  last_i_term_ = integral_;
  float output = last_p_term_ + last_i_term_ + kd_ * derivative;
  if (limit_ > 0.0f) {
    if (output >  limit_) output =  limit_;
    if (output < -limit_) output = -limit_;
  }
  return output;
}

void Pid::Reset()
{
  integral_ = 0.0f;
  last_p_term_ = 0.0f;
  last_i_term_ = 0.0f;
  previous_measurement_ = 0.0f;
  derivative_initialized_ = 0U;
}
void Pid::SetIntegralLimit(float limit) { integral_limit_ = limit; }

/* ================================================================
 * Pitch 控制器: 静态对象(两套 PID + S 曲线 + 限幅)
 * ================================================================ */

/* 视觉模式外环：反馈 IMU pitch(rad)，输出目标角速度(rad/s)。 */
static Pid s_pitch_angle_pid;
/* 视觉模式内环：反馈 IMU pitch 角速度，输出 MIT 扭矩 Nm。 */
static Pid s_pitch_vision_speed_pid;

/* 手操模式 PID: 速度环(独立参数, 与视觉模式解耦) */
static Pid s_pitch_manual_pid;  /* 反馈 IMU 角速度，输出 torque_nm */

/* S 曲线规划器(视觉模式用, 平滑目标 IMU pitch 角) */
static SCurve_t s_pitch_curve;

/* ESO 实例(两种模式共用, 在线估扰动做前馈补偿)
 *   数据来源: Pitch_Control_Update 入参的 imu_pitch_rad / imu_gyro_rad_s
 *            （即 Task_Gimbal_Task 从 IMU 队列取得的数据）
 *   写电机指令 u_cmd 使用 MIT 扭矩指令 (Nm)。 */
static ESO_t s_pitch_eso;
/* 上一拍成功进入 FDCAN 发送队列、折算到 IMU Pitch 正方向的限幅扭矩。ESO 用它
 * 解释本拍测得的角速度，绝不使用尚未作用到机构上的本拍新指令。 */
static float s_pitch_last_applied_torque_nm;

/* 初始化标志, 防 Update 在 Init 前被调用 */
static uint8_t s_pitch_initialized;

/* DebugTask 每秒取一次快照；仅一个控制任务写入，读到跨周期字段仅影响显示。 */
static volatile Pitch_Control_Diagnostic_t s_pitch_control_diagnostic;

static void pitch_control_clear_diagnostic(void)
{
  s_pitch_control_diagnostic.velocity_reference_rad_s = 0.0f;
  s_pitch_control_diagnostic.measured_velocity_rad_s = 0.0f;
  s_pitch_control_diagnostic.p_term_nm = 0.0f;
  s_pitch_control_diagnostic.i_term_nm = 0.0f;
  s_pitch_control_diagnostic.torque_command_nm = 0.0f;
  s_pitch_control_diagnostic.mode = (uint8_t)PITCH_CONTROL_MANUAL;
}

/* 上次控制模式, 用于检测 VISION 上升沿并 Reset S 曲线锚点 */
static Pitch_Control_Mode_t s_pitch_prev_mode = PITCH_CONTROL_MANUAL;

/**
 * @brief 初始化两套 PID, S 曲线与限幅.
 * @note  参数对照参考工程:
 *   - 角度环/速度环参数从 gimbal_task.c 的 PITCH_POS_KP 等迁移
 *   - S 曲线 v_max/a_max/j_max 取参考工程 PITCH_SCURVE_* 的量级
 */
void Pitch_Control_Init(void)
{
  const float motor_torque_limit = Can_Motor_PitchTorqueLimit();
  const float manual_output_limit =
      (PITCH_MANUAL_OUTPUT_LIMIT_NM < motor_torque_limit) ?
          PITCH_MANUAL_OUTPUT_LIMIT_NM : motor_torque_limit;
  const float position_speed_output_limit =
      (PITCH_POSITION_SPEED_OUTPUT_LIMIT_NM < motor_torque_limit) ?
          PITCH_POSITION_SPEED_OUTPUT_LIMIT_NM : motor_torque_limit;

  /* 视觉外环：角度(rad) -> Pitch 本体目标角速度(rad/s)。输出限幅必须与
   * 遥控、键鼠的最大手操速度一致，不能使用电机协议的 30 rad/s。 */
  s_pitch_angle_pid.Init(8.0f, 0.0f, 0.15f,
                         PITCH_POSITION_ANGLE_OUTPUT_LIMIT_RAD_S,
                         PITCH_POSITION_ANGLE_INTEGRAL_LIMIT_RAD_S);

  /* 视觉内环：角速度(rad/s) -> MIT 扭矩(Nm)。PID 内部上限与最终发送上限
   * 同为电机峰值扭矩，避免 PID 自身在电机硬限幅外继续积累。 */
  s_pitch_vision_speed_pid.Init(1.5f, 0.02f, 0.02f,
                                position_speed_output_limit,
                                PITCH_POSITION_SPEED_INTEGRAL_LIMIT_NM);

  /* 遥控手操速度 PID：输入/反馈为 rad/s，输出同样限制到峰值扭矩。 */
  s_pitch_manual_pid.Init(0.85f, 0.6f, 0.15f,
                          manual_output_limit,
                          PITCH_MANUAL_INTEGRAL_LIMIT_NM);

  /* S 曲线全部使用 rad / rad/s / rad/s^2 / rad/s^3。 */
  SCurve_InitSeg7(&s_pitch_curve, 3.0f, 250.0f, 8000.0f);
  SCurve_SetDeadband(&s_pitch_curve, 0.001f);

  /* ESO: b0 严格随 J 取 1/J；前馈在最终总扭矩限幅前受限叠加。 */
  ESO_Init(&s_pitch_eso, PITCH_AXIS_INERTIA_J,
           ESO_DEFAULT_B0, ESO_DEFAULT_OMEGA_O);
  s_pitch_last_applied_torque_nm = 0.0f;

  s_pitch_initialized = 1U;
  s_pitch_prev_mode   = PITCH_CONTROL_MANUAL;
  pitch_control_clear_diagnostic();
}

/**
 * @brief 清除 pitch 控制器的动态状态。
 * @details
 *   IMU 超时后电机必须立即停止；同时清掉 PID 积分、微分历史、ESO
 *   观测状态和 S 曲线段状态，防止 IMU 恢复后的第一拍继承旧数据而冲击。
 */
void Pitch_Control_Reset(void)
{
  if (s_pitch_initialized == 0U) return;
  s_pitch_angle_pid.Reset();
  s_pitch_vision_speed_pid.Reset();
  s_pitch_manual_pid.Reset();
  SCurve_Reset(&s_pitch_curve, 0.0f);
  ESO_Reset(&s_pitch_eso);
  s_pitch_last_applied_torque_nm = 0.0f;
  s_pitch_prev_mode = PITCH_CONTROL_MANUAL;
  pitch_control_clear_diagnostic();
}

void Pitch_Control_GetDiagnostic(Pitch_Control_Diagnostic_t *diagnostic)
{
  if (diagnostic == NULL) return;
  diagnostic->velocity_reference_rad_s =
      s_pitch_control_diagnostic.velocity_reference_rad_s;
  diagnostic->measured_velocity_rad_s =
      s_pitch_control_diagnostic.measured_velocity_rad_s;
  diagnostic->p_term_nm = s_pitch_control_diagnostic.p_term_nm;
  diagnostic->i_term_nm = s_pitch_control_diagnostic.i_term_nm;
  diagnostic->torque_command_nm = s_pitch_control_diagnostic.torque_command_nm;
  diagnostic->mode = s_pitch_control_diagnostic.mode;
}

/**
 * @brief Pitch 控制统一入口(状态机分派视觉/手操模式)
 *
 * 数据流:
 *   VISION 模式:
 *     target(绝对 IMU 角 rad) -> S 曲线 -> 平滑 IMU 角
 *     -> 角度环(反馈=imu_pitch_rad) -> 期望角速度
 *     -> 速度环(反馈=imu_gyro_rad_s) -> 电机速度
 *     -> DM_Motor_MIT_Control
 *
 *   MANUAL 模式:
 *     target(速度指令 rad/s) -> 速度 PID(反馈=imu_gyro_rad_s) -> MIT 扭矩
 *
 * 安全职责:
 *   GimbalTask 已完成实际角度保护和绝对目标限幅；本函数只计算 PID/ESO
 *   及写入最终 MIT 扭矩，不再决定停机或机械限位。
 */
void Pitch_Control_Update(float dt, float imu_pitch_rad, float imu_gyro_rad_s,
                          Pitch_Control_Mode_t mode, float target)
{
  /* 防御: Init 未执行则先执行 */
  if (s_pitch_initialized == 0U) Pitch_Control_Init();

  /* dt 合法性兜底 */
  if (dt <= 0.0f) dt = 0.001f;

  /* === 模式切换：清除即将启用的 PID 状态，避免旧模式状态带入。 === */
  if (mode != s_pitch_prev_mode) {
    if (mode != PITCH_CONTROL_MANUAL) {
      /* 手操 -> 视觉：S 曲线从当前 IMU pitch 起步。 */
      SCurve_Reset(&s_pitch_curve, imu_pitch_rad);
      s_pitch_angle_pid.Reset();
      s_pitch_vision_speed_pid.Reset();
    } else {
      /* 视觉 -> 手操：手操速度 PID 从零动态状态开始。 */
      s_pitch_manual_pid.Reset();
    }
    /* 模式切换与 PID 动态状态同步清 ESO，避免旧模式的扰动估计成为新模式扭矩。 */
    ESO_Reset(&s_pitch_eso);
    s_pitch_last_applied_torque_nm = 0.0f;
  }
  s_pitch_prev_mode = mode;

  /* === ESO: 用本拍测量值和上一拍已实际发送的最终扭矩更新。 ===
     以本拍 omega[k] 配本拍刚计算的 u[k] 会把尚未施加的输入错当成已作用，
     因此这里严格使用 u_applied[k-1]；随后才计算并发送新的 u[k]。 */
  s_pitch_eso.omega = imu_gyro_rad_s;
  s_pitch_eso.u_cmd = s_pitch_last_applied_torque_nm;
  ESO_Update(&s_pitch_eso, dt);

  /* === 状态机分派 === */
  float torque_cmd = 0.0f;  /* IMU Pitch 正方向的等效扭矩 Nm */
  float velocity_ref = target;
  const Pid *torque_pid = &s_pitch_manual_pid;

  if (mode != PITCH_CONTROL_MANUAL) {
    /* --- 视觉自瞄: S 曲线规划目标 IMU 角 -> 角度 PID -> MIT 扭矩 --- */

    /* S 曲线 target 是绝对 IMU pitch 角, pos 是平滑后的 IMU pitch 角 */
    SCurve_SetTarget(&s_pitch_curve, target);
    SCurve_Update(&s_pitch_curve, dt);
    const float smooth_target = s_pitch_curve.pos;  /* 平滑后 IMU 角 */

    /* 外环：IMU pitch 角度反馈，输出目标角速度。 */
    velocity_ref = s_pitch_angle_pid.Calculate(smooth_target, imu_pitch_rad, dt);
    /* 内环：IMU pitch 角速度反馈，输出 MIT 扭矩。 */
    torque_cmd = s_pitch_vision_speed_pid.Calculate(velocity_ref,
                                                    imu_gyro_rad_s, dt);
    torque_pid = &s_pitch_vision_speed_pid;
  } else {
    /* --- 手操模式: 不经过曲线，速度 PID 直接输出 MIT 扭矩。 --- */

    /* target 语义为速度指令 rad/s */
    torque_cmd = s_pitch_manual_pid.Calculate(target, imu_gyro_rad_s, dt);
  }

  /* ESO 前馈已在内部完成 8 Hz 低通、±0.2 Nm 限幅和变化率限制。它与 PID
   * 同属 IMU Pitch 正方向，必须在最终总扭矩限幅之前叠加。 */
  torque_cmd += ESO_GetFeedforward(&s_pitch_eso);
  if (!isfinite(torque_cmd)) torque_cmd = 0.0f;
  const float torque_limit = Can_Motor_PitchTorqueLimit();
  if (torque_cmd > torque_limit)
    torque_cmd = torque_limit;
  else if (torque_cmd < -torque_limit)
    torque_cmd = -torque_limit;

  /* === 写 MIT 扭矩控制帧 ===
     PID/ESO 统一以 IMU Pitch 正方向为正。实测该方向与 DM-S3519 回帧速度方向
     相反（Pitch 增大时 motor velocity 为负），故只在最终 MIT 物理扭矩边界反号。
     禁止反转 IMU、PID 误差或限位，否则视觉/键鼠的绝对角坐标会被破坏。 */
  const float mit_torque_nm = -torque_cmd;
  if (Can_Motor_PitchMIT(0.0f, 0.0f,
                          0.0f, 0.0f, mit_torque_nm) == 0U)
    /* ESO 输入必须仍是机构实际得到的 IMU 坐标正扭矩，而不是协议符号。 */
    s_pitch_last_applied_torque_nm = torque_cmd;

  /* 发送完成后才记录诊断：这里不参与下一拍控制，也不改变 CAN 帧内容。 */
  s_pitch_control_diagnostic.velocity_reference_rad_s = velocity_ref;
  s_pitch_control_diagnostic.measured_velocity_rad_s = imu_gyro_rad_s;
  s_pitch_control_diagnostic.p_term_nm = torque_pid->LastPTerm();
  s_pitch_control_diagnostic.i_term_nm = torque_pid->LastITerm();
  s_pitch_control_diagnostic.torque_command_nm = torque_cmd;
  s_pitch_control_diagnostic.mode = (uint8_t)mode;
}
#endif
