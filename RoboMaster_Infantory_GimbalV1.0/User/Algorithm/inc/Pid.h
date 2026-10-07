#ifndef USER_ALGORITHM_PID_H
#define USER_ALGORITHM_PID_H

#include <stdint.h>

/* ============================================================
 * 控制模式状态机
 *   - PITCH_CONTROL_MANUAL : 手操模式, 速度 PID -> MIT 扭矩
 *   - PITCH_CONTROL_VISION : 视觉自瞄模式, S 曲线规划目标 IMU 角度(rad)
 *                            -> 角度 PID -> 速度 PID -> MIT 扭矩
 * ============================================================ */
typedef enum {
  PITCH_CONTROL_MANUAL = 0,
  PITCH_CONTROL_VISION = 1,
  PITCH_CONTROL_KEYBOARD_POSITION = 2 /* 鼠标帧积分出的手操绝对 pitch 目标。 */
} Pitch_Control_Mode_t;

/* 仅供低频串口诊断：P/I 是实际参与 MIT 扭矩输出的速度环贡献。 */
typedef struct {
  float velocity_reference_rad_s;
  float measured_velocity_rad_s;
  float p_term_nm;
  float i_term_nm;
  float torque_command_nm;
  uint8_t mode;
} Pitch_Control_Diagnostic_t;

/* Pitch 本体（IMU 测量轴）的最大手操/视觉目标角速度，单位 rad/s。
 * 这是云台机构安全速度，不是达妙电机 CAN 协议可编码的 30 rad/s。
 * 遥控、键鼠和视觉角度外环必须共用此值，避免不同来源切换时速度突变。 */
#define PITCH_MAX_RATE_RAD_S      (2.5f)

/* 各 PID 的“总输出”和“I 项”独立调节。Nm 类上限还会被 Can_Motor 的
 * DM_S3519 峰值扭矩硬限制再次夹紧；因此这里调小立即生效，调大不会越过电机保护。
 * 键鼠与视觉同走位置双环，故共用 POSITION_* 两组参数。 */
#define PITCH_MANUAL_OUTPUT_LIMIT_NM         (1.0f)
#define PITCH_MANUAL_INTEGRAL_LIMIT_NM       (0.4f)

#define PITCH_POSITION_ANGLE_OUTPUT_LIMIT_RAD_S    (2.5f)
#define PITCH_POSITION_ANGLE_INTEGRAL_LIMIT_RAD_S  (2.5f)
#define PITCH_POSITION_SPEED_OUTPUT_LIMIT_NM       (1.0f)
#define PITCH_POSITION_SPEED_INTEGRAL_LIMIT_NM     (1.0f)

#ifdef __cplusplus
/**
 * @brief 通用位置/速度 PID 控制器
 * @note Calculate 为位置式 PID。积分项直接以输出单位累加并限幅；微分采用
 *       测量值微分（微分先行），避免目标突变时产生微分冲击。
 */
class Pid {
 public:
  Pid();

  /**
   * @brief 初始化 PID 参数与限幅
   * @param kp              比例系数
   * @param ki              积分系数
   * @param kd              微分系数
   * @param output_limit    输出限幅(>0 时启用, 取绝对值)
   * @param integral_limit  积分项输出限幅(>0 时启用)。单位必须与 output_limit
   *                        相同：角度环为 rad/s，速度/扭矩环为 Nm。
   */
  void Init(float kp, float ki, float kd, float output_limit,
            float integral_limit = 0.0f);

  /**
   * @brief 单步 PID 计算
   * @param reference  期望值
   * @param measurement 测量值(反馈)
   * @param dt         控制周期 s, 非法时兜底为 1ms
   * @return PID 输出(已限幅)
   */
  float Calculate(float reference, float measurement, float dt);

  /** @brief 清零积分与微分项, 模式切换/使能上升沿调用 */
  void Reset();

  /** @brief 运行时修改积分限幅 */
  void SetIntegralLimit(float limit);

  /** @brief 最近一次计算中 P、I 对输出的贡献，单位与 PID 输出相同。 */
  float LastPTerm() const { return last_p_term_; }
  float LastITerm() const { return last_i_term_; }

 private:
  float kp_, ki_, kd_;              /**< PID 三系数 */
  float limit_;                     /**< 输出限幅 */
  float integral_limit_;            /**< 积分项输出限幅 */
  float integral_;                  /**< 已乘 Ki 的积分输出项 */
  float last_p_term_;               /**< 最近一次 P 输出贡献 */
  float last_i_term_;               /**< 最近一次 I 输出贡献 */
  float previous_measurement_;      /**< 上一周期测量值，用于测量值微分 */
  uint8_t derivative_initialized_;  /**< 首次测量标志，首拍微分强制为零 */
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 pitch 控制器(两套 PID + S 曲线).
 * @note  必须在 Update 前调用一次; 内部有 s_pitch_initialized 自检
 */
void Pitch_Control_Init(void);

/** @brief IMU 超时或安全停机时清除所有 pitch 控制内部状态。 */
void Pitch_Control_Reset(void);

/** @brief 读取最近一次 Pitch 速度环控制量；仅用于诊断，不参与控制。 */
void Pitch_Control_GetDiagnostic(Pitch_Control_Diagnostic_t *diagnostic);

/**
 * @brief Pitch 控制统一入口(状态机分派视觉/手操)
 * @param dt          控制周期 s
 * @param imu_pitch_rad     当前 IMU pitch 欧拉角 rad (反馈)
 * @param imu_gyro_rad_s    IMU pitch 角速度 rad/s (反馈)
 * @param mode        控制模式: MANUAL=遥控速度环；VISION/KEYBOARD_POSITION=角度环
 * @param target 目标量:
 *                 - VISION/KEYBOARD_POSITION 模式: 期望 IMU pitch 绝对角 rad
 *                 - MANUAL 模式: 遥控器下发的速度指令 rad/s
 */
void Pitch_Control_Update(float dt, float imu_pitch_rad, float imu_gyro_rad_s,
                          Pitch_Control_Mode_t mode, float target);

#ifdef __cplusplus
}
#endif

#endif
