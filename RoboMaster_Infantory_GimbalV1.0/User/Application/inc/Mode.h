#ifndef USER_APPLICATION_MODE_H
#define USER_APPLICATION_MODE_H

/* 输入有效性策略统一归 ModeTask 管理；通信模块只为已校验帧附加接收时间戳。 */
#include <stdint.h>
#include "cmsis_os2.h"
#include "Refree.h" /* 键鼠原始按键常量；仅 ModeTask 解释其业务含义。 */
#include "Pid.h"    /* Pitch 本体最大速度，供所有输入来源统一映射。 */

/* ModeTask 是遥控、图传键鼠、视觉三个原始控制来源的唯一消费者，因此这三项
 * 超时阈值只定义在此处。生产模块仅在一帧通过协议校验时更新各自 update_tick。
 * 当前为台架调试，全部临时放宽为 1 秒；正式运动前必须按各链路更新率收紧。 */
#define MODE_REMOTE_INPUT_TIMEOUT_MS        (1000U)
#define MODE_KEYBOARD_MOUSE_TIMEOUT_MS      (1000U)
#define MODE_VISION_INPUT_TIMEOUT_MS        (1000U)
#define MODE_MOUSE_LONG_PRESS_MS        (1000U) /* 对齐 Hero：右键长按切换视觉自瞄。 */

/* 键鼠离散开关分配：仅在按下沿翻转一次，避免按住重复触发。 */
#define MODE_KEY_SPIN_MODE               REFEREE_KEY_G /* G: 小陀螺模式开/关。 */
#define MODE_KEY_YAW_DECREASE             REFEREE_KEY_Q /* Q: 底盘 yaw 速度减小。 */
#define MODE_KEY_YAW_INCREASE             REFEREE_KEY_E /* E: 底盘 yaw 速度增大。 */
#define MODE_KEY_LEG_LENGTH_DECREASE      REFEREE_KEY_Z /* Z: 腿长缩短一步。 */
#define MODE_KEY_LEG_LENGTH_INCREASE      REFEREE_KEY_X /* X: 腿长增加一步。 */

/* ModeTask 是唯一的原始输入 -> SI 执行指令映射层。所有可调范围集中于此。 */
#define MODE_STICK_DEADBAND              (10)
#define MODE_STICK_FULL_SCALE            (783.0f)
#define MODE_SWITCH_MID_THRESHOLD        (0)
#define MODE_SWC_MAX_THRESHOLD           (512)
#define MODE_CHASSIS_MAX_VX_M_S          (1.2f)
#define MODE_CHASSIS_MAX_VY_M_S          (1.2f)
#define MODE_GIMBAL_YAW_MAX_RAD_S        (2.5f)
#define MODE_CHASSIS_YAW_MAX_RAD_S        (6.28318530718f)
#define MODE_LEG_LENGTH_MIN_M             (0.15f)
#define MODE_LEG_LENGTH_MAX_M             (0.35f)
/* 两个旋钮均使用 SBUS 原始值 0~2047：实测物理最小端约为 240，故 0~250
 * 全部作为死区，251~2047 线性映射到完整指令范围。 */
#define MODE_KNOB_DEADBAND_RAW             (250U)
#define MODE_KNOB_ACTIVE_MIN_RAW           (MODE_KNOB_DEADBAND_RAW + 1U)
/* 有效值为 251~2047（含两端），两端的数值跨度为 1796。 */
#define MODE_KNOB_ACTIVE_SPAN_RAW          (1796.0f)
#define MODE_KEY_YAW_STEP_RAD_S           (0.20f)
#define MODE_KEY_LEG_LENGTH_STEP_M        (0.005f)
#define MODE_KEYBOARD_NORMAL_SCALE        (0.60f)
#define MODE_KEYBOARD_FAST_SCALE          (1.00f)
#define MODE_KEYBOARD_SLOW_SCALE          (0.30f)
#define MODE_MOUSE_XY_RAD_S_PER_COUNT     (0.1f) /* Hero KEY_MOUSE_XY_GAIN。 */
#define MODE_FRIC_WHEEL_SPEED_RAD_S       (20.0f) /* 摩擦轮开启时的固定目标转速。 */

typedef enum {
  MODE_CONTROL_SOURCE_NONE = 0U,
  MODE_CONTROL_SOURCE_REMOTE,
  MODE_CONTROL_SOURCE_KEYBOARD_MOUSE
} Mode_ControlSource_t;

/* 不可变的 SI 单位成品命令帧。ModeTask 是唯一写入者，通过队列副本发布给
 * 消费任务；消费任务不能再次仲裁遥控/键鼠，也不得自行修改模式状态。 */
typedef struct {
  float chassis_vx_m_s;       /* 底盘前后线速度，m/s；当前仅保留，尚无底盘执行端。 */
  float chassis_vy_m_s;       /* 底盘横移线速度，m/s；当前仅保留，尚无底盘执行端。 */
  float chassis_yaw_rad_s;    /* 底盘 yaw/小陀螺角速度，rad/s；VrA 或键鼠 Q/E。 */
  float gimbal_pitch_rad_s;   /* 云台 pitch 手操速度，rad/s；键鼠时供云台按帧积分。 */
  float gimbal_yaw_rad_s;     /* 云台 yaw 手操速度，rad/s；当前无 yaw 执行端。 */
  float leg_length_m;         /* 腿长目标，m；遥控 VrB 或键鼠 Z/X，尚无执行端。 */
  float vision_yaw_rad;       /* 视觉 yaw 绝对目标，rad；当前无 yaw 执行端。 */
  float vision_pitch_rad;     /* 视觉 pitch 绝对目标，rad；视觉模式有效时云台使用。 */
  uint8_t friction_wheel_on;  /* 0=停摩擦轮，1=固定相反方向转动。 */
  uint8_t vision_mode_on;     /* 0=手操，1=视觉自瞄；视觉帧超时会令命令无效。 */
  uint8_t spin_mode_enable;   /* 0=小陀螺关，1=小陀螺开；遥控 SwC 或键鼠 G 控制。 */
  uint8_t leg_mode_enable;    /* 与 total_enable 合并，0=腿模式关，1=腿模式开。 */
  uint8_t total_enable;       /* 0=安全停机，1=允许执行；遥控 SwD 控制，键鼠新鲜时为 1。 */
  uint8_t command_valid;      /* 0=下游必须零输出，1=来源、新鲜度和总使能均通过。 */
  uint8_t source;             /* Mode_ControlSource_t：REMOTE 或 KEYBOARD_MOUSE。 */
  uint8_t reserved;           /* 结构体对齐预留，必须置零。 */
  uint32_t update_tick;       /* 当前仲裁来源最近一帧有效数据的 HAL ms 时间戳。 */
  uint32_t publish_tick;      /* ModeTask 完成本帧仲裁并写入队列的 HAL ms 时间戳。 */
  uint32_t source_frame_count;/* 当前仲裁来源帧号；鼠标增量仅允许按新帧积分一次。 */
  uint32_t sequence;          /* ModeTask 每发布一帧递增，用于调试队列链路。 */
} Mode_Control_Command_t;

/* ModeTask -> 云台执行任务的最新统一命令邮箱。 */
extern osMessageQueueId_t Gimbal_Control_QueueHandle;

#endif
