/**
 * @file ESO.h
 * @brief Pitch 角速度扩张状态观测器。
 *
 * 模型：omega_dot = b*u + d，b0 约等于 1/J。
 *
 * 符号：
 *   omega : IMU 实测 Pitch 角速度(rad/s)
 *   u     : 作用在机构、按 IMU Pitch 正方向计的实际扭矩(Nm)
 *   J     : Pitch 轴等效转动惯量(kg*m^2)
 *   b=1/J : 真实输入增益；b0 是其用于观测器的估计值
 *   d     : 合并后的未知角加速度扰动(rad/s^2)
 *   z1/z2 : 分别为 omega 与 d 的估计值
 *
 * 连续观测器：
 *   z1_dot = b0*u + z2 + beta1*(omega-z1)
 *   z2_dot = beta2*(omega-z1)
 *
 * 前向欧拉离散：
 *   e        = omega - z1
 *   z1_next  = z1 + dt * (b0*u + z2 + beta1*e)
 *   z2_next  = z2 + dt * beta2*e
 * 带宽整定：beta1 = 2*omega_o，beta2 = omega_o^2。
 *
 * 扰动定义：d = -T_d/J。因此 z2 收敛后估计的是角加速度扰动；若后续需要
 * 换算扰动力矩，应由调用方采用 T_d_hat=-J*z2 完成并明确决定是否叠加。
 * 本模块只维护 z1/z2 观测状态；它不会自行向电机叠加任何前馈扭矩。
 */
#ifndef USER_ALGORITHM_ESO_H
#define USER_ALGORITHM_ESO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 异常 dt 的保护值(s)；正常值必须由调用方传入相邻有效 IMU 样本的真实间隔。
 * 通常无需调节，Pitch 控制周期为 1 ms。 */
#define ESO_DEFAULT_DT_S        (0.001f)
/* 可调：Pitch 轴折算到控制轴的等效惯量(kg*m^2)。修改后必须同步保证 b0≈1/J。 */
#define PITCH_AXIS_INERTIA_J    (0.003798723f)
/* 可调：ESO 输入增益估计，理想值为 1/J。一般不单独调，随 J 一起计算。 */
#define ESO_DEFAULT_B0          (1.0f / PITCH_AXIS_INERTIA_J)
/* 可调：观测器带宽(rad/s)。增大：z1/z2 跟踪更快但更易吃进振动；减小：更稳但
 * 对重力/阻力变化反应更慢。当前 30 为接入前馈的保守起点。 */
#define ESO_DEFAULT_OMEGA_O     (35.0f)
#define ESO_BETA1_FROM_OMEGA(w) (2.0f * (w))
#define ESO_BETA2_FROM_OMEGA(w) ((w) * (w))

/* 可调：z2 换算扭矩后的低通截止频率(Hz)。>0 启用一阶低通；0 旁路滤波。
 * 你的 IMU 已做滤波，当前设 0 保留算法但不再叠加第二层低通。 */
#define ESO_FEEDFORWARD_FILTER_HZ        (0.0f)
/* 可调：ESO 前馈绝对上限(Nm)。无论 z2 多大，额外补偿不会超过 ±此值。 */
#define ESO_FEEDFORWARD_LIMIT_NM         (0.2f)
/* 可调：前馈最大变化率(Nm/s)。1 ms 控制周期下 40 等于每拍最多改变 0.04 Nm。
 * 调小更柔和，调大响应更快但更可能把扰动带入电机。 */
#define ESO_FEEDFORWARD_SLEW_LIMIT_NM_S  (40.0f)

typedef struct {
    float omega;       /* 实测 IMU Pitch 角速度 rad/s */
    float u_cmd;       /* 上一拍实际成功发送的等效轴扭矩 Nm */
    float z1;          /* omega 估计 rad/s */
    float z2;          /* 总扰动估计 rad/s^2 */
    float beta1;       /* 2*omega_o */
    float beta2;       /* omega_o^2 */
    float b0;          /* 输入增益估计，约为 1/J */
    float J;           /* 等效转动惯量 kg*m^2 */
    float ff_filtered_nm; /* 低通后的扰动力矩候选值 */
    float ff_nm;          /* 限幅、限速后的实际前馈扭矩 */
    uint8_t initialized;
} ESO_t;

/** 初始化参数与观测状态。 */
void ESO_Init(ESO_t *eso, float J, float b0, float omega_o);
/** 运行时更新 J、b0、带宽；带宽会同步重算 beta1/beta2。 */
void ESO_SetParams(ESO_t *eso, float J, float b0, float omega_o);
/**
 * 用本拍 omega 及上一拍已实际发送的 u_cmd 做一次前向欧拉离散更新。
 * 调用顺序必须是：先写 omega/u_cmd，再调用本函数。u_cmd 不能使用尚未
 * 发出的本拍新扭矩，否则会把未来输入错误地解释为已作用在机构上。
 */
void ESO_Update(ESO_t *eso, float dt);
/** 返回已经低通、限幅、限速后的总扰动补偿扭矩(Nm)。 */
float ESO_GetFeedforward(const ESO_t *eso);
/** 模式切换、失能或重使能时清空 z1/z2，防止旧观测状态带入。 */
void ESO_Reset(ESO_t *eso);

#ifdef __cplusplus
}
#endif

#endif
