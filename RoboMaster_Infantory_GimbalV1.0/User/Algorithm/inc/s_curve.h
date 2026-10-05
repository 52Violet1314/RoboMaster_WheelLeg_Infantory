/**
 ******************************************************************************
 * @file    s_curve.h
 * @brief   七段式 S 曲线运动规划库
 *
 * 语义说明(重要):
 *   本规划器是"绝对位置规划器"——SCurve_SetTarget 写入的是"最终期望位置",
 *   SCurve_Update 每周期推进后, s->pos 是从起点到终点的平滑轨迹当前位置,
 *   语义上等同于"平滑后的目标位置", 可直接喂给位置环做 reference.
 *
 *   典型用法(以 pitch IMU 角度为例):
 *     SCurve_SetTarget(&s, target_imu_pitch_rad);   // 绝对 IMU pitch 角
 *     SCurve_Update(&s, dt);
 *     angle_pid_target = s.pos;                    // 平滑后的 IMU pitch 角
 *
 * 限幅三段:
 *   v_max: 速度上限   (单位与 target 一致, 本项目 pitch 使用 rad/s)
 *   a_max: 加速度上限 (单位^2)
 *   j_max: 加加速度(jerk)上限 (单位^3)
 *
 * 七段含义:
 *   段1: jerk +j_max, 加速度从 0 升到 a_peak
 *   段2: jerk  0    , 加速度保持 a_peak (匀加速)
 *   段3: jerk -j_max, 加速度从 a_peak 降到 0 (达到 v_peak)
 *   段4: jerk  0    , 匀速 v_peak
 *   段5: jerk -j_max, 加速度从 0 降到 -a_peak
 *   段6: jerk  0    , 加速度保持 -a_peak (匀减速)
 *   段7: jerk +j_max, 加速度从 -a_peak 升到 0 (到达终点, 速度为 0)
 ******************************************************************************
 */
#ifndef S_CURVE_H
#define S_CURVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief S 曲线规划器状态机
 * @note  pos/target/last_target 单位由调用方约定，本项目 pitch 使用 rad
 */
typedef struct {
    /* === 输出 (调用方每周期读取) === */
    float pos;             /**< 当前规划位置(平滑轨迹点), 喂位置环 */
    float vel;             /**< 当前规划速度(平滑导数), 可用于前馈 */
    float accel;           /**< 当前规划加速度, 调试用 */

    /* === 输入 (调用方设置) === */
    float target;          /**< 期望最终位置 */
    float last_target;     /**< 上次重规划时的目标, 用于检测目标跳变 */
    float deadband;        /**< 死区, |target - last_target| < deadband 视为未变化 */
    float v_max;           /**< 速度上限 */
    float a_max;           /**< 加速度上限 */
    float j_max;           /**< 加加速度(jerk)上限 */

    /* === 内部状态 === */
    int   phase;           /**< 当前所处段号 1~7, 0=空闲(到达/未启动) */
    float t[8];            /**< 各段时长 t[1]~t[7], t[0] 未用 */
    int8_t jerk_sign[8];   /**< 各段 jerk 符号 (-1/0/+1), 实际 jerk = jerk_sign[i] * j_max */
    float t_now;           /**< 当前段已走过的时间 */
    float v_peak;          /**< 本次规划能达到的峰值速度 */
} SCurve_t;

/**
 * @brief 初始化为七段式规划器.
 * @param s     规划器对象
 * @param v_max 速度上限(>0)
 * @param a_max 加速度上限(>0)
 * @param j_max jerk 上限(>0)
 */
void SCurve_InitSeg7(SCurve_t *s, float v_max, float a_max, float j_max);

/**
 * @brief 写入新的最终期望位置.
 * @note  若与上次目标差超过 deadband, Update 时会触发在线重规划
 * @param s      规划器对象
 * @param target 期望位置(与 pos 同单位)
 */
void SCurve_SetTarget(SCurve_t *s, float target);

/**
 * @brief 推进一个控制周期.
 * @param s  规划器对象
 * @param dt 步长 s, 越界(<=0 或 >0.1) 时兜底为 1ms
 */
void SCurve_Update(SCurve_t *s, float dt);

/**
 * @brief 重置规划器到指定位置, 清零速度/加速度/段号.
 * @param s   规划器对象
 * @param pos  起点位置(通常为当前反馈角度)
 */
void SCurve_Reset(SCurve_t *s, float pos);

/**
 * @brief 设置目标跳变死区.
 * @note  本项目 pitch 死区使用 rad
 * @param s        规划器对象
 * @param deadband 死区, >0
 */
void SCurve_SetDeadband(SCurve_t *s, float deadband);

#ifdef __cplusplus
}
#endif

#endif /* S_CURVE_H */
