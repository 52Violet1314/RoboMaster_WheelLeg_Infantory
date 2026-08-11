#ifndef __WHEEL_KALMAN_FLITER_H__
#define __WHEEL_KALMAN_FLITER_H__

#include <stdint.h>

#define WHEEL_RADIUS 0.05f   // 轮子半径（单位：米）

/**
 * @brief 轮式里程计卡尔曼滤波器
 *
 * 状态向量 x[2]:
 *   x[0] — 车体位置 (m)
 *   x[1] — 车体速度 (m/s)
 *
 * 控制输入 u:
 *   加速度 (m/s²)，由期望速度差分或遥控器指令给出
 *
 * 测量向量 z[2]:
 *   z[0] — 车体位置（运动学积分累积）
 *   z[1] — 车体速度（轮速 + 陀螺 + 腿运动学合成）
 */
typedef struct {
    float x[2];        // 状态: [位置, 速度]
    float x_head[2];   // 状态备份（预留，当前未使用）
    float P[2][2];     // 误差协方差矩阵
    float A[2][2];     // 状态转移矩阵（每周期根据 dt 更新）
    float B[2][1];     // 控制输入矩阵（每周期根据 dt 更新）
    float H[2][2];     // 观测矩阵（恒等矩阵，直接观测位置和速度）
    float Q[2][2];     // 过程噪声协方差（模型不确定性）
    float R[2][2];     // 测量噪声协方差（传感器不确定性）
    float z[2];        // 测量向量，内部自动计算
    float dt;          // 采样周期 (s)
} WheelKalmanFliter_t;

#ifdef __cplusplus
extern "C" {
#endif

extern WheelKalmanFliter_t Wheel_Kalman_L;
extern WheelKalmanFliter_t Wheel_Kalman_R;

/**
 * @brief 初始化卡尔曼滤波器
 * @param kf        滤波器实例指针
 * @param dt        采样周期 (s)
 * @param init_pos  初始位置 (m)
 * @param init_vel  初始速度 (m/s)
 */
void WheelKalman_Init(WheelKalmanFliter_t *kf, float dt, float init_pos, float init_vel);

/**
 * @brief 执行一次卡尔曼滤波更新（预测 + 校正）
 *
 * 测量值 z 在函数内部由运动学模型自动计算：
 *   z[0] = 速度积分累积位置
 *   z[1] = 轮速 + 陀螺仪 + 腿长变化合成车体速度
 *
 * @param kf        滤波器实例指针
 * @param u         控制输入 / 加速度 (m/s²)
 * @param pos       电机关节位置（预留）
 * @param vel       电机关节速度（预留）
 * @param dt        本轮实际时间步长 (s)
 * @param phi       腿摆角 phi (rad)
 * @param dot_phi   腿摆角速率 d(phi)/dt (rad/s)
 * @param L         腿长 L (m)
 * @param dot_L     腿长变化率 dL/dt (m/s)
 * @param gy        车体陀螺仪角速度 (rad/s)
 * @param wheel_vel 轮子电机角速度 (rad/s)，正方向定义为前进方向
 */
void WheelKalman_Update(WheelKalmanFliter_t *kf,
                        float u,
                        float dt,
                        float phi, float dot_phi,
                        float L, float dot_L,
                        float gy,
                        float wheel_vel);

/**
 * @brief 打印卡尔曼滤波器状态（浮点值 x10000 缩放为带符号整数）
 *
 * 输出格式: "L z_p z_v x_p x_v\r\n" 或 "R z_p z_v x_p x_v\r\n"
 *   - z_p / z_v: 测量向量 z[0]位置, z[1]速度
 *   - x_p / x_v: 状态向量 x[0]位置, x[1]速度
 *   - 所有值乘以 10000 后转 int 输出，保留 4 位小数精度
 *
 * @param kf   滤波器实例指针
 * @param side 侧标识，'L' 或 'R'
 */
void WheelKalman_Print(const WheelKalmanFliter_t *kf, char side);

#ifdef __cplusplus
}
#endif

#endif
