/**
 * @file    wheel_kalman_fliter.c
 * @brief   轮式里程计卡尔曼滤波器实现
 *
 * 状态向量: x[2] = [车体位置, 车体速度]
 * 控制输入: u = 加速度 (m/s²)
 * 测量向量: z[2] = [运动学积分位置, 运动学合成速度]
 *
 * 测量值在函数内部由运动学模型自动计算：
 *   z[0] = 速度积分累积位置（每周期 body_vel_real * dt 累加）
 *   z[1] = 轮速 + 陀螺仪 + 腿长摆动补偿后的车体质心速度
 *
 * 卡尔曼滤波流程（五步标准公式）：
 *   第一步: 状态预测      x̂ₖ₋ = A·x̂ₖ₋₁ + B·u
 *   第二步: 先验协方差    Pₖ₋  = A·Pₖ₋₁·Aᵀ + Q
 *   第三步: 新息          y    = z - H·x̂ₖ₋
 *   第四步: 卡尔曼增益    K    = Pₖ₋·Hᵀ·(H·Pₖ₋·Hᵀ + R)⁻¹
 *   第五步: 状态更新      x̂ₖ   = x̂ₖ₋ + K·y
 *   第六步: 协方差更新    Pₖ   = (I - K·H)·Pₖ₋
 */

#include "wheel_kalman_fliter.h"
#include "arm_math.h"
#include "bsp_UART.h"


WheelKalmanFliter_t Wheel_Kalman_L;
WheelKalmanFliter_t Wheel_Kalman_R;

/**
 * @brief 初始化卡尔曼滤波器
 *
 * @param kf        滤波器实例
 * @param dt        采样周期 (s)，通常 1ms
 * @param init_pos  初始位置 (m)
 * @param init_vel  初始速度 (m/s)
 */
void WheelKalman_Init(WheelKalmanFliter_t *kf, float dt, float init_pos, float init_vel)
{
    /* 保存采样周期 */
    kf->dt = dt;

    /* 初始状态 */
    kf->x[0] = init_pos;
    kf->x[1] = init_vel;

    /* 误差协方差矩阵 P₀ — 初始置信度设为 1（单位阵） */
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;

    /* 状态转移矩阵 A — 匀速模型：位置 += 速度 * dt*/
    kf->A[0][0] = 1.0f; kf->A[0][1] = dt;
    kf->A[1][0] = 0.0f; kf->A[1][1] = 1.0f;

    /* 控制输入矩阵 B — 加速度 dt 对位置/速度的影响 */
    kf->B[0][0] = 0.5f * dt * dt;  /* 位置:   ½·a·t² */
    kf->B[1][0] = dt;              /* 速度:   a·t    */

    /* 观测矩阵 H — 恒等矩阵（直接观测位置和速度） */
    kf->H[0][0] = 1.0f; kf->H[0][1] = 0.0f;
    kf->H[1][0] = 0.0f; kf->H[1][1] = 1.0f;

    /* 过程噪声协方差 Q — 模型不确定性
     * Q[0][0] 较小 -> 对位置模型比较信任
     * Q[1][1] 较大 -> 速度受扰动较多 */
    kf->Q[0][0] = 0.1f; kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f; kf->Q[1][1] = 0.1f;

    /* 测量噪声协方差 R — 传感器不确定性
     * R[1][1] > R[0][0] -> 更信任位置测量、较少信任速度测量 */
    kf->R[0][0] = 1.0f; kf->R[0][1] = 0.0f;
    kf->R[1][0] = 0.0f; kf->R[1][1] = 1.0f;

    /* 测量向量清零 */
    kf->z[0] = 0.0f;
    kf->z[1] = 0.0f;
}


/**
 * @brief 执行一次卡尔曼滤波更新（预测 + 校正）
 *
 * @param kf        滤波器实例
 * @param u         控制输入 / 加速度 (m/s²)
 * @param vel       电机关节速度（预留，当前未使用）
 * @param dt        本轮实际时间步长 (s)
 * @param phi       腿摆角 phi (rad)
 * @param dot_phi   腿摆角速率 d(phi)/dt (rad/s)
 * @param L         腿长 L (m)
 * @param dot_L     腿长变化率 dL/dt (m/s)
 * @param gy        车体陀螺仪角速度 (rad/s)
 * @param wheel_vel 轮子电机角速度 (rad/s)，正方向为前进
 */
void WheelKalman_Update(WheelKalmanFliter_t *kf,
                        float u,
                        float dt,
                        float phi, float dot_phi,
                        float L, float dot_L,
                        float gy,
                        float wheel_vel)
{
    // ================================================================
    // 第〇步：构造测量向量 z（运动学模型合成）
    // ================================================================

    /* 真实轮速 = (陀螺角速度 + 电机轮速) × 轮子半径
     * gy 补偿了车体自身旋转对轮速读数的影响 */
    float wheel_vel_real = (gy + wheel_vel) * WHEEL_RADIUS;

    /* 车体质心速度 = 轮速 + 腿摆动线速度分量 + 腿伸缩线速度分量
     *   L·φ̇·cos(φ)  — 腿绕关节转动的切向速度（cos 投影到水平方向）
     *   L̇·sin(φ)     — 腿伸缩带来的径向速度（sin 投影到水平方向）*/
    float body_vel_real = wheel_vel_real
                        + L * dot_phi * arm_cos_f32(phi)
                        + dot_L * arm_sin_f32(phi);

    /* z[0] = 速度积分累积位置（以运动学模型为基准）*/
    kf->z[0] += body_vel_real * dt;

    /* z[1] = 运动学合成车体速度 */
    kf->z[1] = body_vel_real;

    // ================================================================
    // 更新 A、B 矩阵参数（dt 可能每周期变化）
    // ================================================================

    /* 状态转移矩阵 A — 匀速模型 */
    kf->A[0][0] = 1.0f; kf->A[0][1] = dt;
    kf->A[1][0] = 0.0f; kf->A[1][1] = 1.0f;

    /* 控制输入矩阵 B */
    kf->B[0][0] = 0.5f * dt * dt;
    kf->B[1][0] = dt;

    // ================================================================
    // 将 C 数组包装为 CMSIS-DSP 矩阵对象（零拷贝，直接引用原始数据）
    // ================================================================

    /* 模型矩阵 */
    arm_matrix_instance_f32 mat_A = {2, 2, (float *)kf->A};
    arm_matrix_instance_f32 mat_B = {2, 1, (float *)kf->B};
    arm_matrix_instance_f32 mat_H = {2, 2, (float *)kf->H};
    arm_matrix_instance_f32 mat_Q = {2, 2, (float *)kf->Q};

    /* 状态矩阵 */
    arm_matrix_instance_f32 mat_x = {2, 1, kf->x};
    arm_matrix_instance_f32 mat_P = {2, 2, (float *)kf->P};

    /* 控制向量（标量加速度，包装为 1×1 矩阵）*/
    arm_matrix_instance_f32 mat_u = {1, 1, &u};

    /* 测量向量 — 使用结构体内部的 z */
    arm_matrix_instance_f32 mat_z = {2, 1, kf->z};

    /* 临时缓冲区 — 复用以减少栈开销 */
    float temp_2x1_a[2], temp_2x1_b[2];
    float temp_2x2_a[4], temp_2x2_b[4], temp_2x2_c[4];

    // ================================================================
    // 第一步：状态预测
    //   x̂ₖ₋ = A·x̂ₖ₋₁ + B·u
    // ================================================================
    arm_matrix_instance_f32 mat_Ax = {2, 1, temp_2x1_a};   /* A·x̂ */
    arm_matrix_instance_f32 mat_Bu = {2, 1, temp_2x1_b};   /* B·u  */

    arm_mat_mult_f32(&mat_A, &mat_x, &mat_Ax);             /* A·x̂     */
    arm_mat_mult_f32(&mat_B, &mat_u, &mat_Bu);             /* B·u      */
    arm_mat_add_f32(&mat_Ax, &mat_Bu, &mat_x);             /* x̂ = 上式之和，此时 mat_x 为先验估计 */

    // ================================================================
    // 第二步：先验误差协方差更新
    //   Pₖ₋ = A·Pₖ₋₁·Aᵀ + Q
    // ================================================================
    arm_matrix_instance_f32 mat_AT    = {2, 2, temp_2x2_a};   /* Aᵀ        */
    arm_matrix_instance_f32 mat_P_AT  = {2, 2, temp_2x2_b};   /* P·Aᵀ      */
    arm_matrix_instance_f32 mat_AP_AT = {2, 2, temp_2x2_c};   /* A·P·Aᵀ    */

    arm_mat_trans_f32(&mat_A, &mat_AT);                       /* Aᵀ        */
    arm_mat_mult_f32(&mat_P, &mat_AT, &mat_P_AT);             /* P·Aᵀ      */
    arm_mat_mult_f32(&mat_A, &mat_P_AT, &mat_AP_AT);          /* A·P·Aᵀ    */
    arm_mat_add_f32(&mat_AP_AT, &mat_Q, &mat_P);              /* + Q，此时 mat_P 为先验协方差 */

    // ================================================================
    // 第三步：计算新息（测量残差）
    //   y = z - H·x̂ₖ₋
    // ================================================================
    arm_matrix_instance_f32 mat_Hx = {2, 1, temp_2x1_a};      /* H·x̂ₖ₋  */
    arm_matrix_instance_f32 mat_y  = {2, 1, temp_2x1_b};      /* 新息 y  */

    arm_mat_mult_f32(&mat_H, &mat_x, &mat_Hx);                /* H·x̂ₖ₋   */
    arm_mat_sub_f32(&mat_z, &mat_Hx, &mat_y);                 /* y = 测量 - 预测 */

    // ================================================================
    // 第四步：计算卡尔曼增益（2×2 手动求逆，避免 CMSIS-DSP 精度问题）
    //   S = Pₖ₋ + R            （H = I 时简化）
    //   K = Pₖ₋ · S⁻¹          （H = I 时简化）
    // ================================================================

    /* 直接计算 S = P + R（H = I 恒等矩阵，省略 H·P·Hᵀ 变换）*/
    float s00 = kf->P[0][0] + kf->R[0][0];
    float s01 = kf->P[0][1] + kf->R[0][1];
    float s10 = kf->P[1][0] + kf->R[1][0];
    float s11 = kf->P[1][1] + kf->R[1][1];

    /* 2×2 手动求逆: det = s00·s11 - s01·s10 */
    float det = s00 * s11 - s01 * s10;
    if (fabsf(det) < 1e-12f) {
        /* 奇异矩阵：跳过本周期更新，保留先验估计 */
        return;
    }
    float inv_det = 1.0f / det;

    /* S⁻¹ = [s11 -s01; -s10 s00] / det */
    float k00 = (kf->P[0][0] * s11 + kf->P[0][1] * (-s10)) * inv_det;
    float k01 = (kf->P[0][0] * (-s01) + kf->P[0][1] * s00) * inv_det;
    float k10 = (kf->P[1][0] * s11 + kf->P[1][1] * (-s10)) * inv_det;
    float k11 = (kf->P[1][0] * (-s01) + kf->P[1][1] * s00) * inv_det;

    /* 新息向量（mat_y 已在第三步写入 temp_2x1_b）*/
    float y0 = temp_2x1_b[0];
    float y1 = temp_2x1_b[1];

    // ================================================================
    // 第五步：状态更新（后验估计）
    //   x̂ₖ = x̂ₖ₋ + K·y
    // ================================================================
    float x_old0 = kf->x[0];
    float x_old1 = kf->x[1];
    kf->x[0] = x_old0 + k00 * y0 + k01 * y1;
    kf->x[1] = x_old1 + k10 * y0 + k11 * y1;

    // ================================================================
    // 第六步：协方差更新（后验协方差）
    //   Pₖ = (I - K·H)·Pₖ₋ = (I - K)·Pₖ₋    （H = I）
    // ================================================================
    float ik00 = 1.0f - k00;
    float ik01 = -k01;
    float ik10 = -k10;
    float ik11 = 1.0f - k11;

    float p00 = kf->P[0][0], p01 = kf->P[0][1];
    float p10 = kf->P[1][0], p11 = kf->P[1][1];

    kf->P[0][0] = ik00 * p00 + ik01 * p10;
    kf->P[0][1] = ik00 * p01 + ik01 * p11;
    kf->P[1][0] = ik10 * p00 + ik11 * p10;
    kf->P[1][1] = ik10 * p01 + ik11 * p11;
}

/* ========== 调试打印 ========== */

#define WK_PRINT_SCALE 10000.0f  /* 精度 0.1mm / 0.1mm/s */

void WheelKalman_Print(const WheelKalmanFliter_t *kf, char side)
{
    uart_print("%d,%d,%d,%d\r\n",
               (int)(kf->z[0] * WK_PRINT_SCALE),
               (int)(kf->z[1] * WK_PRINT_SCALE),
               (int)(kf->x[0] * WK_PRINT_SCALE),
               (int)(kf->x[1] * WK_PRINT_SCALE));
}
