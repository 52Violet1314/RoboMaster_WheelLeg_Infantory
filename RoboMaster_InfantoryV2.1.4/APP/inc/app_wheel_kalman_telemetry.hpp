#ifndef __APP_WHEEL_KALMAN_TELEMETRY_HPP__
#define __APP_WHEEL_KALMAN_TELEMETRY_HPP__

/*
 * 固定 FireWater CSV 调试行：wheel_kalman_raw_pos（单位 m）。
 *
 * wheel_kalman_raw_pos 为左右卡尔曼 x[0] 的平均值：这是卡尔曼直接推算的
 * 原始位置，尚未减去 Pos_INT，也未经过 LQR 输入限幅。
 * 本模块只读取数据并输出，不参与控制。
 */
void WheelKalmanTelemetry_Print(void);

#endif
