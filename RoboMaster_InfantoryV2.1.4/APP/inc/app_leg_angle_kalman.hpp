#ifndef __APP_LEG_ANGLE_KALMAN_HPP__
#define __APP_LEG_ANGLE_KALMAN_HPP__

#include <stdint.h>

/* 两状态腿摆观测器：x = [phi0, dphi0]。过程模型采用恒角速度模型，
 * VMC 正运动学 phi0 与两髋电机速度平均值分别校正角度和角速度。 */
typedef struct {
    float x[2];
    float P[2][2];
    uint8_t initialized;
} LegAngleKalman_t;

/* 白噪声角加速度模型的标准差（rad/s^2）。 */
#define LEG_ANGLE_KF_ACCEL_STD_RAD_S2  40.0f
/* VMC 正运动学腿角观测标准差（rad）。 */
#define LEG_ANGLE_KF_PHI_STD_RAD        0.005f
/* 两髋电机速度平均观测标准差（rad/s）。静止抖动较大，故设得较大。 */
#define LEG_ANGLE_KF_DPHI_STD_RAD_S     5.000f

void LegAngleKalman_Reset(LegAngleKalman_t *kf, float phi0, float dphi0);
void LegAngleKalman_Update(LegAngleKalman_t *kf, float dt,
                           float phi0_measurement, float dphi0_measurement);

#endif
