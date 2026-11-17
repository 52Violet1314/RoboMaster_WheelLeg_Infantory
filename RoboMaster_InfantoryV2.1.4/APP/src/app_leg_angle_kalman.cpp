#include "app_leg_angle_kalman.hpp"

#include <math.h>

static float LegAngleKalman_WrapPi(float angle) {
    while (angle > 3.1415926f) angle -= 6.2831853f;
    while (angle < -3.1415926f) angle += 6.2831853f;
    return angle;
}

void LegAngleKalman_Reset(LegAngleKalman_t *kf, float phi0, float dphi0) {
    if (kf == nullptr) return;

    const float r_phi = LEG_ANGLE_KF_PHI_STD_RAD * LEG_ANGLE_KF_PHI_STD_RAD;
    const float r_dphi = LEG_ANGLE_KF_DPHI_STD_RAD_S * LEG_ANGLE_KF_DPHI_STD_RAD_S;
    kf->x[0] = phi0;
    kf->x[1] = dphi0;
    kf->P[0][0] = r_phi;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = r_dphi;
    kf->initialized = 1U;
}

void LegAngleKalman_Update(LegAngleKalman_t *kf, float dt,
                           float phi0_measurement, float dphi0_measurement) {
    if (kf == nullptr || dt <= 0.0f) return;
    if (kf->initialized == 0U) {
        LegAngleKalman_Reset(kf, phi0_measurement, dphi0_measurement);
        return;
    }

    /* 预测：x- = A*x，A = [[1, dt], [0, 1]]。Q 来自白噪声角加速度模型。 */
    const float sigma_a2 = LEG_ANGLE_KF_ACCEL_STD_RAD_S2 *
                           LEG_ANGLE_KF_ACCEL_STD_RAD_S2;
    const float dt2 = dt * dt;
    const float q00 = 0.25f * dt2 * dt2 * sigma_a2;
    const float q01 = 0.5f * dt2 * dt * sigma_a2;
    const float q11 = dt2 * sigma_a2;
    const float x0_prior = kf->x[0] + dt * kf->x[1];
    const float x1_prior = kf->x[1];
    const float p00_prior = kf->P[0][0] + dt * (kf->P[0][1] + kf->P[1][0]) +
                            dt2 * kf->P[1][1] + q00;
    const float p01_prior = kf->P[0][1] + dt * kf->P[1][1] + q01;
    const float p10_prior = kf->P[1][0] + dt * kf->P[1][1] + q01;
    const float p11_prior = kf->P[1][1] + q11;

    /* 观测 z=[phi0_VMC, dphi0_motor]，H=I。 */
    const float r00 = LEG_ANGLE_KF_PHI_STD_RAD * LEG_ANGLE_KF_PHI_STD_RAD;
    const float r11 = LEG_ANGLE_KF_DPHI_STD_RAD_S * LEG_ANGLE_KF_DPHI_STD_RAD_S;
    const float s00 = p00_prior + r00;
    const float s01 = p01_prior;
    const float s10 = p10_prior;
    const float s11 = p11_prior + r11;
    const float det = s00 * s11 - s01 * s10;
    if (fabsf(det) < 1e-12f) return;

    const float inv_det = 1.0f / det;
    const float k00 = (p00_prior * s11 - p01_prior * s10) * inv_det;
    const float k01 = (-p00_prior * s01 + p01_prior * s00) * inv_det;
    const float k10 = (p10_prior * s11 - p11_prior * s10) * inv_det;
    const float k11 = (-p10_prior * s01 + p11_prior * s00) * inv_det;
    const float innovation_phi = LegAngleKalman_WrapPi(phi0_measurement - x0_prior);
    const float innovation_dphi = dphi0_measurement - x1_prior;

    kf->x[0] = x0_prior + k00 * innovation_phi + k01 * innovation_dphi;
    kf->x[1] = x1_prior + k10 * innovation_phi + k11 * innovation_dphi;

    /* Joseph 形式协方差更新，保持 float32 下的对称与半正定性。 */
    const float m00 = 1.0f - k00;
    const float m01 = -k01;
    const float m10 = -k10;
    const float m11 = 1.0f - k11;
    const float mp00 = m00 * p00_prior + m01 * p10_prior;
    const float mp01 = m00 * p01_prior + m01 * p11_prior;
    const float mp10 = m10 * p00_prior + m11 * p10_prior;
    const float mp11 = m10 * p01_prior + m11 * p11_prior;
    kf->P[0][0] = mp00 * m00 + mp01 * m01 + k00 * k00 * r00 + k01 * k01 * r11;
    kf->P[0][1] = mp00 * m10 + mp01 * m11 + k00 * k10 * r00 + k01 * k11 * r11;
    kf->P[1][0] = mp10 * m00 + mp11 * m01 + k10 * k00 * r00 + k11 * k01 * r11;
    kf->P[1][1] = mp10 * m10 + mp11 * m11 + k10 * k10 * r00 + k11 * k11 * r11;
    const float p01_symmetric = 0.5f * (kf->P[0][1] + kf->P[1][0]);
    kf->P[0][1] = p01_symmetric;
    kf->P[1][0] = p01_symmetric;
}
