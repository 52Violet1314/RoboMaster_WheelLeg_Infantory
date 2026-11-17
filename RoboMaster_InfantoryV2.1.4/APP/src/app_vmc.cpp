#include "app_vmc.hpp"
#include "app_leg_angle_kalman.hpp"
#include "arm_math.h"
#include <math.h>

#define VMC_FEEDBACK_FILTER_A 0.2f

static float VMC_ClampSymmetric(float value, float limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

/* VMC 虚拟腿正运动学。phi0 = atan2(yc, xc) 留在车身坐标约定中：
 * 后摆为正。此处不引入 IMU pitch，故 phi0 不是地面系腿角；到
 * CalculateTask.cpp 才与 IMU pitch 合成为地面系 theta。 */
static VMC_LegKinematics_t VMC_ForwardLeg(float phi1, float phi4) {
    constexpr float l1 = 0.210f;
    constexpr float l2 = 0.250f;
    constexpr float l3 = 0.250f;
    constexpr float l4 = 0.210f;
    VMC_LegKinematics_t leg = {};
    leg.phi1 = phi1;
    leg.phi4 = phi4;
    const float yd = l4 * arm_sin_f32(phi4);
    const float yb = l1 * arm_sin_f32(phi1);
    const float xd = l4 * arm_cos_f32(phi4);
    const float xb = l1 * arm_cos_f32(phi1);
    const float dx = xd - xb;
    const float dy = yd - yb;
    float lbd;
    arm_sqrt_f32(dx * dx + dy * dy, &lbd);
    const float a0 = 2.0f * l2 * dx;
    const float b0 = 2.0f * l2 * dy;
    const float c0 = l2 * l2 + lbd * lbd - l3 * l3;
    float root;
    arm_sqrt_f32(a0 * a0 + b0 * b0 - c0 * c0, &root);
    leg.phi2 = 2.0f * atan2f(b0 + root, a0 + c0);
    leg.phi3 = atan2f(yb - yd + l2 * arm_sin_f32(leg.phi2),
                      xb - xd + l2 * arm_cos_f32(leg.phi2));
    const float xc = l1 * arm_cos_f32(phi1) + l2 * arm_cos_f32(leg.phi2);
    const float yc = l1 * arm_sin_f32(phi1) + l2 * arm_sin_f32(leg.phi2);
    arm_sqrt_f32(xc * xc + yc * yc, &leg.L0);
    leg.phi0 = atan2f(yc, xc);
    return leg;
}

void VMC_UpdateLegFeedback(Classic_Data_t *data,
                           VMC_Kinematics_t *kinematics, float dt) {
    const float inv_dt = 1.0f / dt;
    kinematics->left = VMC_ForwardLeg(data->Motor_Data.L1_Motor_POS,
                                      data->Motor_Data.L2_Motor_POS);
    kinematics->right = VMC_ForwardLeg(data->Motor_Data.R1_Motor_POS,
                                       data->Motor_Data.R2_Motor_POS);
    VMC_LegKinematics_t *legs[2] = {&kinematics->left, &kinematics->right};
    float *l0[2] = {&data->Leg_Data.L0_L, &data->Leg_Data.L0_R};
    float *phi0[2] = {&data->Leg_Data.phi_0_L, &data->Leg_Data.phi_0_R};
    float *dl0[2] = {&data->Leg_Data.d_L0_L, &data->Leg_Data.d_L0_R};
    float *ddl0[2] = {&data->Leg_Data.dd_L0_L, &data->Leg_Data.dd_L0_R};
    float *last_dl0[2] = {&data->Leg_Data.d_last_L0_L, &data->Leg_Data.d_last_L0_R};
    float *dphi0[2] = {&data->Leg_Data.delta_phi_0_L, &data->Leg_Data.delta_phi_0_R};
    static float last_l0[2] = {0.0f, 0.0f};
    static LegAngleKalman_t leg_angle_kf[2] = {};

    for (uint32_t side = 0; side < 2U; ++side) {
        *l0[side] = legs[side]->L0;
        *phi0[side] = legs[side]->phi0;
        const float raw_dl0 = (legs[side]->L0 - last_l0[side]) * inv_dt;
        *dl0[side] = VMC_FEEDBACK_FILTER_A * raw_dl0 +
                      (1.0f - VMC_FEEDBACK_FILTER_A) * *dl0[side];
        const float raw_ddl0 = (*dl0[side] - *last_dl0[side]) * inv_dt;
        *ddl0[side] = VMC_FEEDBACK_FILTER_A * raw_ddl0 +
                       (1.0f - VMC_FEEDBACK_FILTER_A) * *ddl0[side];
        *last_dl0[side] = *dl0[side];
        last_l0[side] = legs[side]->L0;
    }

    /* 对称五连杆（l1=l4、l2=l3）有 phi0=(phi1+phi4)/2，因此
     * dphi0=(dphi1+dphi4)/2。左侧电机反馈已是 VMC 角度方向；右侧
     * 位置映射为负，故速度也同步镜像取负。电机速度平均值作为
     * dphi0 的观测，VMC phi0 作为角度观测，经独立卡尔曼输出 dphi0。 */
    const float motor_dphi0[2] = {
        0.5f * (data->Motor_Data.L1_Motor_VEL + data->Motor_Data.L2_Motor_VEL),
        -0.5f * (data->Motor_Data.R1_Motor_VEL + data->Motor_Data.R2_Motor_VEL),
    };
    data->Leg_Data.raw_delta_phi_0_L = motor_dphi0[0];
    data->Leg_Data.raw_delta_phi_0_R = motor_dphi0[1];
    for (uint32_t side = 0; side < 2U; ++side) {
        LegAngleKalman_Update(&leg_angle_kf[side], dt, *phi0[side], motor_dphi0[side]);
        *dphi0[side] = leg_angle_kf[side].x[1];
    }
}

void VMC_MapForceToJointTorques(Classic_Data_t *data,
                                const VMC_Kinematics_t *kinematics,
                                float F0L, float F0R, float TpL, float TpR) {
    constexpr float l1 = 0.210f;
    constexpr float l4 = 0.210f;
    const VMC_LegKinematics_t *legs[2] = {&kinematics->left, &kinematics->right};
    const float f0[2] = {F0L, F0R};
    const float tp[2] = {TpL, TpR};
    float *t1[2] = {&data->Contronller_Data.T1_L, &data->Contronller_Data.T1_R};
    float *t2[2] = {&data->Contronller_Data.T2_L, &data->Contronller_Data.T2_R};
    for (uint32_t side = 0; side < 2U; ++side) {
        const VMC_LegKinematics_t *leg = legs[side];
        const float sin32 = arm_sin_f32(leg->phi3 - leg->phi2);
        const float sin12 = arm_sin_f32(leg->phi1 - leg->phi2);
        const float sin03 = arm_sin_f32(leg->phi0 - leg->phi3);
        const float cos03 = arm_cos_f32(leg->phi0 - leg->phi3);
        const float sin02 = arm_sin_f32(leg->phi0 - leg->phi2);
        const float cos02 = arm_cos_f32(leg->phi0 - leg->phi2);
        const float sin34 = arm_sin_f32(leg->phi3 - leg->phi4);
        const float j11 = l1 * sin03 * sin12 / sin32;
        const float j12 = l1 * cos03 * sin12 / (leg->L0 * sin32);
        const float j21 = l4 * sin02 * sin34 / sin32;
        const float j22 = l4 * cos02 * sin34 / (leg->L0 * sin32);
        *t1[side] = j11 * f0[side] + j12 * tp[side];
        *t2[side] = j21 * f0[side] + j22 * tp[side];
    }
    data->Contronller_Data.Tp_L = TpL;
    data->Contronller_Data.Tp_R = TpR;
}

void VMC_LimitVirtualPitchTorque(Classic_Data_t *data, float limit_nm) {
    data->Contronller_Data.Tp_L = VMC_ClampSymmetric(
            data->Contronller_Data.Tp_L, limit_nm);
    data->Contronller_Data.Tp_R = VMC_ClampSymmetric(
            data->Contronller_Data.Tp_R, limit_nm);
}

void VMC_LimitJointTorques(Classic_Data_t *data) {
    data->Contronller_Data.T1_L = VMC_ClampSymmetric(
            data->Contronller_Data.T1_L, VMC_JOINT_TORQUE_LIMIT_NM);
    data->Contronller_Data.T2_L = VMC_ClampSymmetric(
            data->Contronller_Data.T2_L, VMC_JOINT_TORQUE_LIMIT_NM);
    data->Contronller_Data.T1_R = VMC_ClampSymmetric(
            data->Contronller_Data.T1_R, VMC_JOINT_TORQUE_LIMIT_NM);
    data->Contronller_Data.T2_R = VMC_ClampSymmetric(
            data->Contronller_Data.T2_R, VMC_JOINT_TORQUE_LIMIT_NM);
}
