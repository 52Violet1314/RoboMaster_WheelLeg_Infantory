#ifndef __APP_VMC_HPP__
#define __APP_VMC_HPP__

#include "app_Data_Task.hpp"

/* VMC 的最终髋关节力矩保护，单位 Nm，与 8009 额定输出扭矩一致。 */
#define VMC_JOINT_TORQUE_LIMIT_NM    20.0f

typedef struct {
    float phi1;
    float phi4;
    float phi2;
    float phi3;
    float L0;
    float phi0;
} VMC_LegKinematics_t;

typedef struct {
    VMC_LegKinematics_t left;
    VMC_LegKinematics_t right;
} VMC_Kinematics_t;

/* 本轮电机反馈已写入 Classic_Data 后调用：只更新腿反馈，不写输出力矩。 */
void VMC_UpdateLegFeedback(Classic_Data_t *data,
                           VMC_Kinematics_t *kinematics, float dt);

/* F0/Tp 已完成全部补偿与控制后调用：只映射为四路髋关节力矩。 */
void VMC_MapForceToJointTorques(Classic_Data_t *data,
                                const VMC_Kinematics_t *kinematics,
                                float F0L, float F0R, float TpL, float TpR);

/* VMC 前：按当前雅可比、F0 与重力补偿限制 LQR Tp，使两髋关节均不超额定。
 * 重力补偿作为总虚拟腿摆力矩的一部分参与余量计算。 */
void VMC_LimitVirtualPitchTorqueByJointCapacity(
        Classic_Data_t *data, const VMC_Kinematics_t *kinematics,
        float F0L, float F0R, float gravity_tp_l, float gravity_tp_r);

/* VMC 后：限制四路实际髋关节目标扭矩 T1/T2。 */
void VMC_LimitJointTorques(Classic_Data_t *data);

#endif
