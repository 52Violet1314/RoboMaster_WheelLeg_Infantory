#ifndef __APP_VMC_HPP__
#define __APP_VMC_HPP__

#include "app_Data_Task.hpp"

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

#endif
