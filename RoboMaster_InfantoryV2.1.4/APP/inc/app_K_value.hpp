#ifndef __APP_K_VALUE_HPP__
#define __APP_K_VALUE_HPP__
#include <stdint.h>
#include "app_Data_Task.hpp"

typedef float K_Fixed_Leg_t[4][10];

typedef struct
{
    float leg_length_m;
    K_Fixed_Leg_t gain;
} LQR_K_Schedule_t;

#define LQR_K_SCHEDULE_COUNT         41U
#define LQR_K_SCHEDULE_MIN_LENGTH_M  0.150f
#define LQR_K_SCHEDULE_STEP_M        0.005f

extern const LQR_K_Schedule_t LQR_K_Schedule[LQR_K_SCHEDULE_COUNT];

/* 越界腿长钳到表端；表内按平均实际腿长对相邻 K 线性插值。 */
void LQR_K_GetInterpolated(float leg_length_m, K_Fixed_Leg_t output);

/* 固定腿长 K 的状态反馈计算；限幅仍由输出限幅模块在 VMC 映射后完成。 */
void LQR_K_CalculateWithGain(Classic_Data_t *pClassicData,
                             K_Fixed_Leg_t gain,
                             float theta_ref, float d_theta_ref,
                             float theta_l_ref, float d_theta_l_ref,
                             float theta_r_ref, float d_theta_r_ref);


#endif
