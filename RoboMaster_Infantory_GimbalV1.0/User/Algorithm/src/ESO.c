#include "ESO.h"
#include <stddef.h>

#define ESO_TWO_PI (6.28318530717958647692f)

static float eso_clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void ESO_Init(ESO_t *eso, float J, float b0, float omega_o)
{
    if (eso == NULL) return;

    /* J、b0、带宽非法时回落到明确的默认值，避免 1/J 或增益发散。 */
    eso->J = (J > 0.0f) ? J : PITCH_AXIS_INERTIA_J;
    eso->b0 = (b0 > 0.0f) ? b0 : ESO_DEFAULT_B0;
    const float bandwidth = (omega_o > 0.0f) ? omega_o : ESO_DEFAULT_OMEGA_O;
    eso->beta1 = ESO_BETA1_FROM_OMEGA(bandwidth);
    eso->beta2 = ESO_BETA2_FROM_OMEGA(bandwidth);
    eso->omega = 0.0f;
    eso->u_cmd = 0.0f;
    eso->z1 = 0.0f;
    eso->z2 = 0.0f;
    eso->ff_filtered_nm = 0.0f;
    eso->ff_nm = 0.0f;
    eso->initialized = 1U;
}

void ESO_SetParams(ESO_t *eso, float J, float b0, float omega_o)
{
    if (eso == NULL) return;
    if (J > 0.0f) eso->J = J;
    if (b0 > 0.0f) eso->b0 = b0;
    if (omega_o > 0.0f) {
        eso->beta1 = ESO_BETA1_FROM_OMEGA(omega_o);
        eso->beta2 = ESO_BETA2_FROM_OMEGA(omega_o);
    }
}

void ESO_Update(ESO_t *eso, float dt)
{
    if (eso == NULL || eso->initialized == 0U) return;
    /* dt 仅保护异常输入；不要用固定周期替代正常的实际 IMU 时间间隔。 */
    if (dt <= 0.0f || dt > 0.1f) dt = ESO_DEFAULT_DT_S;

    /* e(k)=omega(k)-z1(k)。两个 next 均由同一旧状态计算，严格是前向欧拉：
     * z1(k+1)=z1(k)+dt*(b0*u(k)+z2(k)+beta1*e(k))
     * z2(k+1)=z2(k)+dt*beta2*e(k) */
    const float error = eso->omega - eso->z1;
    const float z1_next = eso->z1 + dt *
        (eso->b0 * eso->u_cmd + eso->z2 + eso->beta1 * error);
    const float z2_next = eso->z2 + dt * eso->beta2 * error;
    eso->z1 = z1_next;
    eso->z2 = z2_next;

    /* z2 是角加速度扰动，先换算成 Nm。机械振动也会出现在 z2 中，故不能
     * 直接叠加到 PID：按配置选择一阶低通或旁路，再限制绝对值和相邻拍变化量。 */
    const float raw_ff_nm = -eso->J * eso->z2;
    if (ESO_FEEDFORWARD_FILTER_HZ > 0.0f) {
        const float filter_tau_s = 1.0f / (ESO_TWO_PI * ESO_FEEDFORWARD_FILTER_HZ);
        const float alpha = dt / (filter_tau_s + dt);
        eso->ff_filtered_nm += alpha * (raw_ff_nm - eso->ff_filtered_nm);
    } else {
        /* 截止频率为 0 表示旁路滤波，而不是除以 0。仍会执行后续限幅和限速。 */
        eso->ff_filtered_nm = raw_ff_nm;
    }
    const float target_ff_nm = eso_clamp(eso->ff_filtered_nm,
                                         ESO_FEEDFORWARD_LIMIT_NM);
    const float max_step_nm = ESO_FEEDFORWARD_SLEW_LIMIT_NM_S * dt;
    eso->ff_nm += eso_clamp(target_ff_nm - eso->ff_nm, max_step_nm);
    eso->ff_nm = eso_clamp(eso->ff_nm, ESO_FEEDFORWARD_LIMIT_NM);
}

float ESO_GetFeedforward(const ESO_t *eso)
{
    if (eso == NULL || eso->initialized == 0U) return 0.0f;
    return eso->ff_nm;
}

void ESO_Reset(ESO_t *eso)
{
    if (eso == NULL) return;
    eso->omega = 0.0f;
    eso->u_cmd = 0.0f;
    eso->z1 = 0.0f;
    eso->z2 = 0.0f;
    eso->ff_filtered_nm = 0.0f;
    eso->ff_nm = 0.0f;
}
