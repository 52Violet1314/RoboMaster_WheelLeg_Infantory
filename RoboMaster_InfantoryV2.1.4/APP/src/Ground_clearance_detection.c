#include "Ground_clearance_detection.h"
#include "app_leg_force_compensation.hpp"
#include "arm_math.h"
#include <math.h>

/* ==================== 可调参数（上板标定） ==================== */
/* 质量口径：轮毂电机转子 + 轮子。腿对轮轴的作用已由 F0/Tp 投影给出，
 * 不能在此重复加入整腿质量。起身前未启用重力补偿时，腿自重的未建模误差
 * 会保留在 Fn 中，因此离地阈值需要按实测留裕量。 */
#define GD_MASS_WHEEL_KG        0.50f

#define GD_G                    9.81f

/* Fn 一阶低通系数（1 ms 周期时截止频率约 26 Hz） */
#define GD_FN_FILTER_A          0.15f

/* 离地/落地阈值（N）。先按“静态 Fn 的一半”预置，必须上板实测后改。 */
#define GD_FN_OFF_THRESHOLD_N   15.0f
/* 落地迟滞：必须回到 阈值*1.5 以上才认为重新触地 */
#define GD_FN_ON_RATIO          1.5f

/* 连续确认拍数（1ms 一拍） */
#define GD_CONFIRM_OFF_TICKS    10U
#define GD_CONFIRM_ON_TICKS     15U

/* ==================== 内部状态 ==================== */
typedef struct {
    float   fn_filt;
    uint16_t cnt;
    uint8_t  off_ground;
    uint8_t  init;
} GroundDetection_State_t;

static GroundDetection_State_t s_state[2];

/* ==================== 纯观测器 ==================== */
float Ground_Clearance_Detection(float f0_without_spring_n, float tp_vmc_nm, float L0,
                                 float d_L0, float dd_L0, float dd_zm,
                                 float phi, float d_phi, float dd_phi)
{
    if (!isfinite(f0_without_spring_n) || !isfinite(tp_vmc_nm) ||
        !isfinite(L0) || !isfinite(d_L0) || !isfinite(dd_L0) ||
        !isfinite(dd_zm) || !isfinite(phi) || !isfinite(d_phi) ||
        !isfinite(dd_phi) || L0 <= 0.0f) {
        return 0.0f;
    }

    const float cos_phi = arm_cos_f32(phi);
    const float sin_phi = arm_sin_f32(phi);

    /* 腿对轮轴的向下竖直传力。 */
    const float p_down_n = f0_without_spring_n * cos_phi
                         + (tp_vmc_nm * sin_phi) / L0;

    /* 轮心竖直向下加速度。
     * z_w = z_m + L0*cos(phi)，故完整极坐标二阶导如下。 */
    const float dd_z_w = dd_zm
                       + dd_L0 * cos_phi
                       - 2.0f * d_L0 * d_phi * sin_phi
                       - L0 * dd_phi * sin_phi
                       - L0 * d_phi * d_phi * cos_phi;

    /* N 为地面对轮子的向上反力，竖直向下为正：
     * m*a_down = m*g + P_down - N。 */
    return p_down_n + GD_MASS_WHEEL_KG * (GD_G - dd_z_w);
}

/* ==================== 带状态检测 ==================== */
float GroundDetection_Update(uint8_t side,
                             float f0_command_n, float tp_vmc_nm, float L0,
                             float d_L0, float dd_L0, float dd_zm,
                             float phi, float d_phi, float dd_phi)
{
    if (side > 1U) return 0.0f;

    GroundDetection_State_t *st = &s_state[side];
    if (!isfinite(f0_command_n) || !isfinite(tp_vmc_nm) ||
        !isfinite(L0) || !isfinite(d_L0) || !isfinite(dd_L0) ||
        !isfinite(dd_zm) || !isfinite(phi) || !isfinite(d_phi) ||
        !isfinite(dd_phi) || L0 <= 0.0f) {
        /* 无效样本不允许污染滤波器，也不推进连续确认计数。 */
        st->cnt = 0U;
        return st->init ? st->fn_filt : 0.0f;
    }

    /* 还原“腿对轮子的实际沿腿长支持力”。
     *
     * 沿腿长方向压在轮上的力有两部分：
     *   1) 电机力 F0_cmd（= 腿长PID + 气弹簧补偿(负) + 起身预输出 + roll补偿）
     *   2) 气弹簧本身的推力 S = -comp，方向推长腿（正）
     * F0_cmd 里的补偿是负的 -S，正好抵消弹簧正向推力，二者在轮端不贡献净力：
     *   F0_without_spring = F0_cmd + S = F0_cmd - comp。
     * Tp 必须由调用者传入本轮真正送入 VMC 的值；这里不重复叠加重力补偿。 */
    const float spring_force_n = -LegForceCompensation_GetFittedF0(L0);
    const float f0_without_spring_n = f0_command_n + spring_force_n;

    const float fn_raw = Ground_Clearance_Detection(f0_without_spring_n, tp_vmc_nm, L0,
                                                    d_L0, dd_L0, dd_zm,
                                                    phi, d_phi, dd_phi);

    if (!st->init) {
        st->fn_filt = fn_raw;
        st->cnt = 0U;
        st->off_ground = 0U;
        st->init = 1U;
        return st->fn_filt;
    }

    st->fn_filt = GD_FN_FILTER_A * fn_raw
                + (1.0f - GD_FN_FILTER_A) * st->fn_filt;

    if (!st->off_ground) {
        /* 当前判为触地：Fn 掉到阈值以下并持续 N 拍 -> 离地 */
        if (st->fn_filt < GD_FN_OFF_THRESHOLD_N) {
            if (st->cnt < GD_CONFIRM_OFF_TICKS) st->cnt++;
            if (st->cnt >= GD_CONFIRM_OFF_TICKS) {
                st->off_ground = 1U;
                st->cnt = 0U;
            }
        } else {
            st->cnt = 0U;
        }
    } else {
        /* 当前判为离地：Fn 回到 阈值*1.5 以上并持续 M 拍 -> 触地 */
        if (st->fn_filt > GD_FN_OFF_THRESHOLD_N * GD_FN_ON_RATIO) {
            if (st->cnt < GD_CONFIRM_ON_TICKS) st->cnt++;
            if (st->cnt >= GD_CONFIRM_ON_TICKS) {
                st->off_ground = 0U;
                st->cnt = 0U;
            }
        } else {
            st->cnt = 0U;
        }
    }

    return st->fn_filt;
}

float GroundDetection_GetFn(uint8_t side) {
    return (side > 1U) ? 0.0f : s_state[side].fn_filt;
}

uint8_t GroundDetection_IsOffGround(uint8_t side) {
    return (side > 1U) ? 0U : s_state[side].off_ground;
}

uint8_t GroundDetection_IsAirborne(void) {
    return (uint8_t)(s_state[GD_SIDE_LEFT].off_ground &&
                     s_state[GD_SIDE_RIGHT].off_ground);
}

void GroundDetection_Reset(void) {
    for (uint8_t i = 0U; i < 2U; ++i) {
        s_state[i].fn_filt = 0.0f;
        s_state[i].cnt = 0U;
        s_state[i].off_ground = 0U;
        s_state[i].init = 0U;
    }
}
