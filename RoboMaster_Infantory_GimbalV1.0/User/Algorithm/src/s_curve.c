/**
 ******************************************************************************
 * @file    s_curve.c
 * @brief   七段式 S 曲线运动规划库实现
 *
 * 核心思想:
 *   1. SCurve_SetTarget 写入"最终期望位置"(绝对量, 不是增量)
 *   2. 目标变化超过死区时, 以当前 (pos, vel, accel=0) 为初始条件在线重规划
 *   3. 重规划用二分法求峰值速度 vp, 使"加速段距离 + 匀速段距离 + 减速段距离" 等于目标距离
 *   4. 段内用闭式多项式精确积分, 避免欧拉法离散误差导致峰值速度超限
 *   5. 输出 s->pos 是平滑后的"当前目标位置", 直接喂位置环
 ******************************************************************************
 */
#include "s_curve.h"
#include <math.h>
#include <stddef.h>

#define SC_EPS      1e-6f  /* 浮点比较容差: 判零/判段结束时用, 抵消 float 舍入误差 */
#define SC_MAX_ITER 30     /* 二分法求峰值速度的最大迭代: 30 次 ≈ 1e-9 相对精度 */
#define SC_MAX_DT   0.1f   /* dt 上限 s: 超过视为任务卡顿, 钳回默认值防止积分爆炸 */
#define SC_DEF_DT   0.001f /* 默认控制周期 s: dt 非法时兜底, 对应 1kHz 任务 */

/* 前向声明, 供公共 API 调用 */
static void seg7_replan(SCurve_t *s);

/* ================================================================
 * 公共 API
 * ================================================================ */

/**
 * @brief 初始化为七段式规划器, 写入限幅三段并置默认死区.
 * @note  非法 v_max/a_max/j_max 兜底为正值, 避免后续除零/开根号 NaN
 */
void SCurve_InitSeg7(SCurve_t *s, float v_max, float a_max, float j_max)
{
    if (s == NULL) return;
    s->v_max = (v_max > 0.0f) ? v_max : 1e3f;
    s->a_max = (a_max > 0.0f) ? a_max : 1e4f;
    s->j_max = (j_max > 0.0f) ? j_max : 1e6f;
    s->deadband = 0.001f;  /* 本项目 pitch 的默认死区为 0.001 rad */
    SCurve_Reset(s, 0.0f);
}

/**
 * @brief 写入新的最终期望位置.
 * @note  仅写入, 不立即规划——规划在 Update 中按"目标跳变超死区"触发
 */
void SCurve_SetTarget(SCurve_t *s, float target)
{
    if (s == NULL) return;
    s->target = target;
}

/**
 * @brief 重置规划器到指定位置, 清零速度/加速度/段号.
 * @note  典型调用时机: 使能上升沿(锚点到当前反馈)、模式切换捕获起点
 */
void SCurve_Reset(SCurve_t *s, float pos)
{
    if (s == NULL) return;
    s->pos         = pos;
    s->target      = pos;   /* Reset 时 target = pos, 避免下一周期立刻重规划 */
    s->last_target = pos;
    s->vel         = 0.0f;
    s->accel       = 0.0f;
    s->phase       = 0;     /* 0 = 空闲, 不在七段中 */
    s->t_now       = 0.0f;
    s->v_peak      = 0.0f;
}

/**
 * @brief 设置目标跳变死区.
 * @note  本项目 pitch 的 deadband 单位为 rad
 */
void SCurve_SetDeadband(SCurve_t *s, float deadband)
{
    if (s == NULL) return;
    if (deadband > 0.0f) s->deadband = deadband;
}

/**
 * @brief 推进一个控制周期, 内部自动判断是否需要重规划.
 * @note  调用方每周期执行: SetTarget → Update → 读 pos 喂位置环
 */
void SCurve_Update(SCurve_t *s, float dt)
{
    if (s == NULL) return;

    /* dt 合法性检查: 非法或过大时兜底, 防止卡顿后积分爆炸 */
    if (dt <= 0.0f || dt > SC_MAX_DT) dt = SC_DEF_DT;

    /* 触发重规划的两个条件:
     *   (1) 目标变化超过死区: 用户改了目标
     *   (2) 空闲但仍有残余误差: 过冲回拉或外部扰动 */
    if (fabsf(s->target - s->last_target) > s->deadband ||
        (s->phase == 0 && fabsf(s->target - s->pos) > s->deadband)) {
        seg7_replan(s);
    }
    if (s->phase == 0) return;  /* 空闲且无误差, 不需要推进 */

    /* 段内解析式推进: 恒定 jerk 段用闭式多项式精确求 pos/vel/accel,
     * 避免数值积分离散误差导致峰值速度超过 v_max */
    float remaining = dt;
    while (remaining > 0.0f && s->phase >= 1 && s->phase <= 7) {
        float seg_remain = s->t[s->phase] - s->t_now;

        /* 段长为零(如短行程 t2/t6=0), 直接跳过到下一段 */
        if (seg_remain < SC_EPS) {
            s->phase++;
            s->t_now = 0.0f;
            continue;
        }

        /* 本周期步长: 跨段时只走到当前段末, 剩余量留给下一段 */
        float step = (remaining < seg_remain) ? remaining : seg_remain;
        if (step < SC_EPS) break;  /* 跨段后剩余 dt 已可忽略, 丢弃 */

        /* 恒定 jerk 闭式积分:
         *   p(t) = p0 + v0*t + 0.5*a0*t^2 + j*t^3/6
         *   v(t) = v0 + a0*t + 0.5*j*t^2
         *   a(t) = a0 + j*t */
        float j  = (float)s->jerk_sign[s->phase] * s->j_max;
        float a0 = s->accel, v0 = s->vel, p0 = s->pos;
        s->pos   = p0 + v0 * step + 0.5f * a0 * step * step
                 + j * step * step * step / 6.0f;
        s->vel   = v0 + a0 * step + 0.5f * j * step * step;
        s->accel = a0 + j * step;

        s->t_now += step;
        remaining -= step;

        /* 当前段时间走完, 切到下一段 */
        if (s->t_now >= s->t[s->phase] - SC_EPS) {
            s->t_now = 0.0f;
            s->phase++;
        }
    }

    /* 七段走完: 清状态, 死区内吸附到目标 */
    if (s->phase > 7) {
        s->phase = 0;
        s->vel = 0.0f;
        s->accel = 0.0f;
        if (fabsf(s->target - s->pos) < s->deadband) s->pos = s->target;
    }
}

/* ================================================================
 * 内部: 重规划与单段距离计算
 * ================================================================ */

/**
 * @brief 单段 jerk 限幅位移计算 (归一化坐标系, 正向=朝目标).
 * @param v0  起点速度(归一化, 可负=背向目标)
 * @param vp  终点速度(归一化, >=0)
 * @param A   加速度幅值上限
 * @param J   jerk 幅值上限
 * @param t1  输出: 段1时长 (加速度 0→a_peak)
 * @param t2  输出: 段2时长 (匀加速 a_peak)
 * @param t3  输出: 段3时长 (加速度 a_peak→0)
 * @return 净位移(可负, 取决于方向)
 * @note  内部分两种情况:
 *        (a) dv 大: 能到 a_max, 含 t2 匀加速段(梯形加速度)
 *        (b) dv 小: 三角形加速度, 峰值 < a_max, t2 = 0
 */
static float seg7_move_dist(float v0, float vp, float A, float J,
                            float *t1, float *t2, float *t3)
{
    float dv = vp - v0;
    if (fabsf(dv) < SC_EPS) {
        *t1 = *t2 = *t3 = 0.0f;
        return 0.0f;
    }
    float sgn = (dv > 0.0f) ? 1.0f : -1.0f;   /* 加速度方向 */

    float a_peak;   /* 峰值加速度幅值 */
    if (fabsf(dv) >= A * A / J - SC_EPS) {
        /* 能达到 a_max, 含 t2 匀加速段 */
        a_peak = A;
        *t1 = A / J;
        *t3 = A / J;
        float v1 = v0 + sgn * A * A / (2.0f * J);
        float v2 = vp - sgn * A * A / (2.0f * J);
        *t2 = (v2 - v1) / (sgn * A);
        if (*t2 < 0.0f) *t2 = 0.0f;
    } else {
        /* 短行程: 三角形加速度, 峰值 < a_max */
        a_peak = sqrtf(J * fabsf(dv));
        *t1 = a_peak / J;
        *t3 = a_peak / J;
        *t2 = 0.0f;
    }

    /* 三段速度端点: v0 → v1(段1末) → v2(段2末) → vp(段3末) */
    float v1 = v0 + sgn * a_peak * a_peak / (2.0f * J);
    float v2 = vp - sgn * a_peak * a_peak / (2.0f * J);

    /* 三段位移闭式积分:
     *   d1: jerk 段, v(t) = v0 + j*t^2/2,  d1 = v0*t1 + j*t1^3/6
     *   d2: 匀加速段,            d2 = v1*t2 + 0.5*a_peak*t2^2
     *   d3: -jerk 段, d3 = v2*t3 + 0.5*a_peak*t3^2 - j*t3^3/6 */
    float d1 = v0 * (*t1) + sgn * J * (*t1) * (*t1) * (*t1) / 6.0f;
    float d2 = v1 * (*t2) + 0.5f * sgn * a_peak * (*t2) * (*t2);
    float d3 = v2 * (*t3) + 0.5f * sgn * a_peak * (*t3) * (*t3)
             - sgn * J * (*t3) * (*t3) * (*t3) / 6.0f;

    return d1 + d2 + d3;
}

/**
 * @brief 在线重规划: 以当前 (pos, vel, accel=0) 为初始条件, 规划到 target.
 * @note  加速度假设为 0 是简化: 上一周期段末加速度必为 0(七段定义保证),
 *        除非跨段被打断, 此时丢失的加速度项影响可接受(误差小, 下次重规划纠正)
 */
static void seg7_replan(SCurve_t *s)
{
    float D = s->target - s->pos;
    if (fabsf(D) < s->deadband) {
        /* 到位: 直接吸附到目标, 清状态 */
        s->pos = s->target;
        s->vel = 0.0f;
        s->accel = 0.0f;
        s->phase = 0;
        s->last_target = s->target;
        return;
    }

    float dir = (D > 0.0f) ? 1.0f : -1.0f;  /* 朝目标方向 */
    float Dn  = fabsf(D);                    /* 归一化距离(>0) */
    float v0  = s->vel * dir;               /* 归一化速度(可负=背向) */
    float A   = s->a_max;
    float J   = s->j_max;
    s->accel = 0.0f;                        /* 重规划假设 a0 = 0 */

    /* 二分法求峰值速度 vp, 使总位移 == 目标距离:
     *   总位移 = seg7_move_dist(v0, vp) + seg7_move_dist(vp, 0)
     *   vp 越大 → 距离越大 → 单调函数, 可二分
     * 边界: lo=0 (最小峰值速度), hi=v_max (最大峰值速度) */
    float t1, t2, t3, t5, t6, t7;
    float lo = 0.0f, hi = s->v_max;
    for (int i = 0; i < SC_MAX_ITER; i++) {
        float mid = 0.5f * (lo + hi);
        float d = seg7_move_dist(v0, mid, A, J, &t1, &t2, &t3)
                + seg7_move_dist(mid, 0.0f, A, J, &t5, &t6, &t7);
        if (d <= Dn) lo = mid; else hi = mid;
    }
    float vp = lo;  /* 取下界, 保守不超调 */

    /* 最终确认加速段/减速段的精确时长与距离 */
    float d_acc = seg7_move_dist(v0, vp, A, J, &t1, &t2, &t3);
    float d_dec = seg7_move_dist(vp, 0.0f, A, J, &t5, &t6, &t7);
    float d4 = Dn - d_acc - d_dec;   /* 匀速段距离, 若 d_acc+d_dec > Dn 则 vp 实际达不到 */
    if (d4 < 0.0f) d4 = 0.0f;

    /* 七段时长: t1~t3 加速段, t4 匀速段, t5~t7 减速段 */
    s->t[1] = t1; s->t[2] = t2; s->t[3] = t3;
    s->t[4] = (vp > SC_EPS) ? (d4 / vp) : 0.0f;  /* 匀速段 = 距离 / 速度 */
    s->t[5] = t5; s->t[6] = t6; s->t[7] = t7;

    /* 加速度方向: vp > v0 表示要先加速(a_dir=+1), v0 > vp 表示要先减速(a_dir=-1) */
    float a_dir = (vp > v0 + SC_EPS) ? 1.0f
                : ((vp < v0 - SC_EPS) ? -1.0f : 0.0f);

    /* 各段 jerk 符号(已并入 dir 和 a_dir):
     *   段1: +dir*a_dir (加速段开始, jerk 朝 a_peak 方向)
     *   段2: 0          (匀加速, jerk=0)
     *   段3: -dir*a_dir (加速段结束, jerk 反向把加速度拉回 0)
     *   段4: 0          (匀速, jerk=0)
     *   段5: -dir        (减速段开始, jerk 朝 -a_peak, 即朝目标方向反向)
     *   段6: 0          (匀减速)
     *   段7: +dir        (减速段结束, jerk 把加速度拉回 0) */
    s->jerk_sign[1] = (int8_t)(dir * a_dir);
    s->jerk_sign[2] = 0;
    s->jerk_sign[3] = (int8_t)(-dir * a_dir);
    s->jerk_sign[4] = 0;
    s->jerk_sign[5] = (int8_t)(-dir);
    s->jerk_sign[6] = 0;
    s->jerk_sign[7] = (int8_t)(dir);

    s->v_peak = vp;
    s->phase = 1;       /* 进入段1 */
    s->t_now = 0.0f;
    s->last_target = s->target;
}
