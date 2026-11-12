#include "explicit_mpc.h"
#include "mpc_tables.h"
#include <math.h>
#include <stddef.h>

void wheelleg_mpc_init(WheelLegMPC *c) {
    if (c) { c->model = 0; c->region = 0; c->checks = 0; }
}

float wheelleg_mpc_sample_time(void) { return MPC_SAMPLE_TIME; }

float wheelleg_mpc_model_length(const WheelLegMPC *c) {
    return c && c->model < MPC_MODEL_COUNT ? mpc_lengths[c->model] : NAN;
}

int wheelleg_mpc_select_length(WheelLegMPC *c, float length) {
    unsigned best = 0;
    if (!c || !isfinite(length) || length < mpc_lengths[0]
            || length > mpc_lengths[MPC_MODEL_COUNT - 1]) return 0;
    for (unsigned i = 1; i < MPC_MODEL_COUNT; ++i)
        if (fabsf(length - mpc_lengths[i]) < fabsf(length - mpc_lengths[best])) best = i;
    c->model = (uint16_t)best;
    return 1;
}

/* A candidate is an offline affine law. These are its KKT region tests;
 * there is no matrix inversion, active-set update or QP solver here. */
static int region(const WheelLegMPC *c, unsigned r, const float v[4],
                  const float limit[4], float u[4]) {
    const int8_t *status = mpc_regions[r];
    const float (*t)[4] = mpc_correction[c->model][mpc_free_masks[r]];
    const float (*h)[4] = mpc_hessian[c->model];
    float delta[4], error[4], gradient[4], scale = 0.0f;
    for (unsigned i = 0; i < 4; ++i)
        delta[i] = status[i] ? status[i] * limit[i] - v[i] : 0.0f;
    for (unsigned i = 0; i < 4; ++i) {
        u[i] = v[i];
        for (unsigned j = 0; j < 4; ++j) u[i] += t[i][j] * delta[j];
        if (status[i]) u[i] = status[i] * limit[i];
        if (!isfinite(u[i]) || fabsf(u[i]) > limit[i] + 2e-5f) return 0;
        error[i] = u[i] - v[i];
    }
    for (unsigned i = 0; i < 4; ++i) {
        float row_scale = 0.0f;
        gradient[i] = 0.0f;
        for (unsigned j = 0; j < 4; ++j) {
            gradient[i] += h[i][j] * error[j];
            row_scale += fabsf(h[i][j] * error[j]);
        }
        if (row_scale > scale) scale = row_scale;
    }
    for (unsigned i = 0; i < 4; ++i)
        if (status[i] * gradient[i] > 2e-5f * (1.0f + scale)) return 0;
    for (unsigned i = 0; i < 4; ++i) {
        if (u[i] > limit[i]) u[i] = limit[i];
        if (u[i] < -limit[i]) u[i] = -limit[i];
    }
    return 1;
}

int wheelleg_mpc_evaluate(WheelLegMPC *c, const float x[10],
                          const float limits[4], const float offset[4], float u[4]) {
    float desired[4];
    if (!u) return 0;
    for (unsigned i = 0; i < 4; ++i) u[i] = 0.0f;
    if (!c || !x || !limits || c->model >= MPC_MODEL_COUNT || c->region >= 81) return 0;
    c->checks = 0;
    for (unsigned j = 0; j < 10; ++j) if (!isfinite(x[j])) return 0;
    for (unsigned i = 0; i < 4; ++i) {
        if (!isfinite(limits[i]) || limits[i] <= 0 || (offset && !isfinite(offset[i]))) return 0;
        desired[i] = offset ? offset[i] : 0.0f;
        for (unsigned j = 0; j < 10; ++j) desired[i] -= mpc_gain[c->model][i][j] * x[j];
        if (!isfinite(desired[i])) return 0;
    }
    c->checks = 1;
    if (region(c, c->region, desired, limits, u)) return 1;
    for (unsigned r = 0; r < 81; ++r) {
        if (r == c->region) continue;
        ++c->checks;
        if (region(c, r, desired, limits, u)) { c->region = (uint8_t)r; return 1; }
    }
    for (unsigned i = 0; i < 4; ++i) u[i] = 0.0f;
    return 0;
}
