#ifndef WHEELLEG_EXPLICIT_MPC_H
#define WHEELLEG_EXPLICIT_MPC_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t model;
    uint8_t region;
    uint8_t checks;
} WheelLegMPC;

/* One context per robot. select_length returns 0 on an out-of-bank length.
 * x: [s,ds,yaw,dyaw,thetaL,dthetaL,thetaR,dthetaR,pitch,dpitch].
 * u: analytic [TwL,TwR,TbL,TbR], N*m (before wheel-reaction feedforward).
 * offset: optional additive desired torque, or NULL. limits: positive [4].
 * Call at MPC_SAMPLE_TIME; tables are fitted to the MuJoCo model, not hardware.
 * Evaluate returns 1 on success, 0 on invalid input/table/numerical failure.
 * On failure u is zeroed; the caller must decide how to handle the fault.
 */
void wheelleg_mpc_init(WheelLegMPC *context);
int wheelleg_mpc_select_length(WheelLegMPC *context, float length);
int wheelleg_mpc_evaluate(WheelLegMPC *context, const float x[10],
                          const float limits[4], const float offset[4], float u[4]);
float wheelleg_mpc_sample_time(void);
float wheelleg_mpc_model_length(const WheelLegMPC *context);

#ifdef __cplusplus
}
#endif
#endif
