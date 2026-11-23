#include "app_pid.h"
#include <math.h>

void Float_PID_Init(Float_PID_Typedef *pid, float Kp, float Ki, float Kd) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->Target = 0.0f;
  pid->Current = 0.0f;
  pid->Err_Int = 0.0f;
  pid->Last_Error = 0.0f;
  pid->Error = 0.0f;
  pid->Dout = 0.0f;
  pid->Last_Dout = 0.0f;
}

static inline float fast_clip(float x, float min, float max) {
  return x < min ? min : (x > max ? max : x);
}

/* 带真实时间步长的浮点 PID：Ki 单位为输出/(误差·s)，Kd 单位为输出·s/误差。
 * Iout_Max 使用输出单位，而非内部积分状态的 error·s 单位。 */
static float PID_UpdateWithDerivative(Float_PID_Typedef *PID, float error,
                                      float Iout_Max, float Res_Max, float dt,
                                      float derivative_rate) {
  PID->Error = error;
  if (PID->Ki > 0.0f) {
    PID->Err_Int += PID->Error * dt;
    /* 先把积分状态换算为实际输出，再按输出单位限幅并回写状态。
     * 因而 Iout_Max=20 对腿长 PID 就是积分项最多贡献 ±20 N。 */
    float iout = PID->Ki * PID->Err_Int;
    iout = fast_clip(iout, -Iout_Max, Iout_Max);
    PID->Err_Int = iout / PID->Ki;
  }
  PID->Dout = PID->Kd * 0.6f * derivative_rate + 0.4f * PID->Last_Dout;
  PID->Last_Error = PID->Error;
  PID->Last_Dout = PID->Dout;
  PID->Res = PID->Kp * PID->Error + PID->Ki * PID->Err_Int + PID->Dout;
  PID->Res = fast_clip(PID->Res, -Res_Max, Res_Max);
  return PID->Res;
}

float PID_Max_Float_Dt(Float_PID_Typedef *PID, float Target, float Current,
                       float Iout_Max, float Res_Max, float dt) {
  const float error = Target - Current;
  return PID_UpdateWithDerivative(PID, error, Iout_Max, Res_Max, dt,
                                   (error - PID->Last_Error) / dt);
}

float PID_Max_Float_Measurement_Dt(Float_PID_Typedef *PID, float Target, float Current,
                                   float Iout_Max, float Res_Max, float dt,
                                   float measurement_rate) {
  const float derivative_rate = isfinite(measurement_rate) ? -measurement_rate : 0.0f;
  return PID_UpdateWithDerivative(PID, Target - Current, Iout_Max, Res_Max, dt,
                                   derivative_rate);
}
