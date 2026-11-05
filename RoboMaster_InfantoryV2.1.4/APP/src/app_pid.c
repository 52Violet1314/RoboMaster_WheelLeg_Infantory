#include "app_pid.h"

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
float PID_Max_Float_Dt(Float_PID_Typedef *PID, float Target, float Current,
                       float Iout_Max, float Res_Max, float dt) {
  PID->Error = Target - Current;
  if (PID->Ki > 0.0f) {
    PID->Err_Int += PID->Error * dt;
    /* 先把积分状态换算为实际输出，再按输出单位限幅并回写状态。
     * 因而 Iout_Max=20 对腿长 PID 就是积分项最多贡献 ±20 N。 */
    float iout = PID->Ki * PID->Err_Int;
    iout = fast_clip(iout, -Iout_Max, Iout_Max);
    PID->Err_Int = iout / PID->Ki;
  }
  const float error_rate = (PID->Error - PID->Last_Error) / dt;
  PID->Dout = PID->Kd * 0.6f * error_rate + 0.4f * PID->Last_Dout;
  PID->Last_Error = PID->Error;
  PID->Last_Dout = PID->Dout;
  PID->Res = PID->Kp * PID->Error + PID->Ki * PID->Err_Int + PID->Dout;
  PID->Res = fast_clip(PID->Res, -Res_Max, Res_Max);
  return PID->Res;
}
