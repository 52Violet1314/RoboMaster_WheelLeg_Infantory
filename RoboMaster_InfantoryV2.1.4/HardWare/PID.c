#include "PID.h"
#include "main.h"
#include "arm_math.h"

int Max(int x, int max);

void PID_Init(PID_Typedef *pid, float Kp, float Ki, float Kd) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->Target = 0;
  pid->Current = 0;
  pid->Err_Int = 0;
  pid->Last_Error = 0;
  pid->Error = 0;
  pid->Dout = 0;
  pid->Last_Dout = 0;
}

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

int PID(PID_Typedef *PID, int Target, int Current, int Err_Int_Max) {
  PID->Target = Target;
  PID->Current = Current;
  PID->Error = PID->Target - PID->Current;
  PID->Err_Int += PID->Error;
  PID->Err_Int = Max(PID->Err_Int, Err_Int_Max);
  PID->Dout =
      PID->Kd * 0.6 * (PID->Error - PID->Last_Error) + 0.4 * PID->Last_Dout;
  PID->Last_Error = PID->Error;
  PID->Last_Dout = PID->Dout;
  PID->Res = PID->Kp * PID->Error + PID->Ki * PID->Err_Int + PID->Dout;
  return PID->Res;
}

int PID_Max(PID_Typedef *PID, int Target, int Current, int Err_Int_Max,
            int Res_Max) {
  PID->Target = Target;
  PID->Current = Current;
  PID->Error = PID->Target - PID->Current;
  PID->Err_Int += PID->Error;
  PID->Err_Int = Max(PID->Err_Int, Err_Int_Max);
  PID->Dout =
      PID->Kd * 0.6 * (PID->Error - PID->Last_Error) + 0.4 * PID->Last_Dout;
  PID->Last_Error = PID->Error;
  PID->Last_Dout = PID->Dout;
  PID->Res = PID->Kp * PID->Error + PID->Ki * PID->Err_Int + PID->Dout;
  PID->Res = Max(PID->Res, Res_Max);
  return PID->Res;
}

static inline float fast_clip(float x, float min, float max) {
  return x < min ? min : (x > max ? max : x);
}

float PID_Max_Float(Float_PID_Typedef *PID, float Target, float Current,
                    float Err_Int_Max, float Res_Max) {
  PID->Error = Target - Current;
  if (PID->Ki > 0.0f) {
    PID->Err_Int += PID->Error;
    PID->Err_Int = fast_clip(PID->Err_Int, -Err_Int_Max, Err_Int_Max);
  }
  float error_diff = PID->Error - PID->Last_Error;
  PID->Dout = PID->Kd * 0.6f * error_diff + 0.4f * PID->Last_Dout;
  PID->Last_Error = PID->Error;
  PID->Last_Dout = PID->Dout;
  PID->Res = PID->Kp * PID->Error + PID->Ki * PID->Err_Int + PID->Dout;
  PID->Res = fast_clip(PID->Res, -Res_Max, Res_Max);
  return PID->Res;
}

int PID_Loc_Close_Max(PID_Typedef *PID, int Target, int Current,
                      int Err_Int_Max, int Res_Max, int Loc_Max) {
  PID->Error = Target - Current;
  if (PID->Error >= (Loc_Max / 2)) {
    PID->Error = Loc_Max - PID->Error;
  } else if (PID->Error <= -(Loc_Max / 2)) {
    PID->Error = Loc_Max + PID->Error;
  }
  if (Current != Target) {
    PID->Err_Int += PID->Error;
  }
  PID->Err_Int = Max(PID->Err_Int, Err_Int_Max);
  PID->Dout =
      PID->Kd * 0.6 * (PID->Error - PID->Last_Error) + 0.4 * PID->Last_Dout;
  PID->Last_Error = PID->Error;
  PID->Last_Dout = PID->Dout;
  int Out = PID->Kp * PID->Error + PID->Ki * PID->Err_Int + PID->Dout;
  Out = Max(Out, Res_Max);
  if (Out * PID->Error < 0) {
    Out = 0;
  }
  return Out;
}

void Loc_and_Speed_PID_Init(PID_Typedef *Loc_Pid, float Loc_Kp, float Loc_Ki,
                            float Loc_Kd, PID_Typedef *Speed_Pid,
                            float Speed_Kp, float Speed_Ki, float Speed_Kd) {
  PID_Init(Loc_Pid, Loc_Kp, Loc_Ki, Loc_Kd);
  PID_Init(Speed_Pid, Speed_Kp, Speed_Ki, Speed_Kd);
}
int Loc_and_Speed_PID(PID_Typedef *Loc_Pid, float Loc_Kp, float Loc_Ki,
                      float Loc_Kd, int Loc_Target, int Loc_Current,
                      PID_Typedef *Speed_Pid, float Speed_Kp, float Speed_Ki,
                      float Speed_Kd, int Speed_Current, int Loc_Err_Int_Max,
                      int Spd_Err_Int_Max, int Loc_Res_Max) {
  Loc_Pid->Kp = Loc_Kp;
  Loc_Pid->Ki = Loc_Ki;
  Loc_Pid->Kd = Loc_Kd;
  int Loc_Out =
      PID_Max(Loc_Pid, Loc_Target, Loc_Current, Loc_Err_Int_Max, Loc_Res_Max);
  Speed_Pid->Kp = Speed_Kp;
  Speed_Pid->Ki = Speed_Ki;
  Speed_Pid->Kd = Speed_Kd;
  int Spd_Out = PID(Speed_Pid, Loc_Out, Speed_Current, Spd_Err_Int_Max);
  return Spd_Out;
}

int Loc_and_Speed_PID_Close(PID_Typedef *Loc_Pid, float Loc_Kp, float Loc_Ki,
                            float Loc_Kd, int Loc_Target, int Loc_Current,
                            PID_Typedef *Speed_Pid, float Speed_Kp,
                            float Speed_Ki, float Speed_Kd, int Speed_Current,
                            int Loc_Err_Int_Max, int Spd_Err_Int_Max,
                            int Loc_Res_Max, int Loc_Max) {
  Loc_Pid->Kp = Loc_Kp;
  Loc_Pid->Ki = Loc_Ki;
  Loc_Pid->Kd = Loc_Kd;
  int Loc_Out = PID_Loc_Close_Max(Loc_Pid, Loc_Target, Loc_Current,
                                  Loc_Err_Int_Max, Loc_Res_Max, Loc_Max);
  Speed_Pid->Kp = Speed_Kp;
  Speed_Pid->Ki = Speed_Ki;
  Speed_Pid->Kd = Speed_Kd;
  int Spd_Out = PID(Speed_Pid, Loc_Out, Speed_Current, Spd_Err_Int_Max);
  return Spd_Out;
}
int Max(int x, int max) {
  if (x >= max) {
    x = max;
  } else if (x <= -max) {
    x = -max;
  }
  return x;
}

float Max_Float(float x, float max) {
  if (x >= max) {
    x = max;
  } else if (x <= -max) {
    x = -max;
  }
  return x;
}

// void MRAC_Init(MRAC *mrac, float K, float initial_y_m, float initial_y) {
//     mrac->K = K;
//     mrac->y_m = initial_y_m;
//     mrac->y = initial_y;
//     mrac->e = 0.0f;
//     mrac->u = 0.0f;
// }
