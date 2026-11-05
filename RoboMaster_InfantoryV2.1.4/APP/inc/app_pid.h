#ifndef __APP_PID_H
#define __APP_PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Target;
    float Current;
    float Error;
    float Last_Error;
    float Err_Int;
    float Dout;
    float Last_Dout;
    float Res;
} Float_PID_Typedef;

#ifdef __cplusplus
extern "C" {
#endif

void Float_PID_Init(Float_PID_Typedef *pid, float Kp, float Ki, float Kd);
/* 带真实 dt 的浮点 PID。
 * Iout_Max 与 Res_Max 均为输出单位；腿长 PID 中均为 N。
 * 内部 Err_Int 仍保存为 error·s，调用者不应再按其单位设置限幅。 */
float PID_Max_Float_Dt(Float_PID_Typedef *PID, float Target, float Current,
                       float Iout_Max, float Res_Max, float dt);

#ifdef __cplusplus
}
#endif

#endif
