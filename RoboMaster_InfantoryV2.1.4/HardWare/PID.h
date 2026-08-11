#ifndef __PID_H
#define __PID_H
#include <stdint.h>
typedef struct
{
 float Kp;
 float Ki;
 float Kd;
 int16_t Target;
 int16_t Current;
 int16_t Error;
 int16_t Last_Error;
 int16_t Err_Int;
 int16_t Dout;
 int16_t Last_Dout;
 int16_t Res;
}PID_Typedef;

typedef struct
{
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
}Float_PID_Typedef;

typedef enum
{
  M2006_Start = 0,
  M2006_Stuck = 1,
  M2006_Auto = 2,
  M2006_HalfAuto = 3
}M2006_Status_Typedef;

typedef struct 
{

  M2006_Status_Typedef Status;
  PID_Typedef M2006_PID;
  /* data */
}M2006_Typedef;

typedef struct 
{
  PID_Typedef M3508_PID;
  /* data */
}M3508_Typedef;

typedef struct
{
    float K;            // 控制器增益
    float y_m;          // 参考模型输出
    float y;            // 实际系统输出
    float e;            // 误差
    float u;            // 控制信号
}MARC_Typedef;

#ifdef __cplusplus
extern "C" {
#endif

int Max(int x,int max);
void PID_Init(PID_Typedef* pid,float Kp,float Ki,float Kd);
void Float_PID_Init(Float_PID_Typedef* pid,float Kp,float Ki,float Kd);
int PID(PID_Typedef* PID,int Target,int Current,int Err_Int_Max);
int PID_Max(PID_Typedef* PID,int Target,int Current,int Err_Int_Max,int Res_Max);
float PID_Max_Float(Float_PID_Typedef* PID,float Target,float Current,float Err_Int_Max,float Res_Max);
int PID_Loc_Close_Max(PID_Typedef* PID,int Target,int Current,int Err_Int_Max,int Res_Max,int Loc_Max);
void Loc_and_Speed_PID_Init(PID_Typedef* Loc_Pid,float Loc_Kp,float Loc_Ki,float Loc_Kd,PID_Typedef* Speed_Pid,float Speed_Kp,float Speed_Ki,float Speed_Kd);
int Loc_and_Speed_PID(PID_Typedef* Loc_Pid,float Loc_Kp,float Loc_Ki,float Loc_Kd,int Loc_Target,int Loc_Current,PID_Typedef* Speed_Pid,float Speed_Kp,float Speed_Ki,float Speed_Kd,int Speed_Current,int Loc_Err_Int_Max,int Spd_Err_Int_Max,int Loc_Res_Max);
int Loc_and_Speed_PID_Close(PID_Typedef* Loc_Pid,float Loc_Kp,float Loc_Ki,float Loc_Kd,int Loc_Target,int Loc_Current,PID_Typedef* Speed_Pid,float Speed_Kp,float Speed_Ki,float Speed_Kd,int Speed_Current,int Loc_Err_Int_Max,int Spd_Err_Int_Max,int Loc_Res_Max,int Loc_Max);

#ifdef __cplusplus
}
#endif


#endif
