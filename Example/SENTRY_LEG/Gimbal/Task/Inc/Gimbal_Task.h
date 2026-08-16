#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "main.h"

//INCLUDE部分
#include "PID.h"
#include "VT03.h"
#include "Nuc_Task.h"
#include "Ins_Task.h"
#include "Check_Task.h"
#include "Limit_Task.h"
#include "Some_Functions.h"
#include "Board_Can_Task.h"

//角度转弧度
#define Ang_PI 0.01745329f
//弧度转角度
#define PI_Ang 57.2957805f

//正负方向零点位置
#define ZERO_HEAD_SMY    2.10411167f
#define ZERO_HEAD_YAW    1.18284357f //正方向零点位置
#define ZERO_BACK_YAW   -1.97830963 //负方向零点位置

//卡弹反拨的检测转矩电流值以及转速值
#define Jammed_Rpm            15
#define Jammed_Cur         14000

//拨盘以及摩擦轮转速设定
#define Dial_Speed          6000
#define Friction_Speed      5500

//云台俯仰角限位
#define PITCH_UP_LIMIT_POSITION     30
#define PITCH_DOWN_LIMIT_POSITION  -24

//控制器灵敏度以及死区设置
#define RC_PITCH_SENSITIVITY        0.0002
#define RC_YAW_SENSITIVITY          0.0004
#define RC_DEADBAND                 5

#define PC_PITCH_SENSITIVITY        0.0008
#define PC_YAW_SENSITIVITY          0.0008
#define PC_DEADBAND                 10

typedef enum
{
	Close,
	Open
}Shoot_Condition_t; //发射机构状态

typedef struct
{
	bool run_flag; //启动标志

	float Yaw_Zero_Target[2]; //[0:BIG_YAW 1:SMALL_YAW]
}Self_Rescue_t;

typedef struct 
{
	uint32_t dwt_l; //计算DT
	uint32_t dwt_r;
	
	float L_Last_Rpm; //记录上一次转速
	float R_Last_Rpm;
	
	float L_A_Rpm; //摩擦轮的转动加速度
	float R_A_Rpm;
}Calculate_Acc;

typedef struct 
{
	int16_t Rc_Pitch; //遥控器数据
	int16_t Rc_Yaw;
	int16_t Pc_Pitch;
	int16_t Pc_Yaw;
	
	float abs_pitch; //绝对数据
	float abs_yaw[2]; //[0:BIG_YAW 1:SMALL_YAW]
	
	float pitch; //陀螺仪数据
	float d_pitch;
	float yaw;
	float d_yaw;
	
	float follow_theta; //大YAW跟随角度以及角速度
	float follow_dtheta;
	
	float abs_yaw_ref[2]; //绝对控制下的目标值[0:BIG_YAW 1:SMALL_YAW]
	
	float pitch_ref; //相对控制下的目标值
	float yaw_ref[2]; //[0:BIG_YAW 1:SMALL_YAW]
	
	float Pitch_Motor_Out; //电机输出
	float Yaw_Motor_Out[2]; //[0:BIG_YAW 1:SMALL_YAW]
}Gimbal_Status_t;

typedef struct 
{
	int16_t L_Rpm; //摩擦轮电机参数(转子RPM)
	int16_t R_Rpm;
	
	int16_t D_Rpm; //拨盘电机参数(转子RPM 电流)
	int16_t D_Cur;
	
	float D_Pos; //估算出来的拨盘位置
	
	float L_Motor_Out; //摩擦轮电机输出
	float R_Motor_Out;
	float D_Motor_Out; //拨盘电机输出
	
	float Target_Rpm[2]; //设定目标转速[0摩擦轮 1拨盘]
	
	Calculate_Acc acc;
}Shoot_Status_t;

typedef struct
{
	float p_pitch[5]; //项拟合系数(4:四次项系数 3:三次项系数 2:二次项系数 1:一次项系数 0:常数系数)
	
	float Gravity_Comp_PITCH; //重心补偿(PITCH)
}Compensation_Amount_t;

void Gimbal_Init(void);
void Gimbal_Task(void);
void Gimbal_Target_Limit(Gimbal_Status_t *gs);
void Auto_Aim(Aim_Rx *aim,Gimbal_Status_t *gs);
void Gimbal_Can_Data_Send(Shoot_Status_t *ss,Gimbal_Status_t *gs);
void Shoot_Control(Shoot_Status_t *ss,Heat_Control_t *hc,Shoot_Condition_t *sc);
void Variable_Information_Acquisition(INS_t *ins,Gimbal_Motor_t *gm,Shoot_Status_t *ss,Gimbal_Status_t *gs);
void Gimbal_Controllor(Shoot_Status_t *ss,Gimbal_Status_t *gs,Self_Rescue_t *self_re,Controlled_State_t *cs,Compensation_Amount_t *ca);
void Gimbal_Control(RC_Ctrl_t *rc_ctrl,PC_Ctrl_t *pc_ctrl,Gimbal_Status_t *gs,Shoot_Condition_t *sc,Self_Rescue_t *self_re,Controlled_State_t *cs);

//EXTERN部分
extern bool gimbal_ready_flag;
extern float Friction_Speed_Comp;

extern Shoot_Condition_t Shoot_Condition; 

#endif
