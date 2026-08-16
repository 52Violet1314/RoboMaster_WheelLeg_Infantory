#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"

//INCLUDE部分
#include "VMC.h"
#include "PID.h"
#include "stdbool.h"
#include "Ins_Task.h"
#include "Check_Task.h"
#include "Self_Rescue.h"
#include "wheel_kalman.h"

//角度转弧度
#define Ang_PI 0.01745329f
//弧度转角度
#define PI_Ang 57.2957805f

//DT7遥控器参数设置
#define RC_DX           250.0f
#define RC_YAW           60.0f
#define RC_DEADBAND       5.0f

typedef struct
{
	//主动控制
	bool bump_flag;     //进入磕台阶模式
	bool change_flag;   //调转车头标志位
	bool spinning_flag; //小陀螺标志位
	
	//自动判断(根据状态机判断)
	bool off_flag;      //离地标志位
	bool fall_flag;     //倒地标志位
	bool slip_flag[2];  //打滑标志位
	bool theta_flag[3]; //theta角度异常标志位[0左腿杆 1右腿杆 2车身(倒台阶了)]
}Flag_Bit_t;

typedef struct
{
	float dv_acc;          //加速度估计的瞬时速度
	float dv_wheel[2];     //轮部速度估计的瞬时速度
	float f_dv_wheel[2];   //打滑前的轮部速度
	
	float wheel_L_aver[4]; //均值滤波所需结构体
	float wheel_R_aver[4];
	
	float dt;              //打滑检测时间微分量
	uint32_t dwt_d_slip; 
}Slip_Check_t;

typedef struct
{
	int stuck_time[2]; //卡腿时间
	
	float Comp_Fn[2];
	float Comp_Length[2]; //补偿的腿部长度
}Bring_Legs_t;

typedef struct
{
	float yaw_t;         //目标YAW
	float d_x_t;         //目标X速度
	float d_yaw_t;       //目标YAW速度
	float Target_L0;     //目标腿长
	
	float MAX_Dx;        //x速度上限
	float MAX_Dyaw;      //yaw速度上限
	
	float Rc_X;          //遥控器读值
	float Rc_Yaw;	
	float Pc_X;
	float Pc_Yaw;	
}Goal_Setting_t;

typedef struct
{
	float Gravity_Comp_X;     //重心补偿(摆杆角度以及位移量)
	float Gravity_Comp_Theta; 
	
	float nuc_comp;     //NUC重量补偿
	float body_comp;    //车身自补偿
	
	float b_theta[3];   //项拟合系数(2:二次项系数 1:一次项系数 0:常数系数)
	float n_theta[3]; 
}Compensation_Amount_t;

typedef struct
{			
	float theta;            //腿部摆杆的角度以及角加速度(顺时针为正)
	float d_theta;
	
	float abs_leg_theta;    //相对于车体的绝对腿杆角度及角加速度
	
	float wheel_s;          //轮子转速
	float last_wheel_s;
	float	swing_s;          //摆杆的速度
	float stator_s;					//定子的速度
	
	float Fn;               //机体对摆杆的压力
	float fl;               //气弹簧对连杆在竖直方向上的力
	float fn;               //腿杆对大地的压力
	float aver_fn[4];
	
	float dd_theta;         //对腿杆角加速度求微分量
	float last_d_theta;
	float dd_L0;            //对腿长加速度求微分量
	float last_d_L0;
}Leg_Current_Situation_t;

typedef struct
{
	float abs_yaw;          //绝对YAW(由6020反馈值计算得出)
	
	float x;                //机体向前位移以及速度(向前为正)
	float d_x;
	float yaw;              //机体的YAW(逆时针为正)
	float d_yaw;
  float theta;            //机体摆杆的角度以及角加速度(顺时针为正)
	float d_theta;
	
	float R_turn;	          //二轮差速模型下转弯半径
	
	float a_x;              //车体X轴方向加速度
	float Estimate_h;       //车体高度估计
	float Estimate_dx;      //车体X轴方向的估计速度
	float Estimate_dyaw;    //车体YAW方向的估计角速度
}Body_Current_Situation_t;

void Chassis_Init(void);
void Chassis_Task(void);
void Vmc(Flag_Bit_t *flag,Joint_Motor_Status_t *joint_m[2],Vmc_Five_Link_Parameter_t *five_link[2]);
void Chassis_Can_Data_Send(Chassis_Motor_t *cm,Controlled_State_t *cs,Joint_Motor_Status_t *joint_m[2]);
void All_Theta_Err_Check(INS_t *ins,Flag_Bit_t *flag,Leg_Current_Situation_t *leg[2],Joint_Motor_Status_t *joint_m[2]);
void Slip_Check_Calc(Flag_Bit_t *flag,Slip_Check_t *slip,Body_Current_Situation_t *body,Leg_Current_Situation_t *leg[2]);
void Bump_Control(Flag_Bit_t *flag,Goal_Setting_t *goal,Leg_Current_Situation_t *leg[2],Joint_Motor_Status_t *joint_m[2]);
void YAW_Parameter_Processing(Flag_Bit_t *flag,Goal_Setting_t *goal,Controlled_State_t *cs,Body_Current_Situation_t *body);
void Check_Stuck_Leg(Flag_Bit_t *flag,Bring_Legs_t *bring,Controlled_State_t *cs,Leg_Current_Situation_t *leg[2],Joint_Motor_Status_t *joint_m[2]);
void Fast_Processing(Flag_Bit_t *flag,Goal_Setting_t *goal,Controlled_State_t *cs,Body_Current_Situation_t *body,Speed_Fusion_Parameter_t *sp_fusion);
void Variable_Information_Acquisition(INS_t *ins,Chassis_Motor_t *cm,Body_Current_Situation_t *body,Leg_Current_Situation_t *leg[2],Vmc_Five_Link_Parameter_t *five_link[2]);
void Chassis_Control(INS_t *ins,Flag_Bit_t *flag,RC_Ctrl_t *rc_ctrl,PC_Ctrl_t *pc_ctrl,Goal_Setting_t *goal,Self_Rescue_t *self_re,Controlled_State_t *cs,Body_Current_Situation_t *body,Leg_Current_Situation_t *leg[2]);
void Leg_Control(INS_t *ins,Flag_Bit_t *flag,Bring_Legs_t *bring,Goal_Setting_t *goal,Self_Rescue_t *self_re,Controlled_State_t *cs,Leg_Current_Situation_t *leg[2],Joint_Motor_Status_t *joint_m[2],Vmc_Five_Link_Parameter_t *five_link[2]);

//EXTERN部分
extern PID_t Roll_Pid;
extern Feedforward_t G_Comp[2];
extern bool chassis_ready_flag; 

extern Flag_Bit_t Flag;
extern Bring_Legs_t Bring_Legs;
extern Slip_Check_t Slip_Check;               
extern Goal_Setting_t Goal_Setting;  
extern Body_Current_Situation_t Body;
extern Leg_Current_Situation_t Leg[2];    
extern Compensation_Amount_t Compensation_Amount;

extern Leg_Current_Situation_t* leg_ptr[2];
extern Joint_Motor_Status_t* joint_m_ptr[2];
extern Vmc_Five_Link_Parameter_t* five_link_ptr[2];

#endif
