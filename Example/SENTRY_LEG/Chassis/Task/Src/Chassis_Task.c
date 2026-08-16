#include "Chassis_Task.h"

//INCLUDE部分
#include "LQR.h"
#include "Ref_Task.h"
#include "Parameter.h"
#include "Board_Can_Task.h"
//全局变量定义部分(左腿[0] 右腿[1])
Flag_Bit_t Flag;
Bring_Legs_t Bring_Legs;
Slip_Check_t Slip_Check;               
Goal_Setting_t Goal_Setting;  
Body_Current_Situation_t Body;
Leg_Current_Situation_t Leg[2];    
Compensation_Amount_t Compensation_Amount;

Leg_Current_Situation_t* leg_ptr[2] = {&Leg[0], &Leg[1]};
Joint_Motor_Status_t* joint_m_ptr[2] = {&Joint_Motor_Status[0], &Joint_Motor_Status[1]};
Vmc_Five_Link_Parameter_t* five_link_ptr[2] = {&Five_Link_Parameter[0], &Five_Link_Parameter[1]};

//建立控制器结构体
PID_t Roll_Pid;     //Roll补偿
PID_t Leg_L_Pid[2]; //腿长PD控制器
PID_t Leg_P_Pid[2]; //腿杆绝对位置PID控制器

//建立前馈控制结构体
Feedforward_t G_Comp[2]; //车身重力方向补偿

bool chassis_ready_flag; //当前线程初始化完成标志 

//提前声明函数
extern void Save_Self(Flag_Bit_t *flag,Self_Rescue_t *self_re,Leg_Current_Situation_t *leg[2]);
void LQR(Flag_Bit_t *flag,Goal_Setting_t *goal,Controlled_State_t *cs,Compensation_Amount_t *comp,Body_Current_Situation_t *body,Leg_Current_Situation_t *leg[2],Joint_Motor_Status_t *joint_m[2],Vmc_Five_Link_Parameter_t *five_link[2]);

/*******************************************************************************************************
Chassis任务初始化
********************************************************************************************************/
void Chassis_Init(void)
{			
	//速度加速度融合初始化
	V_Kf_Init(&V_kf);
	
	//初始化PID                MAXP MAXI        P     I      D           dt
	PID_Init(&Leg_L_Pid[0]   ,  180,  30, 0, 1820,    0,   103,  0,0,0,0, 0,NO_CIRCLE,NONE); //左右腿腿长串级PID
	PID_Init(&Leg_L_Pid[1]   ,  180,  30, 0, 1820,    0,   103,  0,0,0,0, 0,NO_CIRCLE,NONE);
	
	PID_Init(&Leg_P_Pid[0]   ,   80,  60, 0,  140,    0,     7,  0,0,0,0, 0,RADIAN,NONE);    //左右腿腿杆绝对位置串级PID
	PID_Init(&Leg_P_Pid[1]   ,   80,  60, 0,  140,    0,     7,  0,0,0,0, 0,RADIAN,NONE);
	
	PID_Init(&Roll_Pid       , 0.40, 0.1, 0, 1.24,    0, 0.002,  0,0,0,0, 4,NO_CIRCLE,NONE); //ROLL轴补偿PID
	
	//初始化前馈(c0=静态增益 c1=速度补偿 c2=加速度补偿)
	static float ffc_g_L[3] = {1.1f, 0.0004f, 0.0001f};
	static float ffc_g_R[3] = {1.1f, 0.0004f, 0.0001f};
	Feedforward_Init(&G_Comp[0],100.0f,ffc_g_L ,0.05f,1.0f,1.0f);
	Feedforward_Init(&G_Comp[1],100.0f,ffc_g_R ,0.05f,1.0f,1.0f);
	
	//补偿的拟合项系数初始化
	Compensation_Amount.n_theta[0] =  0.0912f;
	Compensation_Amount.n_theta[1] = -0.2514f;
	Compensation_Amount.n_theta[2] =  0.0932f;
	
	Compensation_Amount.b_theta[0] =  0.0636f;
	Compensation_Amount.b_theta[1] = -0.0810f;
	Compensation_Amount.b_theta[2] = -0.0718f;
	
	//当前线程初始化完成
	chassis_ready_flag = 1;
	
	//等待
	while(all_ready_flag!=1){osDelay(1);}
}

/*******************************************************************************************************
Chassis任务
********************************************************************************************************/
void Chassis_Task(void)
{
	Variable_Information_Acquisition(&INS,&Chassis_Motor,&Body,leg_ptr,five_link_ptr);
	Slip_Check_Calc(&Flag,&Slip_Check,&Body,leg_ptr);
	All_Theta_Err_Check(&INS,&Flag,leg_ptr,joint_m_ptr);
	Chassis_Control(&INS,&Flag,&Rc_Ctrl,&Pc_Ctrl,&Goal_Setting,&Self_Rescue,&Controlled_State,&Body,leg_ptr);
	Fast_Processing(&Flag,&Goal_Setting,&Controlled_State,&Body,&Speed_Fusion_Parameter);
	Check_Stuck_Leg(&Flag,&Bring_Legs,&Controlled_State,leg_ptr,joint_m_ptr);
	YAW_Parameter_Processing(&Flag,&Goal_Setting,&Controlled_State,&Body);
	Bump_Control(&Flag,&Goal_Setting,leg_ptr,joint_m_ptr);
	Leg_Control(&INS,&Flag,&Bring_Legs,&Goal_Setting,&Self_Rescue,&Controlled_State,leg_ptr,joint_m_ptr,five_link_ptr);
	LQR(&Flag,&Goal_Setting,&Controlled_State,&Compensation_Amount,&Body,leg_ptr,joint_m_ptr,five_link_ptr);
	Vmc(&Flag,joint_m_ptr,five_link_ptr);
	Chassis_Can_Data_Send(&Chassis_Motor,&Controlled_State,joint_m_ptr);
}

/*******************************************************************************************************
底盘关键数据获取
********************************************************************************************************/
void Variable_Information_Acquisition(INS_t *ins,
																			Chassis_Motor_t *cm,
                                    	Body_Current_Situation_t *body,
																			Leg_Current_Situation_t *leg[2],
																			Vmc_Five_Link_Parameter_t *five_link[2])
{
	//轮子转速(仅仅为转子相对于定子的速度,而非定子相对于地面的速度)
	leg[0]->last_wheel_s =   leg[0]->wheel_s;
	leg[0]->wheel_s      = -(cm->Dji_3508[0].speed_rpm/Gear_Ratio/60.0f)*2.0f*PI*R_wheel;
	leg[1]->last_wheel_s =   leg[1]->wheel_s;
	leg[1]->wheel_s      =  (cm->Dji_3508[1].speed_rpm/Gear_Ratio/60.0f)*2.0f*PI*R_wheel;
	
	//机体数据获取
	body->yaw     =  ins->Yaw*Ang_PI;
	body->d_yaw   =  ins->Gyro[2];
	body->theta   = -ins->Roll*Ang_PI;
	body->d_theta = -ins->Gyro[1];
	body->abs_yaw = Half_Circle_RADIAN((Up_Cboard_Info.ecd_yaw_6020/8191.0f)*2*PI);
	
	//腿部数据获取
	//bz:phi1和phi4都需要分别归一化到域值[0,2PI]和[-PI,PI]
	five_link[0]->phi1 = PI-cm->DM_8009[1].pos; five_link[1]->phi1 = PI-cm->DM_8009[2].pos;
	five_link[0]->phi4 =  0-cm->DM_8009[0].pos; five_link[1]->phi4 =  0-cm->DM_8009[3].pos;
	five_link[0]->d_phi1 = -cm->DM_8009[1].vel; five_link[1]->d_phi1 = -cm->DM_8009[2].vel;                                      
	five_link[0]->d_phi4 = -cm->DM_8009[0].vel; five_link[1]->d_phi4 = -cm->DM_8009[3].vel; 
	
	Leg_Calc(five_link[0]); Leg_Calc(five_link[1]);	
	
	leg[0]->theta   = Half_Circle_RADIAN(-five_link[0]->phi0 + (PI/2.0f) + body->theta); 
	leg[1]->theta   = Half_Circle_RADIAN( five_link[1]->phi0 - (PI/2.0f) + body->theta);    
	leg[0]->d_theta = -five_link[0]->d_phi0 + body->d_theta;  
	leg[1]->d_theta =  five_link[1]->d_phi0 + body->d_theta; 
	
	leg[0]->abs_leg_theta  = Half_Circle_RADIAN(-five_link[0]->phi0 + (PI/2.0f));
	leg[1]->abs_leg_theta  = Half_Circle_RADIAN( five_link[1]->phi0 - (PI/2.0f));
	
	leg[0]->Fn = Positive_Number_Out(((M_car*9.8f)/2.0f)*arm_cos_f32(leg[0]->theta));       
	leg[1]->Fn = Positive_Number_Out(((M_car*9.8f)/2.0f)*arm_cos_f32(leg[1]->theta));
 	leg[0]->fl = fabs((0.5f*(FL*arm_sin_f32(five_link[0]->alpha[0])))*arm_sin_f32(PI-0.5f*five_link[0]->alpha[1]-five_link[0]->alpha[0]));
	leg[1]->fl = fabs((0.5f*(FL*arm_sin_f32(five_link[1]->alpha[0])))*arm_sin_f32(PI-0.5f*five_link[1]->alpha[1]-five_link[1]->alpha[0]));
	
	//车体的原始速度获取
	leg[0]->stator_s = leg[0]->wheel_s + leg[0]->d_theta*R_wheel; //未打滑的理想情况下,定子相对于地面的速度
	leg[1]->stator_s = leg[1]->wheel_s + leg[1]->d_theta*R_wheel;
	leg[0]->swing_s  = five_link[0]->d_L0*arm_sin_f32(leg[0]->theta) + five_link[0]->L0*(leg[0]->d_theta)*arm_cos_f32(leg[0]->theta); //摆杆相对于机体的速度
	leg[1]->swing_s  = five_link[1]->d_L0*arm_sin_f32(leg[1]->theta) + five_link[1]->L0*(leg[1]->d_theta)*arm_cos_f32(leg[1]->theta);
	
	//车体估计值获取
	body->Estimate_dx   = ( leg[0]->stator_s + leg[0]->swing_s
										   	+ leg[1]->stator_s + leg[1]->swing_s )/2.0f; //估计的车体相对于地面速度
											
	body->Estimate_dyaw = (leg[1]->wheel_s - leg[0]->wheel_s)/L_wheel; //估计的车体YAW速度
	
	body->Estimate_h    = 0.5f*(five_link[0]->L0 + five_link[1]->L0);  //估计的车体高度
}

/*******************************************************************************************************
打滑检测
********************************************************************************************************/
void Slip_Check_Calc(Flag_Bit_t *flag,
										 Slip_Check_t *slip,
										 Body_Current_Situation_t *body,
								     Leg_Current_Situation_t *leg[2])
{
	//获取加速度估计以及轮部速度估计的瞬时速度
	slip->dt = DWT_GetDeltaT(&slip->dwt_d_slip);
	slip->dv_acc = fabs(body->a_x*slip->dt); 
	
	slip->dv_wheel[0] = fabs(Mean_Filtering(leg[0]->wheel_s - leg[0]->last_wheel_s,slip->wheel_L_aver));
	slip->dv_wheel[1] = fabs(Mean_Filtering(leg[1]->wheel_s - leg[1]->last_wheel_s,slip->wheel_R_aver));

	//判断车体是否打滑(小陀螺时关闭打滑检测)
	if( ( fabs(body->Estimate_dyaw - body->d_yaw) >= DBY_THRESHOLD   ||
		    fabs(leg[0]->wheel_s) - fabs(body->d_x) >= DVB_THRESHOLD   ||
	      fabs(leg[1]->wheel_s) - fabs(body->d_x) >= DVB_THRESHOLD ) &&
			  flag->spinning_flag == 0                                      )
	{
		if(slip->dv_wheel[0] - slip->dv_acc >= DVW_THRESHOLD && flag->slip_flag[0] == 0) 
		{ 
			flag->slip_flag[0] = 1; slip->f_dv_wheel[0] = fabs(leg[0]->wheel_s);
		}
		else if(fabs(leg[0]->wheel_s) <= slip->f_dv_wheel[0] && flag->slip_flag[0] == 1) flag->slip_flag[0] = 0; 

		if(slip->dv_wheel[1] - slip->dv_acc >= DVW_THRESHOLD && flag->slip_flag[1] == 0) 
		{ 
			flag->slip_flag[1] = 1; slip->f_dv_wheel[1] = fabs(leg[1]->wheel_s);
		}
		else if(fabs(leg[1]->wheel_s) <= slip->f_dv_wheel[1] && flag->slip_flag[1] == 1) flag->slip_flag[1] = 0; 
		
		body->R_turn = 0;
	}
	else
	{
		flag->slip_flag[0] = 0;
		flag->slip_flag[1] = 0;
		
		if(fabs(leg[0]->wheel_s - leg[1]->wheel_s) >= LRW_THRESHOLD)
		{
			body->R_turn = fabs((L_wheel*(leg[0]->wheel_s + leg[1]->wheel_s))/
										      (   2.0f*(leg[0]->wheel_s - leg[1]->wheel_s)));
		}  
		else body->R_turn = 0;
	}
	
	body->a_x = -INS.MotionAccel_b[0] + body->d_yaw*body->d_yaw*body->R_turn;
}

/*******************************************************************************************************
车体倾角以及腿杆位置异常判断
********************************************************************************************************/
void All_Theta_Err_Check(INS_t *ins,
												 Flag_Bit_t *flag,
												 Leg_Current_Situation_t *leg[2],
												 Joint_Motor_Status_t *joint_m[2])
{
	static bool bump_down_flag[2] = {0,0};	
	
	//车体是否翻倒判断
	if(flag->bump_flag) //进入磕台阶模式提高倒地判断阈值
	{
		if(leg[0]->abs_leg_theta<=LEG_THETA_HEAD_MAX||leg[0]->abs_leg_theta>=BUMP_LEG_THETA_BACK_MAX||
			 leg[1]->abs_leg_theta<=LEG_THETA_HEAD_MAX||leg[1]->abs_leg_theta>=BUMP_LEG_THETA_BACK_MAX|| 
			 fabs(ins->Pitch) >= 90) flag->fall_flag = 1;
	}
	else
	{
		if(leg[0]->abs_leg_theta<=LEG_THETA_HEAD_MAX||leg[0]->abs_leg_theta>=LEG_THETA_BACK_MAX||
			 leg[1]->abs_leg_theta<=LEG_THETA_HEAD_MAX||leg[1]->abs_leg_theta>=LEG_THETA_BACK_MAX|| 
			 fabs(ins->Pitch) >= 90) flag->fall_flag = 1;
	}
	
	if(flag->bump_flag) //磕台阶模式--开始判断是否磕到台阶
	{
		if((fabs(joint_m[0]->T_Wheel)>=BUMP_TWHEEL_MAX) && (fabs(Leg[0].theta)>=BUMP_LEG_THETA_MAX)) bump_down_flag[0] = 1;
		if((fabs(joint_m[1]->T_Wheel)>=BUMP_TWHEEL_MAX) && (fabs(Leg[1].theta)>=BUMP_LEG_THETA_MAX)) bump_down_flag[1] = 1;
		
		if(bump_down_flag[0] && bump_down_flag[1]) flag->theta_flag[2] = 1;
	}
	else
	{
		bump_down_flag[0] = 0;
		bump_down_flag[1] = 0;
		
		flag->theta_flag[2] = 0;
	}
}

void Goal_Site(Flag_Bit_t *flag,
							 RC_Ctrl_t *rc_ctrl,
							 Goal_Setting_t *goal,
							 Body_Current_Situation_t *body)
{
	static float follow_theta = 0;
	
	if(body->Estimate_h<=0.20f) //根据不同腿长控制x以及yaw速度上限
	{
		goal->MAX_Dx = DX_DOWN_MAX; goal->MAX_Dyaw = DYAW_DOWN_MAX;
	}
	else if(body->Estimate_h>0.20f && body->Estimate_h<=0.30f)
	{
		goal->MAX_Dx = DX_MID_MAX;  goal->MAX_Dyaw = DYAW_MID_MAX;
	}
	else if(body->Estimate_h>0.30f)
	{
		goal->MAX_Dx = DX_UP_MAX;   goal->MAX_Dyaw = DYAW_UP_MAX;
	}
	
	//获取跟随角度
	if(flag->spinning_flag)
	{
		if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_A) || (rc_ctrl->key.v&KEY_PRESSED_OFFSET_D))
		{
			follow_theta = Find_Min_RADIAN(body->abs_yaw,ZERO_HEAD_YAW+PI/2.0f);
		}
		else follow_theta = Find_Min_RADIAN(body->abs_yaw,ZERO_HEAD_YAW); 
	}
	else follow_theta = Find_Min_RADIAN(body->abs_yaw,ZERO_HEAD_YAW);
	
	//遥控器x及yaw速度设定值
	goal->Rc_X   = Max_Output((Limit_Min(rc_ctrl->rc.ch[1],RC_DEADBAND)/RC_DX )*arm_cos_f32(follow_theta),goal->MAX_Dx  ); 
	goal->Rc_Yaw = Max_Output (Limit_Min(rc_ctrl->rc.ch[4],RC_DEADBAND)/RC_YAW                           ,goal->MAX_Dyaw);
	
	if(Remote_Select == UNLINK) //未连接遥控器
	{
		goal->Pc_X   = 0;
		goal->Pc_Yaw = 0;
	}
	else if(Remote_Select == DT7) //连接DT7遥控器
	{
		//键鼠x及yaw速度设定值
		if(flag->spinning_flag)
		{
			if     (rc_ctrl->key.v&KEY_PRESSED_OFFSET_A) goal->Pc_X = Max_Output( goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx);
			else if(rc_ctrl->key.v&KEY_PRESSED_OFFSET_D) goal->Pc_X = Max_Output(-goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx);
			else if(rc_ctrl->key.v&KEY_PRESSED_OFFSET_W) goal->Pc_X = Max_Output(-goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx); 
			else if(rc_ctrl->key.v&KEY_PRESSED_OFFSET_S) goal->Pc_X = Max_Output( goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx);
			else                                         goal->Pc_X = 0;
		}
		else
		{
			if     (rc_ctrl->key.v&KEY_PRESSED_OFFSET_W) goal->Pc_X = Max_Output( goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx); 
			else if(rc_ctrl->key.v&KEY_PRESSED_OFFSET_S) goal->Pc_X = Max_Output(-goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx);
			else                                         goal->Pc_X = 0;
		}

		goal->Pc_Yaw = Max_Output(goal->MAX_Dyaw,goal->MAX_Dyaw);
	}
	else if(Remote_Select == VT_03) //连接VT03遥控器
	{
		//键鼠x及yaw速度设定值
		if(flag->spinning_flag)
		{
			if     (VT03.key&KEY_PRESSED_OFFSET_A) goal->Pc_X = Max_Output( goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx);
			else if(VT03.key&KEY_PRESSED_OFFSET_D) goal->Pc_X = Max_Output(-goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx);
			else if(VT03.key&KEY_PRESSED_OFFSET_W) goal->Pc_X = Max_Output(-goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx); 
			else if(VT03.key&KEY_PRESSED_OFFSET_S) goal->Pc_X = Max_Output( goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx);
			else                                   goal->Pc_X = 0;
		}
		else
		{
			if     (VT03.key&KEY_PRESSED_OFFSET_W) goal->Pc_X = Max_Output( goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx); 
			else if(VT03.key&KEY_PRESSED_OFFSET_S) goal->Pc_X = Max_Output(-goal->MAX_Dx*arm_cos_f32(follow_theta),goal->MAX_Dx);
			else                                   goal->Pc_X = 0;
		}

		goal->Pc_Yaw = Max_Output(goal->MAX_Dyaw,goal->MAX_Dyaw);
	}
}	

/*******************************************************************************************************
底盘控制各项参数初始化
********************************************************************************************************/
void Chassis_Control_Init(Flag_Bit_t *flag,
												  Goal_Setting_t *goal)
{
	goal->d_x_t     = 0;               //初始化x轴速度
	goal->d_yaw_t   = 0;		         	 //初始化yaw速度
	goal->Target_L0 = DOWN_LEG_LENGTH; //初始化腿长
	
	flag->bump_flag     = 0; //关闭磕台阶
	flag->change_flag   = 0; //关闭车体反转
	flag->spinning_flag = 0; //关闭小陀螺
}

/*******************************************************************************************************
遥控器模式
********************************************************************************************************/
void Rc_Mode(Flag_Bit_t *flag,
						 RC_Ctrl_t *rc_ctrl,
						 Goal_Setting_t *goal)
{
//	static bool first_bump_flag = 0;

//	if     (switch_is_down(rc_ctrl->rc.s[1])) { goal->Target_L0 = DOWN_LEG_LENGTH; } //腿长控制
//	else if(switch_is_mid (rc_ctrl->rc.s[1])) { goal->Target_L0 = MID_LEG_LENGTH;  }						

//	if(switch_is_up(rc_ctrl->rc.s[1]) && first_bump_flag == 0) //磕台阶模式
//	{
//		flag->bump_flag = 1;
//		first_bump_flag = 1;
//	}
//	else if(!switch_is_up(rc_ctrl->rc.s[1]))
//	{
//		flag->bump_flag = 0; //复位
//		first_bump_flag = 0; 
//	}
	
	flag->bump_flag = 0;
	goal->Target_L0 = DOWN_LEG_LENGTH;
}

/*******************************************************************************************************
键鼠模式初始化
********************************************************************************************************/
void Pc_Init(PC_Ctrl_t *pc_ctrl)
{
	pc_ctrl->E = 0;
	pc_ctrl->F = 0;
	pc_ctrl->R = 0;	
	pc_ctrl->CTRL = 0;
	
	pc_ctrl->e_t = 0;
	pc_ctrl->f_t = 0;
	pc_ctrl->r_t = 0;
	pc_ctrl->ctrl_t = 0;
}

/*******************************************************************************************************
键鼠模式
********************************************************************************************************/
void Pc_Mode(Flag_Bit_t *flag,
						 RC_Ctrl_t *rc_ctrl,
						 PC_Ctrl_t *pc_ctrl,
						 Goal_Setting_t *goal)
{
	/******Z键检测,刷新UI******/
	if(rc_ctrl->key.v&KEY_PRESSED_OFFSET_Z) Rest_UI_Flag = 1;
	else                                    Rest_UI_Flag = 0;
	
	/******R键检测,转换正方向******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_R) && !pc_ctrl->R)
	{
		pc_ctrl->R = 1;
		pc_ctrl->r_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_R) && pc_ctrl->R)
	{
		pc_ctrl->R = 0;
		if(HAL_GetTick()-pc_ctrl->r_t < 500)
		{
			if(!flag->off_flag      &&
				 !flag->bump_flag     &&
	       !flag->spinning_flag   )
			{
				if(flag->change_flag) flag->change_flag = 0;
				else                  flag->change_flag = 1;
			}
			else flag->change_flag = 0;
		}
	}
	
	/******E键检测,小陀螺******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_E) && !pc_ctrl->E)
	{
		pc_ctrl->E = 1;
		pc_ctrl->e_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_E) && pc_ctrl->E)
	{
		pc_ctrl->E = 0;
		if(HAL_GetTick()-pc_ctrl->e_t < 500)
		{
			if(!flag->off_flag  &&
				 !flag->bump_flag   )
			{
				if(flag->spinning_flag) 
				{
					flag->change_flag   = 0;
					flag->spinning_flag = 0;
				}
				else 
				{
					flag->change_flag   = 0;
					flag->spinning_flag = 1;
				}
			}
			else flag->spinning_flag = 0;
		}
	}
	
	/******F键检测,切换腿长******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_F) && !pc_ctrl->F)
	{
		pc_ctrl->F = 1;
		pc_ctrl->f_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_F) && pc_ctrl->F)
	{
		pc_ctrl->F = 0;
		if(HAL_GetTick()-pc_ctrl->f_t < 500)
		{
			if(goal->Target_L0 == DOWN_LEG_LENGTH) goal->Target_L0 = MID_LEG_LENGTH;
			else                                   goal->Target_L0 = DOWN_LEG_LENGTH;
		}
	}
	else
	{
		if(goal->Target_L0!=DOWN_LEG_LENGTH && goal->Target_L0!=MID_LEG_LENGTH && goal->Target_L0!=UP_LEG_LENGTH) //异常腿长目标
		{
			goal->Target_L0 = DOWN_LEG_LENGTH;
		}
	}
	
	/******CTRL键检测,进入磕台阶模式******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_CTRL) && !pc_ctrl->CTRL)
	{
		pc_ctrl->CTRL = 1;
		pc_ctrl->ctrl_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_CTRL) && pc_ctrl->CTRL)
	{
		pc_ctrl->CTRL = 0;
		if(HAL_GetTick()-pc_ctrl->ctrl_t < 500)
		{
			if(!flag->fall_flag     &&  
			   !flag->change_flag   &&
				 !flag->slip_flag[0]  &&
			   !flag->slip_flag[0]  &&
				 !flag->theta_flag[0] &&
				 !flag->theta_flag[1] &&
	       !flag->spinning_flag   )
			{
				if(!flag->bump_flag) flag->bump_flag = 1;
				else 
				{
					flag->bump_flag = 0;
					goal->Target_L0 = DOWN_LEG_LENGTH;
				}
			}
			else flag->bump_flag = 0;
		}
	}
}

/*******************************************************************************************************
VT03的键鼠模式
********************************************************************************************************/
void Vt03_Pc_Mode(Flag_Bit_t *flag,
									PC_Ctrl_t *pc_ctrl,
									Goal_Setting_t *goal)
{
	/******Z键检测,刷新UI******/
	if(VT03.key&KEY_PRESSED_OFFSET_Z) Rest_UI_Flag = 1;
	else                              Rest_UI_Flag = 0;
	
	/******R键检测,转换正方向******/
	if((VT03.key&KEY_PRESSED_OFFSET_R) && !pc_ctrl->R)
	{
		pc_ctrl->R = 1;
		pc_ctrl->r_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_R) && pc_ctrl->R)
	{
		pc_ctrl->R = 0;
		if(HAL_GetTick()-pc_ctrl->r_t < 500)
		{
			if(!flag->off_flag      &&
				 !flag->bump_flag     &&
	       !flag->spinning_flag   )
			{
				if(flag->change_flag) flag->change_flag = 0;
				else                  flag->change_flag = 1;
			}
			else flag->change_flag = 0;
		}
	}
	
	/******E键检测,小陀螺******/
	if((VT03.key&KEY_PRESSED_OFFSET_E) && !pc_ctrl->E)
	{
		pc_ctrl->E = 1;
		pc_ctrl->e_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_E) && pc_ctrl->E)
	{
		pc_ctrl->E = 0;
		if(HAL_GetTick()-pc_ctrl->e_t < 500)
		{
			if(!flag->off_flag  &&
				 !flag->bump_flag   )
			{
				if(flag->spinning_flag) 
				{
					flag->change_flag   = 0;
					flag->spinning_flag = 0;
				}
				else 
				{
					flag->change_flag   = 0;
					flag->spinning_flag = 1;
				}
			}
			else flag->spinning_flag = 0;
		}
	}
	
	/******F键检测,切换腿长******/
	if((VT03.key&KEY_PRESSED_OFFSET_F) && !pc_ctrl->F)
	{
		pc_ctrl->F = 1;
		pc_ctrl->f_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_F) && pc_ctrl->F)
	{
		pc_ctrl->F = 0;
		if(HAL_GetTick()-pc_ctrl->f_t < 500)
		{
			if(goal->Target_L0 == DOWN_LEG_LENGTH) goal->Target_L0 = MID_LEG_LENGTH;
			else                                   goal->Target_L0 = DOWN_LEG_LENGTH;
		}
	}
	else
	{
		if(goal->Target_L0!=DOWN_LEG_LENGTH && goal->Target_L0!=MID_LEG_LENGTH && goal->Target_L0!=UP_LEG_LENGTH) //异常腿长目标
		{
			goal->Target_L0 = DOWN_LEG_LENGTH;
		}
	}
	
	/******CTRL键检测,进入磕台阶模式******/
	if((VT03.key&KEY_PRESSED_OFFSET_CTRL) && !pc_ctrl->CTRL)
	{
		pc_ctrl->CTRL = 1;
		pc_ctrl->ctrl_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_CTRL) && pc_ctrl->CTRL)
	{
		pc_ctrl->CTRL = 0;
		if(HAL_GetTick()-pc_ctrl->ctrl_t < 500)
		{
			if(!flag->fall_flag     &&  
			   !flag->change_flag   &&
				 !flag->slip_flag[0]  &&
			   !flag->slip_flag[0]  &&
				 !flag->theta_flag[0] &&
				 !flag->theta_flag[1] &&
	       !flag->spinning_flag   )
			{
				if(!flag->bump_flag) flag->bump_flag = 1;
				else 
				{
					flag->bump_flag = 0;
					goal->Target_L0 = DOWN_LEG_LENGTH;
				}
			}
			else flag->bump_flag = 0;
		}
	}
}

/*******************************************************************************************************
触发倒地自救
********************************************************************************************************/
void DT7_Start_Selfsave(INS_t *ins,
									      RC_Ctrl_t *rc_ctrl,
									     	Self_Rescue_t *self_re,
									    	Body_Current_Situation_t *body)
{
	if(((rc_ctrl->key.v&KEY_PRESSED_OFFSET_R)||rc_ctrl->rc.ch[1]==-660) && self_re->run_flag==0)//触发自救
	{
		//获取当前倒地状态[0车身正 1车身倒头着地 2屁股着地]
		if(fabs(ins->Pitch) >= 90) 
		{
			if(ins->Roll <= 0) self_re->downed_state = 1;
			else self_re->downed_state = 2;
		}
		else self_re->downed_state = 0;	
		
		//选择先收哪条腿[0左 1右]
		if(body->abs_yaw>=ZERO_BACK_YAW && body->abs_yaw<=ZERO_HEAD_YAW) self_re->run_first = 0;
		else self_re->run_first = 1;
		
		//获取初始绝对腿杆位置
		self_re->init_pos[0] = Leg[0].abs_leg_theta; self_re->init_pos[1] = Leg[1].abs_leg_theta;
		self_re->goal_pos[0] = Leg[0].abs_leg_theta; self_re->goal_pos[1] = Leg[1].abs_leg_theta;
		//复原步骤清零
		self_re->step[0][0] = 0; self_re->step[0][1] = 0; self_re->step[0][2] = 0;
		self_re->step[1][0] = 0; self_re->step[1][1] = 0; self_re->step[1][2] = 0;
		//标志位初始化
		self_re->col_flag[0] = 0; self_re->col_flag[1] = 0;
		//开启自救
		self_re->run_flag = 1;
	}
}

void VT04_Start_Selfsave(INS_t *ins,
									       Self_Rescue_t *self_re,
										     Body_Current_Situation_t *body)
{
	if((VT03.key&KEY_PRESSED_OFFSET_R) && self_re->run_flag==0)//触发自救
	{
		//获取当前倒地状态[0车身正 1车身倒头着地 2屁股着地]
		if(fabs(ins->Pitch) >= 90) 
		{
			if(ins->Roll <= 0) self_re->downed_state = 1;
			else self_re->downed_state = 2;
		}
		else self_re->downed_state = 0;	
		
		//选择先收哪条腿[0左 1右]
		if(body->abs_yaw>=ZERO_BACK_YAW && body->abs_yaw<=ZERO_HEAD_YAW) self_re->run_first = 0;
		else self_re->run_first = 1;
		
		//获取初始绝对腿杆位置
		self_re->init_pos[0] = Leg[0].abs_leg_theta; self_re->init_pos[1] = Leg[1].abs_leg_theta;
		self_re->goal_pos[0] = Leg[0].abs_leg_theta; self_re->goal_pos[1] = Leg[1].abs_leg_theta;
		//复原步骤清零
		self_re->step[0][0] = 0; self_re->step[0][1] = 0; self_re->step[0][2] = 0;
		self_re->step[1][0] = 0; self_re->step[1][1] = 0; self_re->step[1][2] = 0;
		//标志位初始化
		self_re->col_flag[0] = 0; self_re->col_flag[1] = 0;
		//开启自救
		self_re->run_flag = 1;
	}
}

/*******************************************************************************************************
底盘控制逻辑
********************************************************************************************************/
void Chassis_Control(INS_t *ins,
										 Flag_Bit_t *flag,
										 RC_Ctrl_t *rc_ctrl,
										 PC_Ctrl_t *pc_ctrl,
										 Goal_Setting_t *goal,
										 Self_Rescue_t *self_re,
										 Controlled_State_t *cs,
										 Body_Current_Situation_t *body,
										 Leg_Current_Situation_t *leg[2])
{	
	Goal_Site(flag,rc_ctrl,goal,body);

	if(*cs!=ERO && *cs!=STOP)
	{
		if(Remote_Select == UNLINK) //未连接遥控器
		{
			self_re->run_flag = 0; //初始化翻到自救参数
		
			Pc_Init(pc_ctrl); //初始化
			Chassis_Control_Init(flag,goal);
		}
		else if(Remote_Select == DT7) //连接DT7遥控器
		{
			if(flag->fall_flag) //倒地
			{
				Pc_Init(pc_ctrl); //初始化
				Chassis_Control_Init(flag,goal);
				
				DT7_Start_Selfsave(ins,rc_ctrl,self_re,body);
				
				Save_Self(flag,self_re,leg); //自救
			}
			else //正常
			{
				if(Controlled_State == RC) //遥控器模式
				{
					Pc_Init(pc_ctrl); 
					Rc_Mode(flag,rc_ctrl,goal);
					
					if(goal->Rc_Yaw != 0) //小陀螺判定
					{
						flag->spinning_flag = 1;
						
						goal->d_x_t   = Ramp_Function(           0,&goal->d_x_t  ,RC_DX_RAMP_SENS);
						goal->d_yaw_t = Ramp_Function(goal->Rc_Yaw,&goal->d_yaw_t,RC_DYAW_RAMP_SENS);
					}
					else
					{
						flag->spinning_flag = 0;
						
						goal->d_x_t   = Ramp_Function(goal->Rc_X,&goal->d_x_t  ,RC_DX_RAMP_SENS);
						goal->d_yaw_t = Ramp_Function(         0,&goal->d_yaw_t,RC_DYAW_RAMP_SENS);
					}	
				}
				else if(Controlled_State == MOUSE) //键鼠模式
				{				
					Pc_Mode(flag,rc_ctrl,pc_ctrl,goal);
					
					if(flag->spinning_flag) //小陀螺判定
					{
						goal->d_x_t   = Ramp_Function(            0,&goal->d_x_t  ,PC_DX_RAMP_SENS);
						goal->d_yaw_t = Ramp_Function( goal->Pc_Yaw,&goal->d_yaw_t,PC_DYAW_RAMP_SENS);
					}
					else
					{
						goal->d_x_t   = Ramp_Function(   goal->Pc_X,&goal->d_x_t  ,PC_DX_RAMP_SENS);
						goal->d_yaw_t = Ramp_Function(            0,&goal->d_yaw_t,PC_DYAW_RAMP_SENS);
					}		
				}
				
				self_re->run_flag = 0; //初始化翻到自救参数
			}
		}
		else if(Remote_Select == VT_03) //连接VT03遥控器
		{
			if(flag->fall_flag) //倒地
			{
				Pc_Init(pc_ctrl); //初始化
				Chassis_Control_Init(flag,goal);
				
				VT04_Start_Selfsave(ins,self_re,body);
				
				Save_Self(flag,self_re,leg); //自救
			}
			else //正常
			{
				if(Controlled_State == MOUSE) //键鼠模式
				{				
					Vt03_Pc_Mode(flag,pc_ctrl,goal);
					
					if(flag->spinning_flag) //小陀螺判定
					{
						goal->d_x_t   = Ramp_Function(            0,&goal->d_x_t  ,PC_DX_RAMP_SENS);
						goal->d_yaw_t = Ramp_Function( goal->Pc_Yaw,&goal->d_yaw_t,PC_DYAW_RAMP_SENS);
					}
					else
					{
						goal->d_x_t   = Ramp_Function(   goal->Pc_X,&goal->d_x_t  ,PC_DX_RAMP_SENS);
						goal->d_yaw_t = Ramp_Function(            0,&goal->d_yaw_t,PC_DYAW_RAMP_SENS);
					}		
				}
				
				self_re->run_flag = 0; //初始化翻到自救参数
			}
		}
	}
	else
	{
		self_re->run_flag = 0; //初始化翻到自救参数
		
		Pc_Init(pc_ctrl); //初始化
		Chassis_Control_Init(flag,goal);
	}
}

/*******************************************************************************************************
车体速度处理
********************************************************************************************************/
void Fast_Processing(Flag_Bit_t *flag,
										 Goal_Setting_t *goal,
										 Controlled_State_t *cs,
										 Body_Current_Situation_t *body,
										 Speed_Fusion_Parameter_t *sp_fusion)
{
	static bool start_situate_flag = 0; //开启机体定位标志
	
	//估计速度与加速度融合
	sp_fusion->actual_acc = body->a_x;
	sp_fusion->actual_vel = body->Estimate_dx;
	
	V_Kf_Update(&V_kf,sp_fusion,DWT_GetDeltaT(&sp_fusion->dwt_v_kf));
	
	body->d_x = sp_fusion->filter_vel;
	
	if(*cs!=ERO             && 
		 *cs!=STOP            &&
		 !flag->off_flag      && 
		 !flag->fall_flag		  && 
		 !flag->slip_flag[0]  &&
		 !flag->slip_flag[1]  &&
		 !flag->theta_flag[0] &&
	   !flag->theta_flag[1] &&
		 !flag->theta_flag[2] &&
	   !flag->spinning_flag   )
	{
		if(fabs(goal->d_x_t)>=0.05f)
		{
			start_situate_flag = 0;
			
			body->x = 0;
		}
		else if((fabs(goal->d_x_t)<=0.05f && fabs(body->d_x)<=0.2f) || start_situate_flag) //开启机体定位
		{
			start_situate_flag = 1;
			
//			body->x = 0;
			body->x += body->d_x*0.008f;
		}

	}
	else
	{
		start_situate_flag = 0;
		
		body->x = 0;
	}
}

/*******************************************************************************************************
卡腿异常判断
********************************************************************************************************/
void Check_Stuck_Leg(Flag_Bit_t *flag,
										 Bring_Legs_t *bring,
										 Controlled_State_t *cs,
										 Leg_Current_Situation_t *leg[2],
								     Joint_Motor_Status_t *joint_m[2])
{
	//判断卡腿并计算卡腿时长
	if(*cs!=ERO             &&
		 *cs!=STOP            &&
		 !flag->off_flag      && 
		 !flag->fall_flag		  &&
		 !flag->theta_flag[2]    )
	{
		if((fabs(joint_m[0]->T_Wheel)>=BRING_TWHEEL_MAX) && (fabs(Leg[0].theta)>=BRING_LEG_THETA_MAX)) bring->stuck_time[0]++;
		else if(fabs(Leg[0].theta)<=BRING_LEG_THETA_MIN) bring->stuck_time[0] = 0;
		if((fabs(joint_m[1]->T_Wheel)>=BRING_TWHEEL_MAX) && (fabs(Leg[1].theta)>=BRING_LEG_THETA_MAX)) bring->stuck_time[1]++;
		else if(fabs(Leg[1].theta)<=BRING_LEG_THETA_MIN) bring->stuck_time[1] = 0;
	}
	else
	{
		bring->stuck_time[0] = 0;
		bring->stuck_time[1] = 0;
	}
	
	if(bring->stuck_time[0]>=BST_THRESHOLD)
	{
		flag->theta_flag[0] = 1;
		
		bring->Comp_Fn[0] = Max_Output(bring->stuck_time[0]*BRING_COMP_FN_COEF,BRING_COMP_FN_MAX);
		bring->Comp_Length[0] = bring->stuck_time[0]*BRING_COMP_TL_COEF;
	}
	else
	{
		flag->theta_flag[0] = 0;
		
		bring->Comp_Fn[0] = 0;
		bring->Comp_Length[0] = 0;
	}
	
	if(bring->stuck_time[1]>=BST_THRESHOLD)
	{
		flag->theta_flag[1] = 1;
		
		bring->Comp_Fn[1] = Max_Output(bring->stuck_time[1]*BRING_COMP_FN_COEF,BRING_COMP_FN_MAX);
		bring->Comp_Length[1] = bring->stuck_time[1]*BRING_COMP_TL_COEF;
	}
	else
	{
		flag->theta_flag[1] = 0;
		
		bring->Comp_Fn[1] = 0;
		bring->Comp_Length[1] = 0;
	}
}

/*******************************************************************************************************
对车体YAW相关的参数进行处理
********************************************************************************************************/
void YAW_Parameter_Processing(Flag_Bit_t *flag,
														  Goal_Setting_t *goal,
															Controlled_State_t *cs,
															Body_Current_Situation_t *body)
{					
	static bool start_change_flag = 0; //开始转身标志位
	
	static float dis_to_head = 0; //到正方向零点位置最小角度值
	static float dis_to_back = 0; //到负方向零点位置最小角度值
	
	dis_to_head = fabs(Find_Min_RADIAN(body->abs_yaw,ZERO_HEAD_YAW));
	dis_to_back = fabs(Find_Min_RADIAN(body->abs_yaw,ZERO_BACK_YAW));
	
	if(*cs!=ERO && *cs!=STOP)
	{
		if(!flag->change_flag)
		{
			if(dis_to_head <= dis_to_back) goal->yaw_t = ZERO_HEAD_YAW;
		  else goal->yaw_t = ZERO_BACK_YAW;
			
			start_change_flag = 0; //重置
		}
		else
		{
			if(!start_change_flag)
			{
				if(goal->yaw_t == ZERO_HEAD_YAW) goal->yaw_t = ZERO_BACK_YAW;
				else goal->yaw_t = ZERO_HEAD_YAW;
				
				start_change_flag = 1;
			}
			
			if(start_change_flag)
			{
				if(goal->yaw_t == ZERO_HEAD_YAW)
				{
					if(dis_to_head <= dis_to_back)
					{
						flag->change_flag = 0;
						start_change_flag = 0;
					}
				}
				else
				{
					if(dis_to_head > dis_to_back)
					{
						flag->change_flag = 0;
						start_change_flag = 0;
					}
				}
			}
		}
	}
	else 
	{
		goal->yaw_t = body->abs_yaw; 
	}
}

/*******************************************************************************************************
磕台阶控制
********************************************************************************************************/
void Bump_Control(Flag_Bit_t *flag,
									Goal_Setting_t *goal,
									Leg_Current_Situation_t *leg[2],
									Joint_Motor_Status_t *joint_m[2])
{
	static int bump_step[2] = {0,0};
	
	if(flag->bump_flag)
	{
		if(flag->theta_flag[2]) 
		{
			goal->Target_L0 = RETRACT_LEG_LENGTH; //上台阶收腿
			
			//左腿
			switch(bump_step[0])
			{
				case 0:
				{
					if(fabs(leg[0]->abs_leg_theta)>=1.32f)
					{
						PID_Calculate(&Leg_P_Pid[0],leg[0]->abs_leg_theta,1.22f);
						joint_m[0]->Tp = Leg_P_Pid[0].Output;
					}
					else joint_m[0]->Tp = BACK_LEG_TP;
					
					if(fabs(leg[0]->theta)>=1.2f) bump_step[0] = 1;
					break;
				}
				case 1:
				{		
					if(fabs(leg[0]->theta)>=1.25f)
					{
						PID_Calculate(&Leg_P_Pid[0],leg[0]->abs_leg_theta,1.12f);
						joint_m[0]->Tp = Leg_P_Pid[0].Output;
					}
					else joint_m[0]->Tp = FRONT_LEG_TP;
					
					if(leg[0]->theta<=0.55f) bump_step[0] = 2;
					break;
				}
				case 2:
				{
					PID_Calculate(&Leg_P_Pid[0],leg[0]->abs_leg_theta,0.18f);
					joint_m[0]->Tp = Leg_P_Pid[0].Output;
					break;
				}
				default:
				{
					break;
				}
			}
			
			//右腿
			switch(bump_step[1])
			{
				case 0:
				{
					if(fabs(leg[1]->abs_leg_theta)>=1.32f)
					{
						PID_Calculate(&Leg_P_Pid[1],leg[1]->abs_leg_theta,1.22f);
						joint_m[1]->Tp = Leg_P_Pid[1].Output;
					}
					else joint_m[1]->Tp = BACK_LEG_TP;
					
					if(fabs(leg[1]->theta)>=1.2f) bump_step[1] = 1;
					break;
				}
				case 1:
				{	
					joint_m[1]->Tp = FRONT_LEG_TP;	
					if(leg[1]->theta<=0.55f) bump_step[1] = 2;
					break;
				}
				case 2:
				{
					PID_Calculate(&Leg_P_Pid[1],leg[1]->abs_leg_theta,0.18f);
					joint_m[1]->Tp = Leg_P_Pid[1].Output;
					break;
				}
				default:
				{
					break;
				}
			}
			
			if(bump_step[0] == 2 && bump_step[1] == 2)
			{
				flag->bump_flag = 0; //磕上台阶
			
				goal->Target_L0 = DOWN_LEG_LENGTH; //恢复腿长
			}
		}
		else 
		{
			goal->Target_L0 = UP_LEG_LENGTH; //升高腿长方便磕台阶
			
			bump_step[0] = 0; 
			bump_step[1] = 0;
		}
	}
	else
	{
		bump_step[0] = 0; 
		bump_step[1] = 0;
	}
}

/*******************************************************************************************************
腿部控制
********************************************************************************************************/
void Leg_Control(INS_t *ins,
								 Flag_Bit_t *flag,
								 Bring_Legs_t *bring,
								 Goal_Setting_t *goal,
								 Self_Rescue_t *self_re,
								 Controlled_State_t *cs,
								 Leg_Current_Situation_t *leg[2],
								 Joint_Motor_Status_t *joint_m[2],
								 Vmc_Five_Link_Parameter_t *five_link[2])
{			
	//车体重力方向补偿
	Feedforward_Calculate(&G_Comp[0],leg[0]->Fn - Leg[0].fl);		
	Feedforward_Calculate(&G_Comp[1],leg[1]->Fn - Leg[1].fl);		
	
	//ROLL补偿
	if(!flag->off_flag      && 
		 !flag->fall_flag     && 
		 !flag->theta_flag[2]    )
	{
		PID_Calculate(&Roll_Pid,ins->Pitch*Ang_PI,0);
	}
	else Roll_Pid.Output = 0;
	
	if(*cs!=ERO && *cs!=STOP)
	{
		if(!flag->fall_flag)
		{
			PID_Calculate(&Leg_L_Pid[0],five_link[0]->L0,Min_Output(goal->Target_L0 - bring->Comp_Length[0],0.09f) - Roll_Pid.Output);
			PID_Calculate(&Leg_L_Pid[1],five_link[1]->L0,Min_Output(goal->Target_L0 - bring->Comp_Length[1],0.09f) + Roll_Pid.Output);	

			if(!flag->off_flag) //着地
			{
				joint_m[0]->F0 = -Leg_L_Pid[0].Output - G_Comp[0].Output + bring->Comp_Fn[0];
				joint_m[1]->F0 = -Leg_L_Pid[1].Output - G_Comp[1].Output + bring->Comp_Fn[1];	
			}
			else //离地
			{
				joint_m[0]->F0 = -Leg_L_Pid[0].Output - off_the_ground.fn_comp;
				joint_m[1]->F0 = -Leg_L_Pid[1].Output - off_the_ground.fn_comp;	
			}
		}
		else //车体翻倒
		{
			joint_m[0]->Tp = 0; joint_m[0]->F0 = 0; joint_m[0]->T_Wheel = 0;
			joint_m[1]->Tp = 0; joint_m[1]->F0 = 0; joint_m[1]->T_Wheel = 0;
			
			if(self_re->run_flag == 1)
			{
				//腿杆绝对位置控制
				PID_Calculate(&Leg_P_Pid[0],leg[0]->abs_leg_theta,self_re->goal_pos[0]);
				PID_Calculate(&Leg_P_Pid[1],leg[1]->abs_leg_theta,self_re->goal_pos[1]);
				joint_m[0]->Tp = Leg_P_Pid[0].Output;
				joint_m[1]->Tp = Leg_P_Pid[1].Output;
				
				//收腿 归位
				if(self_re->col_flag[0] == 1)
				{
					PID_Calculate(&Leg_L_Pid[0],five_link[0]->L0,0.12f);
					joint_m[0]->F0 = -Leg_L_Pid[0].Output - G_Comp[0].Output + SELF_RESCUE_FN_COMP;
				}
				if(self_re->col_flag[1] == 1)
				{
					PID_Calculate(&Leg_L_Pid[1],five_link[1]->L0,0.12f);
					joint_m[1]->F0 = -Leg_L_Pid[1].Output - G_Comp[1].Output + SELF_RESCUE_FN_COMP;
				}
			}
		}
	}
}

/*******************************************************************************************************
LQR计算K增益
********************************************************************************************************/
void LQR(Flag_Bit_t *flag,
				 Goal_Setting_t *goal,
				 Controlled_State_t *cs,
	       Compensation_Amount_t *comp,
				 Body_Current_Situation_t *body,
				 Leg_Current_Situation_t *leg[2],
         Joint_Motor_Status_t *joint_m[2],
				 Vmc_Five_Link_Parameter_t *five_link[2])
{
	static float cos_abs_theta = 0;
	
	cos_abs_theta = arm_cos_f32(Find_Min_RADIAN(body->abs_yaw,ZERO_HEAD_YAW));
	
	//计算车体THEATA补偿拟合值
	comp->Gravity_Comp_X = 0.0f;
	
	//小电脑补偿
	comp->nuc_comp = comp->n_theta[2]*body->Estimate_h*body->Estimate_h 
								 + comp->n_theta[1]*body->Estimate_h 
								 + comp->n_theta[0];
	
	if(cos_abs_theta<0) comp->nuc_comp = comp->nuc_comp*0.5f;
	
	//车身补偿
	comp->body_comp = comp->b_theta[2]*body->Estimate_h*body->Estimate_h 
									+ comp->b_theta[1]*body->Estimate_h 
									+ comp->b_theta[0];
	
	if(flag->spinning_flag) comp->Gravity_Comp_Theta = comp->body_comp*1.45f;
	else                    comp->Gravity_Comp_Theta = comp->body_comp + comp->nuc_comp*cos_abs_theta;
	
	//计算拟合K增益
	Fitting_K_Calc(Fitting_K,P,five_link[0]->L0,five_link[1]->L0);	

	if(*cs!=ERO             && 
		 *cs!=STOP            && 
		 !flag->fall_flag     &&
		 !flag->theta_flag[2]    )
	{
		LQR_Calc(flag,goal,comp,body,leg,joint_m);
	}
}

/*******************************************************************************************************
虚拟力映射
********************************************************************************************************/
void Vmc(Flag_Bit_t *flag,
				 Joint_Motor_Status_t *joint_m[2],
				 Vmc_Five_Link_Parameter_t *five_link[2])
{
	VMC_Calc(five_link[0],joint_m[0]->F0, joint_m[0]->Tp,&joint_m[0]->T_A,&joint_m[0]->T_E);
	VMC_Calc(five_link[1],joint_m[1]->F0,-joint_m[1]->Tp,&joint_m[1]->T_A,&joint_m[1]->T_E);
	
	joint_m[0]->A = -Max_Output( (joint_m[0]->T_Wheel/Kt)*819.2f,16000.0f );
	joint_m[1]->A =  Max_Output( (joint_m[1]->T_Wheel/Kt)*819.2f,16000.0f );
	
	//打滑降低轮毅输出
	if(flag->slip_flag[0]) joint_m[0]->A = joint_m[0]->A*0.6f;
	if(flag->slip_flag[1]) joint_m[1]->A = joint_m[1]->A*0.6f;
	
	//卡腿降低轮毅输出
	if(flag->theta_flag[0]) joint_m[0]->A = joint_m[0]->A*0.6f;
	if(flag->theta_flag[1]) joint_m[1]->A = joint_m[1]->A*0.6f;
	
	//离地时关闭轮毅输出
	if(flag->off_flag)
	{
		joint_m[0]->A = 0;
		joint_m[1]->A = 0;
	}
	
	//磕上台阶后关闭轮毅输出
	if(flag->theta_flag[2]) 
	{
		joint_m[0]->A = 0;
		joint_m[1]->A = 0;
	}
}

/*******************************************************************************************************
向电机发送消息
********************************************************************************************************/
void Chassis_Can_Data_Send(Chassis_Motor_t *cm,
													 Controlled_State_t *cs,
													 Joint_Motor_Status_t *joint_m[2])
{                                             
	if(*cs == ERO) //停止所有电机
	{
		Disable_Motor_Mode(&hcan2,0x01); Disable_Motor_Mode(&hcan2,0x02);
		Disable_Motor_Mode(&hcan1,0x03); Disable_Motor_Mode(&hcan1,0x04);
		osDelay(1);
		Dji_Motor_Ctrl(&hcan2,0x200,0,0,0,0);
	}
	else
	{
		if(cm->DM_8009[0].state!=1 || 
			 cm->DM_8009[1].state!=1 ||
			 cm->DM_8009[2].state!=1 ||
			 cm->DM_8009[3].state!=1   ) //8009状态异常
		{
			if(cm->DM_8009[0].state==0 || 
				 cm->DM_8009[1].state==0 ||
			   cm->DM_8009[2].state==0 ||
			   cm->DM_8009[3].state==0   ) //8009失能后重新使能
			{
				Enable_Motor_Mode(&hcan2,0x01); Enable_Motor_Mode(&hcan2,0x02);
				Enable_Motor_Mode(&hcan1,0x03); Enable_Motor_Mode(&hcan1,0x04);
				osDelay(1);
				Dji_Motor_Ctrl(&hcan2,0x200,0,0,0,0);
			}
			else //清除8009异常
			{
				Clear_Err(&hcan2,0x01); Clear_Err(&hcan2,0x02);
				Clear_Err(&hcan1,0x03); Clear_Err(&hcan1,0x04);
				osDelay(1);
				Dji_Motor_Ctrl(&hcan2,0x200,0,0,0,0);
			}
		}
		else if(*cs == STOP)
		{
			Mit_Ctrl(&hcan2,0x01,0,0,0,0,0,DM8009); Mit_Ctrl(&hcan2,0x02,0,0,0,0,0,DM8009);
			Mit_Ctrl(&hcan1,0x03,0,0,0,0,0,DM8009); Mit_Ctrl(&hcan1,0x04,0,0,0,0,0,DM8009);
			osDelay(1);
			Dji_Motor_Ctrl(&hcan2,0x200,0,0,0,0);
		}
		else
		{
			Mit_Ctrl(&hcan2,0x01,0,0,0,0,Max_Output(joint_m[1]->T_A,40),DM8009);
			Mit_Ctrl(&hcan2,0x02,0,0,0,0,Max_Output(joint_m[1]->T_E,40),DM8009);
			Mit_Ctrl(&hcan1,0x03,0,0,0,0,Max_Output(joint_m[0]->T_E,40),DM8009);
			Mit_Ctrl(&hcan1,0x04,0,0,0,0,Max_Output(joint_m[0]->T_A,40),DM8009);
			osDelay(1);
			Dji_Motor_Ctrl(&hcan2,0x200,joint_m[1]->A,joint_m[0]->A,0,0);
		}
	}
}



