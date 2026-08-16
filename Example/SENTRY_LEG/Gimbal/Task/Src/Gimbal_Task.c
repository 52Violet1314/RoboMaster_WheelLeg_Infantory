#include "Gimbal_Task.h"

//全局变量定义部分
Self_Rescue_t Self_Rescue; //自救
Shoot_Status_t Shoot_Status;
Gimbal_Status_t Gimbal_Status; 
Shoot_Condition_t Shoot_Condition; 
Compensation_Amount_t Compensation_Amount;

//建立控制器结构体
PID_t Abs_Big_Yaw_P_Pid; //云台
PID_t Abs_Small_Yaw_P_Pid;

PID_t Pitch_P_Pid; 
PID_t Pitch_S_Pid; 
PID_t Big_Yaw_P_Pid; 
PID_t Small_Yaw_P_Pid; 
PID_t Small_Yaw_S_Pid; 

PID_t L_Rpm_Pid; //发射机构
PID_t R_Rpm_Pid;

PID_t D_Pos_Pid; //拨盘
PID_t D_Rpm_Pid; 

//建立前馈控制结构体
Feedforward_t yaw_FD;   	 //YAW前馈
Feedforward_t pitch_FD;    //PITCH前馈
Feedforward_t dyaw_FD[3];  //YAW速度前馈[0为大YAW 1为小YAW 2为大YAW]
Feedforward_t Shoot_FD[2]; //摩擦轮加速度前馈[0为左 1为右] 

bool Aim_Permission = 0;  //自瞄许可
bool Fire_Permission = 0; //开火许可

bool Jammed_flag = 0; //卡弹标志位
bool Aim_Converge_Flag = 0;  //自瞄收敛标志位

float Friction_Speed_Comp = 0; //摩擦轮速度补偿量

bool gimbal_ready_flag; //当前线程初始化完成标志 

/*******************************************************************************************************
Gimbal任务初始化
********************************************************************************************************/
void Gimbal_Init(void)
{
	//初始化PID
	PID_Init(&Abs_Big_Yaw_P_Pid    , 12000, 8000, 0, 14000,     0,   740,  0,0,0,0, 2,RADIAN,NONE); //云台
	PID_Init(&Abs_Small_Yaw_P_Pid  , 12000, 8000, 0, 14000,     0,   740,  0,0,0,0, 2,RADIAN,NONE);
	
	PID_Init(&Pitch_P_Pid          ,    80,   10, 0,    72,     0,     4,  0,0,0,0, 4,RADIAN   ,NONE);
	PID_Init(&Pitch_S_Pid          , 16384, 8000, 0,  3000,     0,     0,  0,0,0,0, 0,NO_CIRCLE,NONE);
	PID_Init(&Big_Yaw_P_Pid        , 16384, 8000, 0,  1800,     0,   140,  0,0,0,0, 8,RADIAN   ,NONE);
	PID_Init(&Small_Yaw_P_Pid      ,    60,   10, 0,    30,     0,     2,  0,0,0,0, 2,RADIAN   ,NONE);
	PID_Init(&Small_Yaw_S_Pid      , 16384, 8000, 0,  2400,     0,     0,  0,0,0,0, 0,NO_CIRCLE,NONE);

	PID_Init(&L_Rpm_Pid            , 16384, 1000, 0,  16.5,     0,     0,  0,0,0,0, 8,NO_CIRCLE,NONE); //发射机构
	PID_Init(&R_Rpm_Pid            , 16384, 1000, 0,  16.5,     0,     0,  0,0,0,0, 8,NO_CIRCLE,NONE);
	
	PID_Init(&D_Pos_Pid            , 10000, 1000,  0,  2400,     0,   800,  0,0,0,0, 0,RADIAN   ,NONE); //拨盘
	PID_Init(&D_Rpm_Pid            , 16384, 1000,  0,    12,     0, 0.001,  0,0,0,0, 0,NO_CIRCLE,NONE); 
	
	//初始化前馈函数(c0=静态增益 c1=速度补偿 c2=加速度补偿)
	static float ffc_c_pi[3] = {   1300,        0.00001,     0.00000001}; //PITCH轴
	  
	static float ffc_c_by[3] = {    800,         0.00001,          0.00000001}; //YAW轴
	static float ffc_c_sy[3] = {    300,         0.00001,          0.00000001};  
	static float ffc_c_bd[3] = {    540,         0.00001,          0.00000001}; 
	static float ffc_c_yi[3] = {   2000,         0.00001,          0.00000001}; 
	
	static float ffc_c_L[3]  = {0.00010, 0.00000000006, 0.000000000001}; //摩擦轮  
	static float ffc_c_R[3]  = {0.00010, 0.00000000006, 0.000000000001}; 
	
	Feedforward_Init(&pitch_FD  ,16384,ffc_c_pi,0.05,1,1); //PITCH轴
	
	Feedforward_Init(&dyaw_FD[0],16384,ffc_c_by,0.05,1,1); //YAW轴
	Feedforward_Init(&dyaw_FD[1],16384,ffc_c_sy,0.05,1,1);
	Feedforward_Init(&dyaw_FD[2],16384,ffc_c_bd,0.05,1,1);
	Feedforward_Init(&yaw_FD    ,16384,ffc_c_yi,0.05,1,1);
	
	Feedforward_Init(&Shoot_FD[0],0.30,ffc_c_L ,0.05,1,1); //摩擦轮
	Feedforward_Init(&Shoot_FD[1],0.30,ffc_c_R ,0.05,1,1); 
	
	//拟合系数初始化
	Compensation_Amount.p_pitch[0] =  -683.6535f;
	Compensation_Amount.p_pitch[1] = -115.2624f;
	Compensation_Amount.p_pitch[2] = -1.7064f;
	Compensation_Amount.p_pitch[3] =  0.1251f;
	Compensation_Amount.p_pitch[4] =  2.8614e-04;
	
	//发射机构状态关闭
	Shoot_Condition = Close;
	
	//初始化完成
	gimbal_ready_flag  = 1;
	
	//等待
	while(all_ready_flag!=1)
	{
		Gimbal_Status.pitch_ref  = INS.Roll; //设定云台初始目标值
		Gimbal_Status.yaw_ref[1] = INS.Yaw;
		osDelay(1);
	}
}

/*******************************************************************************************************
Gimbal任务
********************************************************************************************************/
void Gimbal_Task(void)
{
	Variable_Information_Acquisition(&INS,&Gimbal_Motor,&Shoot_Status,&Gimbal_Status);
	Gimbal_Control(&Rc_Ctrl,&Pc_Ctrl,&Gimbal_Status,&Shoot_Condition,&Self_Rescue,&Controlled_State);
	Auto_Aim(&aim_rx,&Gimbal_Status);
	Gimbal_Target_Limit(&Gimbal_Status);
	Shoot_Control(&Shoot_Status,&Heat_Control,&Shoot_Condition);
	Gimbal_Controllor(&Shoot_Status,&Gimbal_Status,&Self_Rescue,&Controlled_State,&Compensation_Amount);
	Gimbal_Can_Data_Send(&Shoot_Status,&Gimbal_Status);
}

/*******************************************************************************************************
云台关键数据获取
********************************************************************************************************/
void Variable_Information_Acquisition(INS_t *ins,
																	    Gimbal_Motor_t *gm,
																			Shoot_Status_t *ss,
																			Gimbal_Status_t *gs)
{
	static uint32_t dwt_dpos;
	
	//获取摩擦轮RPM
	ss->L_Rpm = gm->Dji_3508[0].speed_rpm; 
	ss->R_Rpm = gm->Dji_3508[1].speed_rpm;
	
	//计算摩擦轮的转动加速度
	ss->acc.L_A_Rpm = ((ss->L_Rpm - ss->acc.L_Last_Rpm)*((2.0f*PI)/60.0f))/DWT_GetDeltaT(&ss->acc.dwt_l);
	ss->acc.R_A_Rpm = ((ss->R_Rpm - ss->acc.R_Last_Rpm)*((2.0f*PI)/60.0f))/DWT_GetDeltaT(&ss->acc.dwt_r);
	ss->acc.L_Last_Rpm = ss->L_Rpm;
	ss->acc.R_Last_Rpm = ss->R_Rpm;
	
	//获取拨盘转速以及拨盘电流
	ss->D_Rpm = gm->Dji_3508[2].speed_rpm; //拨盘
	ss->D_Cur = gm->Dji_3508[2].given_current;
	
	//估算出来的拨盘位置
	ss->D_Pos = Half_Circle_RADIAN(ss->D_Pos + ((Limit_Min(ss->D_Rpm,10)/36.0f)/60.0f)*2.0f*PI*DWT_GetDeltaT(&dwt_dpos));
	
	//计算获取云台电机绝对位置
	gs->abs_yaw[0] = Half_Circle_RADIAN((gm->Dji_6020[0].ecd/8191.0f)*2*PI);
	gs->abs_yaw[1] = Half_Circle_RADIAN((gm->Dji_6020[1].ecd/8191.0f)*2*PI);
	gs->abs_pitch  = Half_Circle_RADIAN((gm->Dji_6020[2].ecd/8191.0f)*2*PI);
	
	//计算获取云台陀螺仪数据
	gs->yaw     = ins->Yaw;
	gs->d_yaw   = ins->Gyro[2];
	gs->pitch   = ins->Roll;
	gs->d_pitch = ins->Gyro[1];
	
	//计算获取大YAW head与小YAW head的差角度
	gs->follow_theta  = Find_Min_RADIAN(gs->abs_yaw[1],ZERO_HEAD_SMY);
	gs->follow_dtheta = (gm->Dji_6020[1].speed_rpm/60.0f)*2.0f*PI;
}

/*******************************************************************************************************
云台控制各项参数初始化
********************************************************************************************************/
void Gimbal_Control_Init(Gimbal_Status_t *gs,
												 Shoot_Condition_t *sc)
{
	*sc = Close; //发射机构状态重置
	
	gs->yaw_ref[0] =       0; //云台目标状态重置
	gs->yaw_ref[1] = gs->yaw;
	gs->pitch_ref  = gs->pitch;
	
	Aim_Permission = 0; //许可重置
	Fire_Permission = 0; 
}

/*******************************************************************************************************
遥控器模式
********************************************************************************************************/
void Rc_Mode(RC_Ctrl_t *rc_ctrl,
						 Shoot_Condition_t *sc)
{	
	if     (switch_is_down(rc_ctrl->rc.s[1])){*sc = Close;} //关闭发射机构
	else if(switch_is_mid (rc_ctrl->rc.s[1])){*sc = Open ;} //开启摩擦轮
	
	if(switch_is_up(rc_ctrl->rc.s[1])) Fire_Permission = 1; //允许开火
	else Fire_Permission = 0;
	
//	*sc = Close;
//	Fire_Permission = 0;
	
	if(rc_ctrl->rc.ch[0] == 660) Aim_Permission = 1; //开启自瞄
	else Aim_Permission = 0;
}

/*******************************************************************************************************
键鼠模式初始化
********************************************************************************************************/
void Pc_Init(PC_Ctrl_t *pc_ctrl)
{
	pc_ctrl->Q = 0;
	pc_ctrl->q_t = 0;
}

/*******************************************************************************************************
键鼠模式
********************************************************************************************************/
void Pc_Mode(RC_Ctrl_t *rc_ctrl,
						 PC_Ctrl_t *pc_ctrl)
{
	/******左键检测,允许开火******/
	if(rc_ctrl->mouse.press_l) Fire_Permission = 1;
	else Fire_Permission = 0;
	
	/******右键检测,开启自瞄******/
	if(rc_ctrl->mouse.press_r) Aim_Permission = 1;
	else Aim_Permission = 0;
	
	/******Q键检测,摩擦轮开启******/
	if((rc_ctrl->key.v&KEY_PRESSED_OFFSET_Q) && !pc_ctrl->Q)
	{
		pc_ctrl->Q = 1;
		pc_ctrl->q_t = HAL_GetTick();
	}
	else if(!(rc_ctrl->key.v&KEY_PRESSED_OFFSET_Q) && pc_ctrl->Q)
	{
		pc_ctrl->Q = 0;
		if(HAL_GetTick()-pc_ctrl->q_t < 500)
		{
			if(Shoot_Condition != Close) Shoot_Condition = Close;
			else Shoot_Condition = Open;
		}
	}
	
	/******S1检测,转速加减******/
	static bool speed_flag = 0;
	
	if(switch_is_mid(rc_ctrl->rc.s[1]) && !speed_flag)
	{
		speed_flag = 1;
	}
	else if(switch_is_down(rc_ctrl->rc.s[1]) && speed_flag)
	{
		speed_flag = 0;
		Friction_Speed_Comp -= 20;
	}
	else if(switch_is_up(rc_ctrl->rc.s[1]) && speed_flag)
	{
		speed_flag = 0;
		Friction_Speed_Comp += 20;
	}
}

/*******************************************************************************************************
VT03的键鼠模式
********************************************************************************************************/
void Vt03_Pc_Mode(PC_Ctrl_t *pc_ctrl)
{
	/******左键检测,允许开火******/
	if(VT03.mouse_left) Fire_Permission = 1;
	else Fire_Permission = 0;
	
	/******右键检测,开启自瞄******/
	if(VT03.mouse_right) Aim_Permission = 1;
	else Aim_Permission = 0;
	
	/******Q键检测,摩擦轮开启******/
	if((VT03.key&KEY_PRESSED_OFFSET_Q) && !pc_ctrl->Q)
	{
		pc_ctrl->Q = 1;
		pc_ctrl->q_t = HAL_GetTick();
	}
	else if(!(VT03.key&KEY_PRESSED_OFFSET_Q) && pc_ctrl->Q)
	{
		pc_ctrl->Q = 0;
		if(HAL_GetTick()-pc_ctrl->q_t < 500)
		{
			if(Shoot_Condition != Close) Shoot_Condition = Close;
			else Shoot_Condition = Open;
		}
	}
	
	/******FN检测,转速加减******/
	static bool speed_flag = 0;
	
	if(VT03.fn_1 && !speed_flag)
	{
		speed_flag = 1;
		Friction_Speed_Comp -= 20;
	}
	else if(VT03.fn_2 && !speed_flag)
	{
		speed_flag = 1;
		Friction_Speed_Comp += 20;
	}
	else if(!VT03.fn_1 && !VT03.fn_2)
	{
		speed_flag = 0;
	}
}

/*******************************************************************************************************
寻找合适的YAW轴复位零点
********************************************************************************************************/
void Chose_Zero_Target(Gimbal_Status_t *gs,
											 Self_Rescue_t *self_re)
{
	static float dis_to_head = 0; //到正方向零点位置最小角度值
	static float dis_to_back = 0; //到负方向零点位置最小角度值
	
	dis_to_head = fabs(Find_Min_RADIAN(gs->abs_yaw[0],ZERO_HEAD_YAW));
	dis_to_back = fabs(Find_Min_RADIAN(gs->abs_yaw[0],ZERO_BACK_YAW));
	
	if(dis_to_head <= dis_to_back) self_re->Yaw_Zero_Target[0] = ZERO_HEAD_YAW;
	else                           self_re->Yaw_Zero_Target[0] = ZERO_BACK_YAW;
	
	self_re->Yaw_Zero_Target[1] = ZERO_HEAD_SMY;

	gs->abs_yaw_ref[0] = self_re->Yaw_Zero_Target[0]; //对绝对位置目标进行赋值
	gs->abs_yaw_ref[1] = self_re->Yaw_Zero_Target[1];
}

/*******************************************************************************************************
云台控制逻辑
********************************************************************************************************/
void Gimbal_Control(RC_Ctrl_t *rc_ctrl,
										PC_Ctrl_t *pc_ctrl,
										Gimbal_Status_t *gs,
										Shoot_Condition_t *sc,
										Self_Rescue_t *self_re,		
										Controlled_State_t *cs)
{
	if(Remote_Select == UNLINK) //未连接遥控器
	{
		gs->Pc_Pitch = 0;
		gs->Pc_Yaw   = 0;
	}
	else if(Remote_Select == DT7) //连接DT7遥控器
	{
		gs->Pc_Pitch = Limit_Min(rc_ctrl->mouse.y,PC_DEADBAND);
		gs->Pc_Yaw   = Limit_Min(rc_ctrl->mouse.x,PC_DEADBAND);
	}
	else if(Remote_Select == VT_03) //连接VT03遥控器
	{
		gs->Pc_Pitch = Limit_Min(VT03.mouse_y,PC_DEADBAND);
		gs->Pc_Yaw   = Limit_Min(VT03.mouse_x,PC_DEADBAND);
	}
	
	gs->Rc_Pitch = Limit_Min(rc_ctrl->rc.ch[3],RC_DEADBAND);
	gs->Rc_Yaw   = Limit_Min(rc_ctrl->rc.ch[2],RC_DEADBAND);
	
	if(*cs!=ERO && *cs!=STOP)
	{
		if(Remote_Select == UNLINK) //未连接遥控器
		{
			self_re->run_flag = 0; //重启复位
			
			gs->abs_yaw_ref[0] = gs->abs_yaw[0]; //绝对位置目标值重置
			gs->abs_yaw_ref[1] = gs->abs_yaw[1];
			
			Gimbal_Control_Init(gs,sc);
		}
		else if(Remote_Select == DT7) //连接DT7遥控器
		{
			if(Down_Cboard_Info.fall_flag) //倒地
			{
				if(((rc_ctrl->key.v&KEY_PRESSED_OFFSET_R) || rc_ctrl->rc.ch[1]==-660) && !self_re->run_flag)//触发自救
				{
					Chose_Zero_Target(gs,self_re);
					self_re->run_flag = 1;
				}
				
				Pc_Init(pc_ctrl); //初始化
				Gimbal_Control_Init(gs,sc);
			}
			else //正常
			{
				self_re->run_flag = 0; //重启复位
				
				if(*cs == RC) //遥控器模式
				{
					gs->yaw_ref[1] = Half_Circle_ANGLE(gs->yaw_ref[1] - RC_YAW_SENSITIVITY*gs->Rc_Yaw); //云台
					gs->pitch_ref += RC_PITCH_SENSITIVITY * gs->Rc_Pitch; 
					
					Pc_Init(pc_ctrl);
					Rc_Mode(rc_ctrl,sc);
				}
				else if(*cs == MOUSE) //键鼠模式
				{
					gs->yaw_ref[1] =  Half_Circle_ANGLE(gs->yaw_ref[1] - PC_YAW_SENSITIVITY*gs->Pc_Yaw); //云台
					gs->pitch_ref +=  PC_PITCH_SENSITIVITY * gs->Pc_Pitch; 
					
					Pc_Mode(rc_ctrl,pc_ctrl);
				}
			}
		}
		else if(Remote_Select == VT_03) //连接VT03遥控器
		{
			if(Down_Cboard_Info.fall_flag) //倒地
			{
				if((VT03.key&KEY_PRESSED_OFFSET_R) && !self_re->run_flag)//触发自救
				{
					Chose_Zero_Target(gs,self_re);
					self_re->run_flag = 1;
				}
				
				Pc_Init(pc_ctrl); //初始化
				Gimbal_Control_Init(gs,sc);
			}
			else //正常
			{
				self_re->run_flag = 0; //重启复位
				
				if(*cs == MOUSE) //键鼠模式
				{
					gs->yaw_ref[1] =  Half_Circle_ANGLE(gs->yaw_ref[1] - PC_YAW_SENSITIVITY*gs->Pc_Yaw); //云台
					gs->pitch_ref +=  PC_PITCH_SENSITIVITY * gs->Pc_Pitch; 
					
					Vt03_Pc_Mode(pc_ctrl);
				}
			}
		}
	}
	else
	{
		self_re->run_flag = 0; //重启复位
		
		gs->abs_yaw_ref[0] = gs->abs_yaw[0]; //绝对位置目标值重置
		gs->abs_yaw_ref[1] = gs->abs_yaw[1];
		
		Gimbal_Control_Init(gs,sc);
	}
}

/*******************************************************************************************************
自瞄控制逻辑
********************************************************************************************************/
void Auto_Aim(Aim_Rx *aim,
							Gimbal_Status_t *gs)
{
	if(Aim_Permission && aim->mode != 0)
	{
		gs->pitch_ref  = -aim->pitch*PI_Ang;
		gs->yaw_ref[1] =  aim->yaw*PI_Ang;
	}
}

/*******************************************************************************************************
对目标值进行限制
********************************************************************************************************/
void Gimbal_Target_Limit(Gimbal_Status_t *gs)
{
	if     (gs->pitch_ref >= PITCH_UP_LIMIT_POSITION  ) gs->pitch_ref = PITCH_UP_LIMIT_POSITION;
	else if(gs->pitch_ref <= PITCH_DOWN_LIMIT_POSITION) gs->pitch_ref = PITCH_DOWN_LIMIT_POSITION;
}

/*******************************************************************************************************
发射机构控制逻辑
********************************************************************************************************/
void Shoot_Control(Shoot_Status_t *ss,
									 Heat_Control_t *hc,
									 Shoot_Condition_t *sc)
{
	static int back_time = 0; //反拨时间
	static int redu_bullet_num = 1; //热量冗余弹丸设定
	
	//关闭
	if(*sc == Close)
	{
		ss->Target_Rpm[0] = 0;
		ss->Target_Rpm[1] = 0;
		
		Jammed_flag = 0;
	}
	//单开摩擦轮
	else if(*sc == Open)
	{
		ss->Target_Rpm[0] = Friction_Speed + Friction_Speed_Comp;
		
		if(Fire_Permission) //允许发射弹丸
		{
			if(abs(ss->D_Cur)>=Jammed_Cur && abs(ss->D_Rpm)<=Jammed_Rpm) Jammed_flag = 1; //判断是否卡弹
			
			if(Jammed_flag) //卡弹
			{
				back_time++;
				ss->Target_Rpm[1] = -300; 
				
				if(back_time>=160)
				{
					back_time = 0;
					Jammed_flag = 0;
				}
			}
			else //正常
			{
				if(Aim_Permission) //自瞄情况下判断开火
				{
					if(aim_rx.mode==2) //允许开火
					{
						if(hc->bullet_num>=10)
						{
							ss->Target_Rpm[1] = Dial_Speed;
						}
						else if(hc->bullet_num<10)
						{
							ss->Target_Rpm[1] = Dial_Speed*((float)(hc->bullet_num - redu_bullet_num)/9.0f);
							
							if(ss->Target_Rpm[1]<=0) ss->Target_Rpm[1] = 0;
						}
						else ss->Target_Rpm[1] = 0;
					}
					else ss->Target_Rpm[1] = 0;
				}
				else //手动控制下判断开火
				{
					if(hc->bullet_num>=10)
					{
						ss->Target_Rpm[1] = Dial_Speed;
					}
					else if(hc->bullet_num<10)
					{
						ss->Target_Rpm[1] = Dial_Speed*((float)(hc->bullet_num - redu_bullet_num)/9.0f);
						
						if(ss->Target_Rpm[1]<=0) ss->Target_Rpm[1] = 0;
					}
					else ss->Target_Rpm[1] = 0;
				}
			}
		}
		else ss->Target_Rpm[1] = 0;
	}
}

/*******************************************************************************************************
云台控制器
********************************************************************************************************/
void Gimbal_Controllor(Shoot_Status_t *ss,
											 Gimbal_Status_t *gs,
											 Self_Rescue_t *self_re,
											 Controlled_State_t *cs,
											 Compensation_Amount_t *ca)
{
	//发射机构摩擦轮前馈计算
	Feedforward_Calculate(&Shoot_FD[0],ss->acc.L_A_Rpm);
	Feedforward_Calculate(&Shoot_FD[1],ss->acc.R_A_Rpm);
	//YAW前馈计算
	Feedforward_Calculate(&dyaw_FD[0],gs->d_yaw);
	Feedforward_Calculate(&dyaw_FD[1],-(Down_Cboard_Info.down_dyaw + (Gimbal_Motor.Dji_6020[0].speed_rpm/60.0f)*2.0f*PI) - gs->d_yaw);
	Feedforward_Calculate(&dyaw_FD[2],Down_Cboard_Info.down_dyaw + (Down_Cboard_Info.down_dyaw + (Gimbal_Motor.Dji_6020[0].speed_rpm/60.0f)*2.0f*PI));
	Feedforward_Calculate(&yaw_FD    ,Small_Yaw_P_Pid.Output);
	//PITCH前馈计算
	Feedforward_Calculate(&pitch_FD,Pitch_P_Pid.Output);
	//PITCH拟合补偿计算
	ca->Gravity_Comp_PITCH = ca->p_pitch[4]*gs->pitch*gs->pitch*gs->pitch*gs->pitch
												 + ca->p_pitch[3]*gs->pitch*gs->pitch*gs->pitch
												 + ca->p_pitch[2]*gs->pitch*gs->pitch											
												 + ca->p_pitch[1]*gs->pitch
												 + ca->p_pitch[0];
	
	if(*cs!=ERO && *cs!=STOP)
	{
		if(Down_Cboard_Info.fall_flag) //倒地
		{
			gs->Pitch_Motor_Out  = 0; //云台
			gs->Yaw_Motor_Out[0] = 0;
			gs->Yaw_Motor_Out[1] = 0;
			
			ss->L_Motor_Out = 0; //发射机构
			ss->R_Motor_Out = 0;
			
			ss->D_Motor_Out = 0; //拨盘
			
			if(self_re->run_flag == 1) //启动云台复位
			{
				gs->Pitch_Motor_Out  = 6000.0f;
				gs->Yaw_Motor_Out[1] = PID_Calculate(&Abs_Small_Yaw_P_Pid,gs->abs_yaw[1],gs->abs_yaw_ref[1]);
				gs->Yaw_Motor_Out[0] = PID_Calculate(&Abs_Big_Yaw_P_Pid  ,gs->abs_yaw[0],gs->abs_yaw_ref[0]) + gs->Yaw_Motor_Out[1];
			}
		}
		else //正常
		{
			//云台
			PID_Calculate(&Pitch_P_Pid,gs->pitch*Ang_PI,gs->pitch_ref*Ang_PI);
			gs->Pitch_Motor_Out = PID_Calculate(&Pitch_S_Pid,gs->d_pitch,Pitch_P_Pid.Output) + ca->Gravity_Comp_PITCH + pitch_FD.Output;
		
			PID_Calculate(&Small_Yaw_P_Pid,gs->yaw*Ang_PI,gs->yaw_ref[1]*Ang_PI);
			gs->Yaw_Motor_Out[1] = PID_Calculate(&Small_Yaw_S_Pid,gs->d_yaw,Small_Yaw_P_Pid.Output) - dyaw_FD[1].Output + yaw_FD.Output;

			gs->Yaw_Motor_Out[0] = -PID_Calculate(&Big_Yaw_P_Pid,gs->abs_yaw[1]*PI_Ang,ZERO_HEAD_SMY*PI_Ang) - dyaw_FD[2].Output;

			//发射机构
			ss->L_Motor_Out = PID_Calculate(&L_Rpm_Pid,ss->L_Rpm, ss->Target_Rpm[0]) 
											- (10.0f/3.0f)*(3591.0f/187.0f)*(16384.0f/20.0f)*Shoot_FD[0].Output; 
			ss->R_Motor_Out = PID_Calculate(&R_Rpm_Pid,ss->R_Rpm,-ss->Target_Rpm[0]) 
											- (10.0f/3.0f)*(3591.0f/187.0f)*(16384.0f/20.0f)*Shoot_FD[1].Output;
			
			//拨盘
			ss->D_Motor_Out = PID_Calculate(&D_Rpm_Pid,ss->D_Rpm,ss->Target_Rpm[1]); 
		}
	}
	else
	{
		gs->Pitch_Motor_Out  = 0; //云台
		gs->Yaw_Motor_Out[0] = 0;
		gs->Yaw_Motor_Out[1] = 0;
		
		ss->L_Motor_Out = 0; //发射机构
		ss->R_Motor_Out = 0;
		
		ss->D_Motor_Out = 0; //拨盘
	}
}

/*******************************************************************************************************
向电机发送消息
********************************************************************************************************/
void Gimbal_Can_Data_Send(Shoot_Status_t *ss,
													Gimbal_Status_t *gs)
{
	Dji_Motor_Ctrl(&hcan2,0x1FE,Max_Output(gs->Yaw_Motor_Out[0],16384.0f),Max_Output(gs->Yaw_Motor_Out[1],16384.0f),Max_Output(gs->Pitch_Motor_Out,16384.0f),0);
	Dji_Motor_Ctrl(&hcan2,0x200,Max_Output(     ss->D_Motor_Out,16384.0f),Max_Output(     ss->R_Motor_Out,16384.0f),Max_Output(    ss->L_Motor_Out,16384.0f),0);
}

