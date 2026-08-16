#include "Check_Task.h"

//INCLUDE部分
#include "Ref_Task.h"
#include "Parameter.h"
#include "Buzzer_Run.h"
#include "Chassis_Task.h"
#include "Board_Can_Task.h"
#include "Super_Cap_Task.h"
//EXTERN部分
extern void Error_Buzzer(Link_Sit_t *link);
//全局变量定义部分
Link_Sit_t Link_Sit;
Remote_Select_t Remote_Select;
off_the_ground_t off_the_ground;
Controlled_State_t Controlled_State;

bool all_ready_flag; //全部线程初始化完成标志  

//提前声明函数
void Ground_Clearance_Detection(INS_t *ins,Flag_Bit_t *flag,Bring_Legs_t *bring,off_the_ground_t *off,Leg_Current_Situation_t *leg[2],Joint_Motor_Status_t *joint_m[2],Vmc_Five_Link_Parameter_t *five_link[2]);
/*******************************************************************************************************
Check任务初始化
********************************************************************************************************/
void Check_Init(void)
{
	//Vofa初始化
	Vofa_Init();   
	
	//等待全部任务初始化完成 
	while(!chassis_ready_flag || !board_ready_flag || !super_ready_flag || !ref_ready_flag) {osDelay(1);}
	
	osDelay(2000);
	
	//提示初始化完成
	while(Ready_Buzzer_Flag != 1) {Ready_Buzzer();osDelay(1);}
	
	//所有线程初始化完成
	all_ready_flag = 1;
}

/*******************************************************************************************************
Check任务
********************************************************************************************************/
void Check_Task(void)
{
	Ground_Clearance_Detection(&INS,&Flag,&Bring_Legs,&off_the_ground,leg_ptr,joint_m_ptr,five_link_ptr);
	Check_Peripheral_Link(&Link_Sit,&Chassis_Motor);
	Check_Control(&Link_Sit,&Controlled_State);
	Error_Buzzer(&Link_Sit);
	Vofa_Send_Message();
}

/*******************************************************************************************************
离地检测
********************************************************************************************************/
void Ground_Clearance_Detection(INS_t *ins,
																Flag_Bit_t *flag,
																Bring_Legs_t *bring,
																off_the_ground_t *off,							
																Leg_Current_Situation_t *leg[2],
																Joint_Motor_Status_t *joint_m[2],
																Vmc_Five_Link_Parameter_t *five_link[2])
{
	off->Roll_Cut = Roll_Pid.Output*600.0f;
	
	off->dt_off = DWT_GetDeltaT(&off->dwt_d_off);
	
	leg[0]->dd_theta     = (leg[0]->d_theta - leg[0]->last_d_theta)/off->dt_off;
	leg[1]->dd_theta     = (leg[1]->d_theta - leg[1]->last_d_theta)/off->dt_off;
	leg[0]->last_d_theta = leg[0]->d_theta;    leg[1]->last_d_theta = leg[1]->d_theta;
	
	leg[0]->dd_L0        = (five_link[0]->d_L0 - leg[0]->last_d_L0)/off->dt_off;
	leg[1]->dd_L0        = (five_link[1]->d_L0 - leg[1]->last_d_L0)/off->dt_off;
	leg[0]->last_d_L0    = five_link[0]->d_L0; leg[1]->last_d_L0    = five_link[1]->d_L0;				 

	off->fn[0] = Mean_Filtering( ( (joint_m[0]->F0 - G_Comp[0].Output)*arm_cos_f32(leg[0]->theta) 
												     + joint_m[0]->Tp*arm_sin_f32(leg[0]->theta)/five_link[0]->L0
												     + 0.6f*(ins->MotionAccel_b[2]-leg[0]->dd_L0*arm_cos_f32(leg[0]->theta)
												     + 2.0f*five_link[0]->d_L0*leg[0]->d_theta*arm_sin_f32(leg[0]->theta)
												     + five_link[0]->L0*leg[0]->dd_theta*arm_sin_f32(leg[0]->theta)
												     + five_link[0]->L0*leg[0]->d_theta*leg[0]->d_theta*arm_cos_f32(leg[0]->theta))),leg[0]->aver_fn);				
				
	off->fn[1] = Mean_Filtering( ( (joint_m[1]->F0 - G_Comp[1].Output)*arm_cos_f32(leg[1]->theta) 
												     + joint_m[1]->Tp*arm_sin_f32(leg[1]->theta)/five_link[1]->L0
												     + 0.6f*(ins->MotionAccel_b[2]-leg[1]->dd_L0*arm_cos_f32(leg[1]->theta)
												     + 2.0f*five_link[1]->d_L0*leg[1]->d_theta*arm_sin_f32(leg[1]->theta)
												     + five_link[1]->L0*leg[1]->dd_theta*arm_sin_f32(leg[1]->theta)
												     + five_link[1]->L0*leg[1]->d_theta*leg[1]->d_theta*arm_cos_f32(leg[1]->theta))),leg[1]->aver_fn);					 
	
	//消去--ROLL补偿值;离地后向下的补偿力矩;卡腿后收腿补偿力
	if(off->Roll_Cut>=0)
	{
		leg[0]->fn = off->fn[0] - off->Roll_Cut + off->fn_comp - bring->Comp_Fn[0];
		leg[1]->fn = off->fn[1] - off->Roll_Cut + off->fn_comp - bring->Comp_Fn[1];
	}
	else
	{
		leg[0]->fn = off->fn[0] + off->Roll_Cut + off->fn_comp - bring->Comp_Fn[0];
		leg[1]->fn = off->fn[1] + off->Roll_Cut + off->fn_comp - bring->Comp_Fn[1];
	}
	
	//单腿离地判断
	if(leg[0]->fn >= OFF_GROUND_FN_MAX) off->off_flag[0]=1;
	else off->off_flag[0]=0;
	if(leg[1]->fn >= OFF_GROUND_FN_MAX) off->off_flag[1]=1;
	else off->off_flag[1]=0;
	
	//整车离地判断
	if(off->off_flag[0]==1 && 
		 off->off_flag[1]==1 &&
		!flag->fall_flag     &&	
		!flag->bump_flag       )
	{
		flag->off_flag = 1;
		off->fn_comp = OFF_GROUND_FN_COMP;
	}
	else
	{
		flag->off_flag = 0;
		off->fn_comp = 0;
	}
}

/*******************************************************************************************************
根据车体状况进行整车状态设置
********************************************************************************************************/
void Check_Control(Link_Sit_t *link,Controlled_State_t *cs)
{	
	if(((link->err_num&0x00FC)!=0)||((Up_Cboard_Info.up_err_num&0x0001)&&(Up_Cboard_Info.up_err_num&0x0002))) //车体异常 进入异常保护
	{
		*cs = ERO;
		Remote_Select = UNLINK;
	}
	else //无异常 由遥控器控制
	{
		if(!(Up_Cboard_Info.up_err_num&0x0001)) Remote_Select = DT7;
		else                                    Remote_Select = VT_03;
		
		if(Remote_Select == DT7) //DT7控制
		{
			if     (switch_is_down(Rc_Ctrl.rc.s[0])) {*cs = STOP ;}
			else if(switch_is_mid (Rc_Ctrl.rc.s[0])) {*cs = MOUSE;} 
			else if(switch_is_up  (Rc_Ctrl.rc.s[0])) {*cs = RC   ;}
		}
		else if(Remote_Select == VT_03) //VT_03控制
		{
			if     (VT03.mode_sw == 0) {*cs = STOP ;}
			else if(VT03.mode_sw == 1) {*cs = MOUSE;} 
			else if(VT03.mode_sw == 2) {*cs = MOUSE;}
		}
	}
}

/*******************************************************************************************************
检测各个外设的连接状态
********************************************************************************************************/

//RC,图传链路丢失         1位0001:RC丢失||1位0010:图传链路丢失
//
//轮毅,关节电机丢失       1位0100:左轮毅电机丢失||1位1000:右轮毅电机丢失
//                        2位0001:id为1的关节电机丢失||2位0010:id为2的关节电机丢失
//												2位0100:id为3的关节电机丢失||2位1000:id为4的关节电机丢失 
//                              
//超电,发射机构电机丢失   3位0001:左摩擦轮电机丢失||3位0010:右摩擦轮电机丢失
//                        3位0100:拨盘电机丢失
//                        3位1000:超电通讯丢失
//
//云台电机丢失            4位0001:Yaw电机丢失||4位0010:Pitch电机丢失

void Check_Peripheral_Link(Link_Sit_t *link,Chassis_Motor_t *cm)
{
	static int i;

	for(i=0;i<=3;i++)
	{
		link->joint[i] = Check_If_Unchange(cm->DM_8009[i].dm_link,dm_unlink_t);
	}
	for(i=0;i<=1;i++)
	{
		link->wheel[i] = Check_If_Unchange(cm->Dji_3508[i].dji_link,dji_unlink_t);
	}
	
	link->err_num = ( (link->wheel[0]<<2) + (link->wheel[1]<<3)
	                + (link->joint[0]<<4) + (link->joint[1]<<5) 
	                + (link->joint[2]<<6) + (link->joint[3]<<7) ) | Up_Cboard_Info.up_err_num;						
}




