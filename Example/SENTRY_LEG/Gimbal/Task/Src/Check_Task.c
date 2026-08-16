#include "Check_Task.h"

//INCLUDE部分
#include "VT03.h"
#include "Nuc_Task.h"
#include "Buzzer_Run.h"
#include "Gimbal_Task.h"
//EXTERN部分
extern void Error_Buzzer(Link_Sit_t *link);
//全局变量定义部分
Link_Sit_t Link_Sit;
Remote_Select_t Remote_Select;
Controlled_State_t Controlled_State;

bool all_ready_flag; //全部线程初始化完成标志 

/*******************************************************************************************************
Check任务初始化
********************************************************************************************************/
void Check_Init(void)
{
	//Vofa初始化
	Vofa_Init();
	
	//等待其他任务初始化
	while(!nuc_ready_flag || !limit_ready_flag || !gimbal_ready_flag || !board_ready_flag){osDelay(1);}
	
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
	Check_Peripheral_Link(&Link_Sit,&Gimbal_Motor);
	Check_Control(&Link_Sit,&Controlled_State);
	Error_Buzzer(&Link_Sit);
	Vofa_Send_Message();
}

/*******************************************************************************************************
根据车体状况进行整车状态设置
********************************************************************************************************/
void Check_Control(Link_Sit_t *link,Controlled_State_t *cs)
{	
	if(link->rc && link->tv) //车体异常 进入异常保护
	{
		*cs = ERO;
		Remote_Select = UNLINK;
	}
	else //无异常
	{
		if(!link->rc) Remote_Select = DT7;
		else          Remote_Select = VT_03;
		
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
//超电控制板丢失					3位0001:超电通讯丢失
//
//发射机构电机丢失        3位0010:左摩擦轮电机丢失||3位0100:右摩擦轮电机丢失
//                        3位1000:拨盘电机丢失
//                        
//云台电机丢失            4位0001:Big_Yaw电机丢失||4位0010:Small_Yaw电机丢失||4位0100:Pitch电机丢失

void Check_Peripheral_Link(Link_Sit_t *link,Gimbal_Motor_t *cm)
{
	link->rc = Check_If_Unchange(Rc_Ctrl.rc_link,rc_unlink_t);
	link->tv = Check_If_Unchange(        vt_link,vt_unlink_t);
	
	link->shoot[0] = Check_If_Unchange(cm->Dji_3508[0].dji_link,dji_unlink_t);
	link->shoot[1] = Check_If_Unchange(cm->Dji_3508[1].dji_link,dji_unlink_t);
	link->shoot[2] = Check_If_Unchange(cm->Dji_3508[2].dji_link,dm_unlink_t);
	
	link->gimbal[0] = Check_If_Unchange(cm->Dji_6020[0].dji_link,dji_unlink_t);
	link->gimbal[1] = Check_If_Unchange(cm->Dji_6020[1].dji_link,dji_unlink_t);
	link->gimbal[2] = Check_If_Unchange(cm->Dji_6020[2].dji_link,dji_unlink_t);
	
	link->err_num = (link->rc<<0) + (link->tv<<1)
								+ (0<<2) + (0<<3)
								+ (0<<4) + (0<<5) + (0<<6) + (0<<7)
								+ (0<<8)
								+ (link->shoot[0]<<9) + (link->shoot[1]<<10) + (link->shoot[2]<<11)
	              + (link->gimbal[0]<<11) + (link->gimbal[1]<<12) + (link->gimbal[2]<<13);							
}



