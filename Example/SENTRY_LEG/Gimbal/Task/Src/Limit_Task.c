#include "Limit_Task.h"

//INCLUDE部分
#include "bsp_dwt.h"
#include "Check_Task.h"
#include "Can_Feedback.h"
//全局变量定义部分
Heat_Control_t Heat_Control;

bool limit_ready_flag; //当前线程初始化完成标志 

/*******************************************************************************************************
LIMIT任务初始化
********************************************************************************************************/
void Limit_Init(void)
{
	limit_ready_flag = 1;
	
	//设置最大延迟dt
	Heat_Control.pin_dt = 0.5f;
	
	//等待
	while(!all_ready_flag) {osDelay(1);}
}

/*******************************************************************************************************
LIMIT任务
********************************************************************************************************/
void Limit_Task(void)
{
	Heat_Limit_Control(&Heat_Control);
}

/*******************************************************************************************************
热量控制
********************************************************************************************************/
void Heat_Limit_Control(Heat_Control_t *hc)
{
	//获取机载端数据
	hc->Air_Q0        = Down_Cboard_Info.heat_limit;
	hc->Air_Q1        = Down_Cboard_Info.barrel_heat;
	hc->cooling_value = Down_Cboard_Info.cooling_value;
	
	//进行延迟补偿
	if(hc->Air_Q1 != hc->Last_Air_Q1)
	{
		hc->pin_dt = DWT_GetDeltaT(&hc->dwt_pin);
		
		if(hc->pin_dt>=0.5f) hc->pin_dt = 0.5f; //限制最大延迟
		
		//本地热量值计算(补偿通讯延迟)
		hc->Local_Q1 = hc->Air_Q1 + hc->pin_dt*hc->cooling_value;
	}
	
	if(hc->Air_Q1 == 0) hc->Local_Q1 = 0;
	
	hc->Last_Air_Q1 = hc->Air_Q1;
	
	//计算剩余可发弹丸数量
	if(Down_Cboard_Info.heat_limit<=0) hc->bullet_num = 10;
	else hc->bullet_num = (int)(((float)(hc->Air_Q0) - (float)(hc->Local_Q1))/HEAT_17MM);
}









