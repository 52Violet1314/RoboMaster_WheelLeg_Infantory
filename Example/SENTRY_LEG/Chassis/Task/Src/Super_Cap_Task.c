#include "Super_Cap_Task.h"

//全局变量定义部分
PID_t Cap_Buffer_Pid; //超电缓存功率PID控制器

bool super_ready_flag; //当前线程初始化完成标志 

/*******************************************************************************************************
Super_Cap任务初始化
********************************************************************************************************/
void Super_Cap_Init(void)
{
	//初始化PID
	PID_Init(&Cap_Buffer_Pid, 120, 10, 0,  4,  0,  0,  0,0,0,0, 0,NO_CIRCLE,NONE); //超电缓存功率控制PID
	
	super_ready_flag = 1;
	
	//等待
	while(all_ready_flag!=1){osDelay(1);}
}

/*******************************************************************************************************
Super_Cap任务
********************************************************************************************************/
void Super_Cap_Task(void)
{
	Super_Cap_Control(&Controlled_State);
}

/*******************************************************************************************************
超电控制
********************************************************************************************************/
//bz:改变超电的最大电压	Pm_Voltage_Set(&hcan2,2050,0x01)
void Super_Cap_Control(Controlled_State_t *cs)
{
	//超电缓存能量控制
	if(*cs!=ERO && *cs!=STOP) PID_Calculate(&Cap_Buffer_Pid,Power_Heat_Data.buffer_energy,20);
	else Cap_Buffer_Pid.Output = 0;
	Input_Power = Max_Output(Robot_Status.chassis_power_limit - Cap_Buffer_Pid.Output,120);
	
	Pm_Power_Set(&hcan2,Input_Power*100,0x00); //设置最大充电功率
	Can_Power_Read(&hcan2);
}

