#include "Board_Can_Task.h"

//INCLUDE部分
#include "Ref_Task.h"
//全局变量定义部分
bool board_ready_flag; //当前线程初始化完成标志 

/*******************************************************************************************************
Board_Can任务初始化
********************************************************************************************************/
void Board_Can_Init(void)
{
	board_ready_flag = 1;
	
	//等待
	while(all_ready_flag!=1){osDelay(1);}
}

/*******************************************************************************************************
Board_Can任务
********************************************************************************************************/
void Board_Can_Task(void)
{
	Send_Message();
}

/*******************************************************************************************************
向上板发送消息
********************************************************************************************************/
void Send_Message(void)
{
	uint8_t Data[8];
	
	static bool enem_color = 0;
	
	static int down_dyaw = 0;
	static int if_chassis_out_down = 0;
	
	down_dyaw = INS.Gyro[2]*1000;
	
	if(Robot_Status.robot_id>=100) enem_color = 0;
	else                           enem_color = 1;
	
	if_chassis_out_down = Link_Sit.wheel[0] + Link_Sit.wheel[1] + Link_Sit.joint[0]
	                    + Link_Sit.joint[1] + Link_Sit.joint[2] + Link_Sit.joint[3];
	
	if(if_chassis_out_down == 6) Data[0] = (             0 << 0) + (enem_color << 1); //底盘断电(云台保持正常)
	else                         Data[0] = (Flag.fall_flag << 0) + (enem_color << 1);
	
	Data[1] = Robot_Status.shooter_barrel_heat_limit >> 8;
	Data[2] = Robot_Status.shooter_barrel_heat_limit & 0x00FF;
	Data[3] = Power_Heat_Data.shooter_17mm_barrel_heat >> 8;
	Data[4] = Power_Heat_Data.shooter_17mm_barrel_heat & 0x00FF;
	Data[5] = Robot_Status.shooter_barrel_cooling_value;
  Data[6] = down_dyaw >> 8;
	Data[7] = down_dyaw & 0x00FF;

	Can_TxMessage(&hcan1,0x104,8,Data);
}
