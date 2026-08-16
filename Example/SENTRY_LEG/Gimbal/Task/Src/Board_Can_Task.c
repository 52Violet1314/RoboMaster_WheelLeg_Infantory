#include "Board_Can_Task.h"

//INCLUDE部分
#include "VT03.h"
#include "Ins_Task.h"
#include "Check_Task.h"
#include "Limit_Task.h"
#include "Gimbal_Task.h"
//全局变量定义部分
bool board_ready_flag; //当前线程初始化完成标志 

/*******************************************************************************************************
Board_Can任务初始化
********************************************************************************************************/
void Board_Can_Init(void)
{
	board_ready_flag = 1;
	
	while(!all_ready_flag) {osDelay(1);}
}

/*******************************************************************************************************
Board_Can任务
********************************************************************************************************/
void Board_Can_Task(void)
{
	Send_Rc_DT7();
	Send_Message_1();
	Send_Message_2();
}

/*******************************************************************************************************
向下板发送遥控器信息
********************************************************************************************************/
void Send_Rc_DT7(void)
{
	uint8_t Data[8];
	
	Data[0] = Rc_Ctrl.rc.ch[1]>>8;
	Data[1] = Rc_Ctrl.rc.ch[1]&0x00FF;
	Data[2] = Rc_Ctrl.rc.ch[4]>>8;
	Data[3] = Rc_Ctrl.rc.ch[4]&0x00FF;
	Data[4] = Rc_Ctrl.key.v>>8;
	Data[5] = Rc_Ctrl.key.v&0x00FF;
	Data[6] = Rc_Ctrl.rc.s[0];	
	Data[7] = Rc_Ctrl.rc.s[1];
	
	Can_TxMessage(&hcan1,0x080,8,Data);
}

/*******************************************************************************************************
板间通讯
********************************************************************************************************/
void Send_Message_1(void)
{
	uint8_t Data[8];
	
	static int friction_speed = 0;
	
	friction_speed = Friction_Speed + Friction_Speed_Comp;
	
	Data[0] = friction_speed>>8;   //当前弹速
	Data[1] = friction_speed&0x00FF;
	Data[2] = Link_Sit.err_num>>8; //报错码
	Data[3] = Link_Sit.err_num&0x00FF;
	Data[4] = Gimbal_Motor.Dji_6020[0].ecd>>8; 
	Data[5] = Gimbal_Motor.Dji_6020[0].ecd&0x00FF; 
	Data[6] = VT03.key>>8;
	Data[7] = VT03.key&0x00FF; 

	Can_TxMessage(&hcan1,0x100,8,Data);
}

void Send_Message_2(void)
{
	uint8_t Data[8];
	
	Data[0] = aim_rx.mode;       //自瞄标志位      
	Data[1] = Shoot_Condition;   //摩擦轮状态  
	Data[2] = VT03.mode_sw; 
	Data[3] = 0;
	Data[4] = 0;
	Data[5] = 0; 
	Data[6] = 0; 
	Data[7] = 0;  

	Can_TxMessage(&hcan1,0x120,8,Data);
}

