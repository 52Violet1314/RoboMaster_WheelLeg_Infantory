#include "Nuc_Task.h"

//INCLUDE部分
#include "Ins_Task.h"
#include "Check_Task.h"
#include "Gimbal_Task.h"
#include "Can_Feedback.h"
//全局变量定义部分
Aim_Rx aim_rx;
Aim_Tx aim_tx;

static uint8_t usb_tx_buf[APP_TX_DATA_SIZE];

bool nuc_ready_flag; //当前线程初始化完成标志 

/*******************************************************************************************************
NUC任务初始化
********************************************************************************************************/
void Nuc_Init(void)
{
	nuc_ready_flag = 1;
	
	//等待
	while(all_ready_flag!=1){osDelay(1);}
}

/*******************************************************************************************************
NUC任务
********************************************************************************************************/
void Nuc_Task(void)
{
	Send_Packet();
}

/*******************************************************************************************************
向上位机发送数据
********************************************************************************************************/
void Send_Packet(void)
{
	aim_tx.head = FH_TX;
	
	//设置自瞄模式
	aim_tx.mode = 1;
	
	aim_tx.enem_color = Down_Cboard_Info.enem_color;
	
	//上传陀螺仪q矩阵
	aim_tx.q[0] = INS.q[0];
	aim_tx.q[1] = INS.q[1];
	aim_tx.q[2] = INS.q[2];
	aim_tx.q[3] = INS.q[3];
	
	//上传云台状态
	aim_tx.yaw = INS.Yaw*Ang_PI;
	aim_tx.yaw_vel = 0;
	aim_tx.pitch = INS.Roll*Ang_PI;
	aim_tx.pitch_vel = 0;
	
	//上传子弹参数
	aim_tx.bullet_count = 1;
	aim_tx.bullet_speed = 11.6;
	
	aim_tx.checksum = CRC16_Calculate((uint8_t *)(&aim_tx),sizeof(Aim_Tx) - 2);
	
	//发送
  memcpy(usb_tx_buf,&aim_tx,sizeof(Aim_Tx));
  CDC_Transmit_FS(usb_tx_buf,sizeof(Aim_Tx));
}

/*******************************************************************************************************
接收上位机数据
********************************************************************************************************/
void Recieve_Host(uint8_t* buff) 
{
	if(buff[0] == FH_RX) //与接收到的帧头一致
	{
		//如果CRC校验通过则保留数据,否则只保留上一帧数据
		if(CRC16_Verify((uint8_t *)(buff), sizeof(Aim_Rx))) memcpy(&aim_rx,buff,sizeof(Aim_Rx));
	}	
}

