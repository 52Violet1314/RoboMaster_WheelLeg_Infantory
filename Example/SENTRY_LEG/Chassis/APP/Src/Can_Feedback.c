#include "Can_Feedback.h"

/*
     Forward
	  0       2    DM_8009
            
   0         1   DJI_3508

    1       3    DM_8009
*/

//全局变量定义部分
Chassis_Motor_t Chassis_Motor;
Up_Cboard_Info_t Up_Cboard_Info;

/*******************************************************************************************************
接收电机的反馈值并处理(CAN1)
********************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t Rx_Data1[8];
	static CAN_RxHeaderTypeDef CAN1_RxHeader;
	
	HAL_StatusTypeDef RxState;
	RxState = HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CAN1_RxHeader,Rx_Data1);
	
	while(RxState!=HAL_OK){;}
		
	switch(CAN1_RxHeader.StdId)
	{
		//DT7数据
		case 0x080:
		{
			Rc_Ctrl.rc.ch[1]  = (Rx_Data1[0] << 8 | Rx_Data1[1]);
			Rc_Ctrl.rc.ch[4]  = (Rx_Data1[2] << 8 | Rx_Data1[3]);
			Rc_Ctrl.key.v     = (Rx_Data1[4] << 8 | Rx_Data1[5]);
			Rc_Ctrl.rc.s[0]   =  Rx_Data1[6];
			Rc_Ctrl.rc.s[1]   =  Rx_Data1[7];
			break;
		}
		//上板数据
		case 0x100:
		{
			Up_Cboard_Info.Friction_Speed = (Rx_Data1[0] << 8 | Rx_Data1[1]);
			Up_Cboard_Info.up_err_num     = (Rx_Data1[2] << 8 | Rx_Data1[3]);
			Up_Cboard_Info.ecd_yaw_6020   = (Rx_Data1[4] << 8 | Rx_Data1[5]);
			VT03.key                      = (Rx_Data1[6] << 8 | Rx_Data1[7]);
			break;
		}
		case 0x120:
		{
			Up_Cboard_Info.Auto_Aim_Flag    = Rx_Data1[0];
			Up_Cboard_Info.Friction_Status  = Rx_Data1[1];
			VT03.mode_sw                    = Rx_Data1[2];
			break;
		}
		//DM_8009数据
		case 0x30:
		{
			Receive_Motor_Data(&Chassis_Motor.DM_8009[0],Rx_Data1,DM8009);
			break;
		}
		case 0x40:
		{
			Receive_Motor_Data(&Chassis_Motor.DM_8009[1],Rx_Data1,DM8009);
			break;
		}
		default:
		{
			break;
		}
	}
}

/*******************************************************************************************************
接收电机的反馈值并处理(CAN2)
********************************************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static uint8_t Rx_Data2[8]; 
  static CAN_RxHeaderTypeDef CAN2_RxHeader;
	
	HAL_StatusTypeDef RxState;
	RxState=HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&CAN2_RxHeader,Rx_Data2);

	while(RxState!=HAL_OK){;}
		
	switch(CAN2_RxHeader.StdId)
	{
		//DM_8009数据
		case 0x10:
		{
			Receive_Motor_Data(&Chassis_Motor.DM_8009[2],Rx_Data2,DM8009);
			break;
		}
		case 0x20:
		{
			Receive_Motor_Data(&Chassis_Motor.DM_8009[3],Rx_Data2,DM8009);
			break;
		}
		//DJI_3508数据
		case 0x201:
		{
			Get_Motor_Measure(&Chassis_Motor.Dji_3508[1],Rx_Data2);
			break;
		}
		case 0x202:
		{
			Get_Motor_Measure(&Chassis_Motor.Dji_3508[0],Rx_Data2);
			break;
		}
		//超电数据
		case 0x612:
		{
			pm_od.p_out = (uint16_t)(Rx_Data2[0] << 8 | Rx_Data2[1]);
			pm_od.v_out = (uint16_t)(Rx_Data2[2] << 8 | Rx_Data2[3]);
			pm_od.i_out = (uint16_t)(Rx_Data2[4] << 8 | Rx_Data2[5]);
			break;
		}
		case 0x610:
		{
			pm_od.sta_code.all = (uint16_t)(Rx_Data2[0] << 8 | Rx_Data2[1]);	
			pm_od.err_code     = (uint16_t)(Rx_Data2[2] << 8 | Rx_Data2[3]);
			break;
		}
		default:
		{
			break;
		}
	}
}

