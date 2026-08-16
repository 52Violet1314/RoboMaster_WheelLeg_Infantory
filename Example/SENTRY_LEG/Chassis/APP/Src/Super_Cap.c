#include "Super_Cap.h"

//全局变量定义部分
float Input_Power;
volatile pm_od_t pm_od;

/*******************************************************************************************************
设定功率值
********************************************************************************************************/
//save_flag 
//0x0000：指令不保存至EEPROM
//0x0001：  指令保存至EEPROM
void Pm_Power_Set(CAN_HandleTypeDef *hcan,uint16_t power,uint8_t save_flag)
{
	uint8_t data[4];
	
	data[0] = (uint8_t)(power >>    8);
	data[1] = (uint8_t)(power &  0xFF);
	data[2] = 0x00;
	data[3] = (save_flag == 0x01);			
	
	Can_TxMessage(hcan,0x601,0x04,data);
}

/*******************************************************************************************************
设定电压值
********************************************************************************************************/
//save_flag 
//0x0000：指令不保存至EEPROM
//0x0001：  指令保存至EEPROM
void Pm_Voltage_Set(CAN_HandleTypeDef *hcan,uint16_t voltage,uint8_t save_flag)
{
	uint8_t data[4];

	data[0] = (uint8_t)(voltage >>    8);
	data[1] = (uint8_t)(voltage &  0xFF);
	data[2] = 0x00;
	data[3] = (save_flag == 0x01);			
	
	Can_TxMessage(hcan,0x602,0x04,data);
}

/*******************************************************************************************************
使用远程帧发送功率的标识符
********************************************************************************************************/
void Can_Power_Read(CAN_HandleTypeDef *hcan) //远程帧
{
	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];
	uint32_t             send_mail_box;
	power_tx_message.StdId = 0x612;          // 发送相应的标识符
	power_tx_message.IDE   = CAN_ID_STD;     // 标准帧           
	power_tx_message.RTR   = CAN_RTR_REMOTE; // 远程帧           
	power_tx_message.DLC   = 0x00;
	
	HAL_CAN_AddTxMessage(hcan, &power_tx_message, power_can_send_data, &send_mail_box);
}

/*******************************************************************************************************
使用远程帧发送错误的标识符
********************************************************************************************************/
void Can_Err_Read(CAN_HandleTypeDef *hcan) //远程帧
{
	CAN_TxHeaderTypeDef  power_tx_message;
	uint8_t              power_can_send_data[8];
	uint32_t             send_mail_box;
	power_tx_message.StdId = 0x610;          // 发送相应的标识符
	power_tx_message.IDE   = CAN_ID_STD;     // 标准帧           
	power_tx_message.RTR   = CAN_RTR_REMOTE; // 远程帧          
	power_tx_message.DLC   = 0x00;
	
	HAL_CAN_AddTxMessage(hcan, &power_tx_message, power_can_send_data, &send_mail_box);
}
