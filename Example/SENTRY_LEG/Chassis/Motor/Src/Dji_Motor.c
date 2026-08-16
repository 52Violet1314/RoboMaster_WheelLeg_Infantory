#include "Dji_Motor.h"

/*******************************************************************************************************
获取DJI电机传回的数值
********************************************************************************************************/
void Get_Motor_Measure(motor_measure_t *motor_measure, uint8_t *rx_data)                                    
{                                                                   
	motor_measure->last_ecd      = motor_measure->ecd;                                   
	motor_measure->ecd           = (uint16_t)(rx_data[0] << 8 | rx_data[1]);            
	motor_measure->speed_rpm     = (uint16_t)(rx_data[2] << 8 | rx_data[3]);      
	motor_measure->given_current = (uint16_t)(rx_data[4] << 8 | rx_data[5]);  
	motor_measure->temperate     = rx_data[6];   

	//判断电机是否离线
	if(motor_measure->dji_link[0]>=1000) motor_measure->dji_link[0]=0;
	else motor_measure->dji_link[0]++;
}

/*******************************************************************************************************
指定电流控制DJI电机输出(id = 0x200 / 0x1FF)
********************************************************************************************************/
void Dji_Motor_Ctrl(CAN_HandleTypeDef* hcan,uint32_t id,int16_t one,int16_t two,int16_t tree,int16_t four)
{
	uint8_t data[8];
	
	data[0] = one >> 8;
	data[1] = one;
	data[2] = two >> 8;
	data[3] = two;
	data[4] = tree >> 8;
	data[5] = tree;
	data[6] = four >> 8;
	data[7] = four;
	
	Can_TxMessage(hcan,id,8,data);
}
