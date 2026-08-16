#include "Vofa.h"

//INCLUDE部分
#include "VT03.h"
#include "Nuc_Task.h"
#include "Gimbal_Task.h"
//EXTERN部分
extern float dt;
extern UART_HandleTypeDef huart1;
//全局变量定义部分
TypedefVofa Send_Array[7];

/*******************************************************************************************************
对VOFA传输的数据结构体进行初始化
********************************************************************************************************/
void Vofa_Init(void)
{
	Send_Array[6].v_u8[0] = 0x00;
	Send_Array[6].v_u8[1] = 0x00;
	Send_Array[6].v_u8[2] = 0x80;
	Send_Array[6].v_u8[3] = 0x7F;
}

/*******************************************************************************************************
VOFA发送函数
********************************************************************************************************/
void Vofa_Send_Message(void)
{
	Send_Array[0].v_f = VT03.mouse_left;             
	Send_Array[1].v_f = VT03.mouse_z;
	Send_Array[2].v_f = 3;
	Send_Array[3].v_f = 4;
 	Send_Array[4].v_f =	0;
	Send_Array[5].v_f = 0;

//  CDC_Transmit_FS(&Send_Array[0].v_u8[0],4*7);
	HAL_UART_Transmit_DMA(&huart1, &Send_Array[0].v_u8[0], 4*7);
}
