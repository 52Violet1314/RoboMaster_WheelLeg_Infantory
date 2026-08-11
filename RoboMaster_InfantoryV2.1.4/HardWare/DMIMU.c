#include "DMIMU.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "fdcan.h"
#include <string.h>

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max]
*内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max]
*内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

imu_t imu;

void IMU_Init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan,imu_t *IMU)
{
    IMU->can_id = can_id;
    IMU->mst_id = mst_id;
    IMU->can_handle = hfdcan;
}

void IMU_Write_Reg(uint8_t reg_id,uint32_t data,imu_t *IMU)
{
    if(IMU->can_handle==NULL)
		return;
	
	FDCAN_TxHeaderTypeDef tx_header;
	
	uint8_t buf[8]={0xCC,reg_id,CMD_WRITE,0xDD,0,0,0,0};
	memcpy(buf+4,&data,4);
	
	tx_header.DataLength=FDCAN_DLC_BYTES_8;
	tx_header.IdType=FDCAN_STANDARD_ID;
	tx_header.TxFrameType=FDCAN_DATA_FRAME;
	tx_header.Identifier=IMU->can_id;
	tx_header.FDFormat=FDCAN_CLASSIC_CAN;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										
	tx_header.MessageMarker = 0x00; 			      

	if(HAL_FDCAN_GetTxFifoFreeLevel(IMU->can_handle)>2)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(IMU->can_handle,&tx_header,buf);
	}
}

void IMU_Read_Reg(uint8_t reg_id,imu_t *IMU)
{
    if(IMU->can_handle==NULL)
		return;
	
	FDCAN_TxHeaderTypeDef tx_header;
	
	uint8_t buf[8]={0xCC,reg_id,CMD_READ,0xDD,0,0,0,0};
	
	tx_header.DataLength=FDCAN_DLC_BYTES_8;
	tx_header.IdType=FDCAN_STANDARD_ID;
	tx_header.TxFrameType=FDCAN_DATA_FRAME;
	tx_header.Identifier=IMU->can_id;
	tx_header.FDFormat=FDCAN_CLASSIC_CAN;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										
	tx_header.MessageMarker = 0x00; 			      

	if(HAL_FDCAN_GetTxFifoFreeLevel(IMU->can_handle)>2)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(IMU->can_handle,&tx_header,buf);
	}
}

void IMU_RequestEuler(imu_t *IMU)
{
	IMU_Read_Reg(EULER_DATA,IMU);
}

void IMU_UpdateEuler(uint8_t* pData,imu_t *IMU)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	IMU->pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	IMU->yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	IMU->roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
}