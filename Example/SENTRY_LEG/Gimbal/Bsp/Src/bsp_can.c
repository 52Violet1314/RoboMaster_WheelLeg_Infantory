#include "bsp_can.h"

//INCLUDE部分
#include "cmsis_os.h"

/*******************************************************************************************************
CAN滤波器设置以及CAN初始化
********************************************************************************************************/
void CAN_Init(void)
{
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/*******************************************************************************************************
CAN的发送函数
********************************************************************************************************/
uint8_t Can_TxMessage(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t len,uint8_t *data)
{
	CAN_TxHeaderTypeDef CAN_TxHeader;

	CAN_TxHeader.IDE = CAN_ID_STD;
	CAN_TxHeader.StdId = id;
	CAN_TxHeader.DLC = len;
	CAN_TxHeader.RTR = CAN_RTR_DATA;
	CAN_TxHeader.TransmitGlobalTime = DISABLE;
	
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0){osDelay(1);}
	
	if(HAL_CAN_AddTxMessage(hcan,&CAN_TxHeader,data,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) 
	{
		if(HAL_CAN_AddTxMessage(hcan,&CAN_TxHeader,data,(uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) 
		{
			if(HAL_CAN_AddTxMessage(hcan,&CAN_TxHeader,data,(uint32_t*)CAN_TX_MAILBOX2) != HAL_OK)
			{
				return 1;
			}
    }
  }
	
	return 0;
}




