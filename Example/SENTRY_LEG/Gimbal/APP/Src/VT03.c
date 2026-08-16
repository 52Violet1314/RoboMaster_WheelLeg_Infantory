#include "VT03.h"

//INCLUDE部分
#include "CRCs.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
//EXTERN部分
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
//全局变量定义部分

int vt_link[3] = {0,0,0};//遥控器连接检测数组

remote_data_t VT03;

/*******************************************************************************************************
VT03初始化
********************************************************************************************************/
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t vt03_rx_buf[2][VT03_RX_BUF_NUM];

void VT03_Init(void)
{
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	__HAL_DMA_DISABLE(&hdma_usart6_rx);

	while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
	}

	hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
	hdma_usart6_rx.Instance->M0AR = (uint32_t)(vt03_rx_buf[0]);
	hdma_usart6_rx.Instance->M1AR = (uint32_t)(vt03_rx_buf[1]);
	hdma_usart6_rx.Instance->NDTR = VT03_RX_BUF_NUM;

	SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

	__HAL_DMA_ENABLE(&hdma_usart6_rx);
}

/*******************************************************************************************************
VT03数据解析
********************************************************************************************************/
void RX_TO_VT03(void)
{
	if(huart6.Instance->SR & UART_FLAG_RXNE)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart6);
	}
	else if(USART6->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart6);

		if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			this_time_rx_len = VT03_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
			hdma_usart6_rx.Instance->NDTR = VT03_RX_BUF_NUM;
			hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			if(this_time_rx_len == VT03_FRAME_LENGTH)
			{
				//如果CRC校验通过则保留数据,否则只保留上一帧数据
				if(CRC16_Verify((uint8_t *)(vt03_rx_buf[0]),sizeof(remote_data_t))) memcpy(&VT03,vt03_rx_buf[0],sizeof(remote_data_t));
				
				//判断遥控器是否离线
				if(vt_link[0]>=1000) vt_link[0]=0;
				else                 vt_link[0]++;
			}
		}
		else
		{
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			this_time_rx_len = VT03_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
			hdma_usart6_rx.Instance->NDTR = VT03_RX_BUF_NUM;
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
			
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			if(this_time_rx_len == VT03_FRAME_LENGTH)
			{
				//如果CRC校验通过则保留数据,否则只保留上一帧数据
				if(CRC16_Verify((uint8_t *)(vt03_rx_buf[1]),sizeof(remote_data_t))) memcpy(&VT03,vt03_rx_buf[1],sizeof(remote_data_t));
				
				//判断遥控器是否离线
				if(vt_link[0]>=1000) vt_link[0]=0;
				else                 vt_link[0]++;
			}
		}
	}
}









