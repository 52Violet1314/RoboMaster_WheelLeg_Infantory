#include "bsp_rc.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	__HAL_DMA_DISABLE(&hdma_usart3_rx);

	while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart3_rx);
	}

	hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
	hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
	hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
	hdma_usart3_rx.Instance->NDTR = dma_buf_num;

	SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

	__HAL_DMA_ENABLE(&hdma_usart3_rx);
}

void RC_Unable(void)
{
	__HAL_UART_DISABLE(&huart3);
}

void RC_Restart(uint16_t dma_buf_num)
{
	__HAL_UART_DISABLE(&huart3);
	__HAL_DMA_DISABLE(&hdma_usart3_rx);

	hdma_usart3_rx.Instance->NDTR = dma_buf_num;

	__HAL_DMA_ENABLE(&hdma_usart3_rx);
	__HAL_UART_ENABLE(&huart3);

}
