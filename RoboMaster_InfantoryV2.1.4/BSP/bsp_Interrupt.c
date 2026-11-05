#include "bsp_Interrupt.h"

#include <string.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "bsp_UART.h"
#include "app_Task.hpp"
#include "Hipnuv_hi14.h"
#include "usart.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == UART5) {
    /* UART5 DMA buffer holds exactly one SBUS frame.  Reject any impossible
     * callback length before it reaches the ring buffer. */
    if (Size == 0U) {
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE);
      if (huart5.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
      }
      return;
    }
    if (Size > BUFF_SIZE) {
      Size = BUFF_SIZE;
    }
    SCB_InvalidateDCache_by_Addr((void *)rx_buff, Size);
    SBUS_Data_Input(&sbus_ring_buffer, rx_buff, Size);
    SCB_CleanDCache_by_Addr((void *)rx_buff, BUFF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE);
    if (huart5.hdmarx != NULL) {
      __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
  }

  if (huart->Instance == UART7) {
    if (Size == 0) {
      HAL_UARTEx_ReceiveToIdle_DMA(&huart7, hi14_dma_rx_buf, UART_RX_BUF_SIZE);
      if (huart7.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
      }
      return;
    }
    if (Size > UART_RX_BUF_SIZE) {
      Size = UART_RX_BUF_SIZE;
    }

    /* DMA writes physical RAM, so invalidate any stale CPU cache lines first. */
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)hi14_dma_rx_buf,
        (Size + 31) & ~31u);

    if (HI14_RxQueueHandle != NULL) {
      static Hi14_RxBlock_t rx_block;
      static Hi14_RxBlock_t discarded_block;
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;

      rx_block.length = Size;
      memcpy(rx_block.data, hi14_dma_rx_buf, Size);
      if (xQueueSendFromISR(HI14_RxQueueHandle, &rx_block,
                            &xHigherPriorityTaskWoken) != pdPASS) {
        /* IMU control only needs the newest frame: discard the oldest block
         * before publishing this one when the two-slot queue is full. */
        (void)xQueueReceiveFromISR(HI14_RxQueueHandle, &discarded_block,
                                   &xHigherPriorityTaskWoken);
        (void)xQueueSendFromISR(HI14_RxQueueHandle, &rx_block,
                                &xHigherPriorityTaskWoken);
      }
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, hi14_dma_rx_buf, UART_RX_BUF_SIZE);
    if (huart7.hdmarx != NULL) {
      __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5) {
    SCB_InvalidateDCache_by_Addr((void *)rx_buff, BUFF_SIZE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE);
    if (huart5.hdmarx != NULL) {
      __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
    memset(rx_buff, 0, BUFF_SIZE);
    SCB_CleanDCache_by_Addr((void *)rx_buff, BUFF_SIZE);
  }

  if (huart->Instance == UART7) {
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)hi14_dma_rx_buf,
        (UART_RX_BUF_SIZE + 31) & ~31u);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, hi14_dma_rx_buf, UART_RX_BUF_SIZE);
    if (huart7.hdmarx != NULL) {
      __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
  }
}
