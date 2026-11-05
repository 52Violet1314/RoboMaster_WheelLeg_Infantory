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
    SCB_InvalidateDCache_by_Addr((void *)rx_buff, Size);
    SBUS_Data_Input(&sbus_ring_buffer, rx_buff, Size);
    SCB_CleanDCache_by_Addr((void *)rx_buff, BUFF_SIZE * 2);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(DataGroup, SBUS_DATA_READY_BIT, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
      portYIELD();
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE * 2);
  }

  if (huart->Instance == UART7) {
    if (Size == 0) {
      return;
    }

    /* DMA writes physical RAM, so invalidate any stale CPU cache lines first. */
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)hi14_dma_rx_buf,
        (Size + 31) & ~31u);

    memcpy(hi14_uart_rx_buf, hi14_dma_rx_buf, Size);
    hi14_rx_size = Size;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, hi14_dma_rx_buf, UART_RX_BUF_SIZE);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(INS_TaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5) {
    SCB_InvalidateDCache_by_Addr((void *)rx_buff, BUFF_SIZE * 2);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE * 2);
    memset(rx_buff, 0, BUFF_SIZE);
    SCB_CleanDCache_by_Addr((void *)rx_buff, BUFF_SIZE * 2);
  }

  if (huart->Instance == UART7) {
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)hi14_dma_rx_buf,
        (UART_RX_BUF_SIZE + 31) & ~31u);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, hi14_dma_rx_buf, UART_RX_BUF_SIZE);
  }
}
