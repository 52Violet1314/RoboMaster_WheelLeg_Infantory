#include "sbus.h"
#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include "task.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
/**
 * @brief SBUS数据获取函数
 * @param sbus_rx_buf SBUS接收缓冲区
 * @param channels 通道数据输出数组
 * @param SBUSFrameStatus 帧状态指针
 */
void SBUS_DataGet(uint8_t *sbus_rx_buf, int16_t *channels,
                  Data_Frame_Status_Typedef *SBUSFrameStatus) {
  switch (*SBUSFrameStatus) {
  case Frame_Lost:
    // 帧丢失状态：检查起始字节
    if (sbus_rx_buf[0] == 0x0F) {
      *SBUSFrameStatus = Frame_Receiving;
      HAL_UART_Receive_DMA(&huart5, (uint8_t *)sbus_rx_buf, 24);
    } else {
      HAL_UART_Receive_DMA(&huart5, (uint8_t *)sbus_rx_buf, 1);
    }
    break;
    
  case Frame_Receiving:
    // 帧接收中状态：检查结束字节
    if (sbus_rx_buf[23] == 0x00) {
      *SBUSFrameStatus = Frame_Received;
      
      // 解析16个通道数据（11位分辨率）
      channels[0] = ((((sbus_rx_buf[0] | (sbus_rx_buf[1] << 8)) & 0x07FF) / 783) * 800) - 800;  // 通道1
      channels[1] = (((sbus_rx_buf[1] >> 3) | (sbus_rx_buf[2] << 5)) & 0x07FF) - 992;           // 通道2
      channels[2] = (((sbus_rx_buf[2] >> 6) | (sbus_rx_buf[3] << 2) | (sbus_rx_buf[4] << 10)) & 0x07FF) - 992;  // 通道3
      channels[3] = (((sbus_rx_buf[4] >> 1) | (sbus_rx_buf[5] << 7)) & 0x07FF) - 992;           // 通道4
      channels[4] = (((sbus_rx_buf[5] >> 4) | (sbus_rx_buf[6] << 4)) & 0x07FF) - 992;           // 通道5
      channels[5] = (((sbus_rx_buf[6] >> 7) | (sbus_rx_buf[7] << 1) | (sbus_rx_buf[8] << 9)) & 0x07FF) - 992;  // 通道6
      channels[6] = (((sbus_rx_buf[8] >> 2) | (sbus_rx_buf[9] << 6)) & 0x07FF) - 992;           // 通道7
      channels[7] = (((sbus_rx_buf[9] >> 5) | (sbus_rx_buf[10] << 3)) & 0x07FF) - 992;          // 通道8
      channels[8] = ((sbus_rx_buf[11] | (sbus_rx_buf[12] << 8)) & 0x07FF) - 992;                // 通道9
      channels[9] = (((sbus_rx_buf[12] >> 3) | (sbus_rx_buf[13] << 5)) & 0x07FF) - 992;          // 通道10
      channels[10] = (((sbus_rx_buf[13] >> 6) | (sbus_rx_buf[14] << 2) | (sbus_rx_buf[15] << 10)) & 0x07FF);  // 通道11
      channels[11] = (((sbus_rx_buf[15] >> 1) | (sbus_rx_buf[16] << 7)) & 0x07FF);               // 通道12
      channels[12] = (((sbus_rx_buf[16] >> 4) | (sbus_rx_buf[17] << 4)) & 0x07FF);               // 通道13
      channels[13] = (((sbus_rx_buf[17] >> 7) | (sbus_rx_buf[18] << 1) | (sbus_rx_buf[19] << 9)) & 0x07FF);  // 通道14
      channels[14] = (((sbus_rx_buf[19] >> 2) | (sbus_rx_buf[20] << 6)) & 0x07FF);               // 通道15
      channels[15] = (((sbus_rx_buf[20] >> 5) | (sbus_rx_buf[21] << 3)) & 0x07FF);               // 通道16

      // 标志位检测
      if (sbus_rx_buf[22] & 0x04) {
        // 信号丢失保护
      }

      if (sbus_rx_buf[22] & 0x08) {
        // 帧丢失保护
      }
      
      *SBUSFrameStatus = Frame_Received;
      HAL_UART_Receive_DMA(&huart5, (uint8_t *)sbus_rx_buf, 25);
    } else {
      *SBUSFrameStatus = Frame_Lost;
      HAL_UART_Receive_DMA(&huart5, (uint8_t *)sbus_rx_buf, 1);
    }
    break;
    
  case Frame_Received:
    break;
    
  case Frame_Error:
    // 帧错误状态
    break;
  }
    memset(sbus_rx_buf, 0, 25);
    HAL_UART_Receive_DMA(&huart5, (uint8_t *)sbus_rx_buf, 25);
}
