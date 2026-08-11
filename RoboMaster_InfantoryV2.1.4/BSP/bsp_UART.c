#include "bsp_UART.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"

// 全局变量定义
SBUS_Data_t SBUS_Data;
RingBuffer_t sbus_ring_buffer;

// 环形缓冲区初始化
void RingBuffer_Init(RingBuffer_t *rb)
{
  memset(rb->buffer, 0, SBUS_BUFFER_SIZE);
  rb->head = 0;
  rb->tail = 0;
  rb->length = 0;
  rb->overflow = 0;
  rb->frame_count = 0;
  rb->error_count = 0;
}

// 判断环形缓冲区是否为空
uint8_t RingBuffer_IsEmpty(RingBuffer_t *rb)
{
  return (rb->length == 0);
}

// 判断环形缓冲区是否已满
uint8_t RingBuffer_IsFull(RingBuffer_t *rb)
{
  return (rb->length == SBUS_BUFFER_SIZE);
}

// 获取环形缓冲区当前数据长度
uint16_t RingBuffer_GetLength(RingBuffer_t *rb)
{
  return rb->length;
}

// 向环形缓冲区写入一个字节
uint8_t RingBuffer_WriteByte(RingBuffer_t *rb, uint8_t data)
{
  if (RingBuffer_IsFull(rb))
  {
    rb->overflow = 1;
    return 0;  // 缓冲区已满，写入失败
  }
  
  rb->buffer[rb->head] = data;
  rb->head = (rb->head + 1) % SBUS_BUFFER_SIZE;
  rb->length++;
  
  return 1;  // 写入成功
}

// 从环形缓冲区读取一个字节
uint8_t RingBuffer_ReadByte(RingBuffer_t *rb, uint8_t *data)
{
  if (RingBuffer_IsEmpty(rb))
  {
    return 0;  // 缓冲区为空，读取失败
  }
  
  *data = rb->buffer[rb->tail];
  rb->tail = (rb->tail + 1) % SBUS_BUFFER_SIZE;
  rb->length--;
  
  return 1;  // 读取成功
}

// 向环形缓冲区写入多个字节
uint16_t RingBuffer_WriteData(RingBuffer_t *rb, uint8_t *data, uint16_t len)
{
  uint16_t written = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    if (RingBuffer_WriteByte(rb, data[i]))
    {
      written++;
    }
    else
    {
      break;  // 缓冲区已满，停止写入
    }
  }
  return written;
}

// 从环形缓冲区读取多个字节
uint16_t RingBuffer_ReadData(RingBuffer_t *rb, uint8_t *data, uint16_t len)
{
  uint16_t read = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    if (RingBuffer_ReadByte(rb, &data[i]))
    {
      read++;
    }
    else
    {
      break;  // 缓冲区为空，停止读取
    }
  }
  return read;
}

// 清空环形缓冲区
void RingBuffer_Clear(RingBuffer_t *rb)
{
  rb->head = 0;
  rb->tail = 0;
  rb->length = 0;
  rb->overflow = 0;
}

// SBUS环形缓冲区初始化
void SBUS_RingBuffer_Init(void)
{
  RingBuffer_Init(&sbus_ring_buffer);
}

// SBUS数据输入到环形缓冲区
void SBUS_Data_Input(RingBuffer_t *rb, uint8_t *data, uint16_t len)
{
  RingBuffer_WriteData(rb, data, len);
}

// 从环形缓冲区中解析SBUS数据帧
uint8_t SBUS_Frame_Parse_From_Buffer(RingBuffer_t *rb, SBUS_Data_t *sbus_data)
{
  uint16_t i = 0;
  uint8_t found = 0;
  uint8_t temp_buffer[SBUS_FRAME_SIZE];
  
  // 确保缓冲区中有足够的数据
  if (rb->length < SBUS_FRAME_SIZE)
  {
    return 0;
  }
  
  // 查找SBUS帧头
  for (i = 0; i <= rb->length - SBUS_FRAME_SIZE; i++)
  {
    // 计算当前要检查的位置
    uint16_t check_pos = (rb->tail + i) % SBUS_BUFFER_SIZE;
    
    // 找到帧头
    if (rb->buffer[check_pos] == SBUS_HEAD)
    {
      // 检查帧尾
      uint16_t end_pos = (check_pos + SBUS_FRAME_SIZE - 1) % SBUS_BUFFER_SIZE;
      if (rb->buffer[end_pos] == SBUS_END)
      {
        found = 1;
        break;
      }
    }
  }
  
  if (!found)
  {
    // 没有找到完整的帧，移动tail指针到当前检查位置的下一个，丢弃无效数据
    rb->tail = (rb->tail + i) % SBUS_BUFFER_SIZE;
    rb->length -= i;
    rb->error_count++;
    return 0;
  }
  
  // 移动tail指针到帧头位置
  rb->tail = (rb->tail + i) % SBUS_BUFFER_SIZE;
  rb->length -= i;
  
  // 读取完整的SBUS帧
  for (i = 0; i < SBUS_FRAME_SIZE; i++)
  {
    temp_buffer[i] = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % SBUS_BUFFER_SIZE;
    rb->length--;
  }
  
  // 解析SBUS数据帧
  if ((temp_buffer[0] != SBUS_HEAD) || (temp_buffer[24] != SBUS_END))
  {
    rb->error_count++;
    return 0;
  }

  if (temp_buffer[23] == 0x0C)
    sbus_data->SBUS_Status = Data_Error;
  else
    sbus_data->SBUS_Status = Data_Ok;

  sbus_data->Channel[0] = (((temp_buffer[1] | temp_buffer[2] << 8) & 0x07FF) - 1024);
  sbus_data->Channel[1] = (((temp_buffer[2] >> 3 | temp_buffer[3] << 5) & 0x07FF) - 1024);
  sbus_data->Channel[2] = (((temp_buffer[3] >> 6 | temp_buffer[4] << 2 | 
                            temp_buffer[5] << 10) & 0x07FF) - 1024);
  sbus_data->Channel[3] = (((temp_buffer[5] >> 1 | temp_buffer[6] << 7) & 0x07FF) - 1024);
  sbus_data->Channel[4] = (((temp_buffer[6] >> 4 | temp_buffer[7] << 4) & 0x07FF) - 1024);
  sbus_data->Channel[5] = (((temp_buffer[7] >> 7 | temp_buffer[8] << 1 | 
                            temp_buffer[9] << 9) & 0x07FF) - 1024);
  sbus_data->Channel[6] = (((temp_buffer[9] >> 2 | temp_buffer[10] << 6) & 0x07FF) - 1024);
  sbus_data->Channel[7] = (((temp_buffer[10] >> 5 | temp_buffer[11] << 3) & 0x07FF) - 1024);
  sbus_data->Channel[8] = (((temp_buffer[12] | temp_buffer[13] << 8) & 0x07FF) - 1024);
  sbus_data->Channel[9] = (((temp_buffer[13] >> 3 | temp_buffer[14] << 5) & 0x07FF) - 1024);
  sbus_data->Channel[10] = (((temp_buffer[14] >> 6 | temp_buffer[15] << 2 | 
                             temp_buffer[16] << 10) & 0x07FF) - 1024);
  sbus_data->Channel[11] = (((temp_buffer[16] >> 1 | temp_buffer[17] << 7) & 0x07FF) - 1024);
  sbus_data->Channel[12] = (((temp_buffer[17] >> 4 | temp_buffer[18] << 4) & 0x07FF) - 1024);
  sbus_data->Channel[13] = (((temp_buffer[18] >> 7 | temp_buffer[19] << 1 | 
                             temp_buffer[20] << 9) & 0x07FF) - 1024);
  sbus_data->Channel[14] = (((temp_buffer[20] >> 2 | temp_buffer[21] << 6) & 0x07FF) - 1024);
  sbus_data->Channel[15] = (((temp_buffer[21] >> 5 | temp_buffer[22] << 3) & 0x07FF) - 1024);
  
  rb->frame_count++;
  return 1;  // 解析成功
}

// SBUS调试信息打印
void SBUS_Debug_Print(RingBuffer_t *rb)
{
  uart_print("SBUS Ring Buffer Status:\r\n");
  uart_print("  Length: %d/%d\r\n", rb->length, SBUS_BUFFER_SIZE);
  uart_print("  Head: %d, Tail: %d\r\n", rb->head, rb->tail);
  uart_print("  Overflow: %d\r\n", rb->overflow);
  uart_print("  Frame Count: %d\r\n", rb->frame_count);
  uart_print("  Error Count: %d\r\n", rb->error_count);
}

// 原始SBUS数据帧解析函数（保持兼容）
void sbus_frame_parse(SBUS_Data_t *sbus_data, uint8_t* rx_buff) {
  if ((rx_buff[0] != SBUS_HEAD) || (rx_buff[24] != SBUS_END))
    return;

  if (rx_buff[23] == 0x0C)
    sbus_data->SBUS_Status = Data_Error;
  else
    sbus_data->SBUS_Status = Data_Ok;

  sbus_data->Channel[0] = 
      ((rx_buff[1] | rx_buff[2] << 8) & 0x07FF);
  sbus_data->Channel[1] = 
      ((rx_buff[2] >> 3 | rx_buff[3] << 5) & 0x07FF);
  sbus_data->Channel[2] = ((rx_buff[3] >> 6 | rx_buff[4] << 2 | 
                            rx_buff[5] << 10) & 
                           0x07FF);
  sbus_data->Channel[3] = 
      ((rx_buff[5] >> 1 | rx_buff[6] << 7) & 0x07FF);
  sbus_data->Channel[4] = 
      ((rx_buff[6] >> 4 | rx_buff[7] << 4) & 0x07FF);
  sbus_data->Channel[5] = ((rx_buff[7] >> 7 | rx_buff[8] << 1 | 
                            rx_buff[9] << 9) & 
                           0x07FF);
  sbus_data->Channel[6] = 
      ((rx_buff[9] >> 2 | rx_buff[10] << 6) & 0x07FF);
  sbus_data->Channel[7] = 
      ((rx_buff[10] >> 5 | rx_buff[11] << 3) & 0x07FF);
  sbus_data->Channel[8] = 
      ((rx_buff[12] | rx_buff[13] << 8) & 0x07FF);
  sbus_data->Channel[9] = 
      ((rx_buff[13] >> 3 | rx_buff[14] << 5) & 0x07FF);
  sbus_data->Channel[10] = ((rx_buff[14] >> 6 | rx_buff[15] << 2 | 
                            rx_buff[16] << 10) & 
                           0x07FF);
  sbus_data->Channel[11] = 
      ((rx_buff[16] >> 1 | rx_buff[17] << 7) & 0x07FF);
  sbus_data->Channel[12] = 
      ((rx_buff[17] >> 4 | rx_buff[18] << 4) & 0x07FF);
  sbus_data->Channel[13] = ((rx_buff[18] >> 7 | rx_buff[19] << 1 | 
                            rx_buff[20] << 9) & 
                           0x07FF);
  sbus_data->Channel[14] = 
      ((rx_buff[20] >> 2 | rx_buff[21] << 6) & 0x07FF);
  sbus_data->Channel[15] = 
      ((rx_buff[21] >> 5 | rx_buff[22] << 3) & 0x07FF);
}

// DMA 发送完成信号量（在 bsp_UART_Init 中创建，初始为 available）
static SemaphoreHandle_t uart_dma_done = NULL;

// DMA 双缓冲（放在 SRAM4/RAM_D1，DTCM DMA 不可达）
static char uart_dma_buf[2][UART_BUFFER_SIZE]
    __attribute__((aligned(32), section(".sram4")));

void bsp_UART_Init(void) {
  uart_dma_done = xSemaphoreCreateBinary();
  xSemaphoreGive(uart_dma_done);  // 初始状态：可用
}

// DMA 发送完成回调（在 USART1_IRQHandler → UART_EndTransmit_IT 中调用）
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1 && uart_dma_done != NULL) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart_dma_done, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// UART打印函数（DMA 方式 + 双缓冲 + 信号量同步）
// H7 DMA TX 流程：DMA TC 中断搬运完 → UART TC 中断发完最后一字节 → 回调
void uart_print(const char *fmt, ...) {
  static uint8_t buf_idx = 0;

  // 等待上一次 DMA 传输完成（调度器启动前 sem=NULL，跳过）
  if (uart_dma_done != NULL) {
    xSemaphoreTake(uart_dma_done, pdMS_TO_TICKS(20));
  }

  buf_idx ^= 1;
  char *buf = uart_dma_buf[buf_idx];

  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, UART_BUFFER_SIZE, fmt, args);
  va_end(args);

  if (len > 0) {
    // DCache 写回：H7 DMA 读物理 RAM，需确保缓存已刷新
    SCB_CleanDCache_by_Addr((uint32_t *)buf, (len + 31) & ~31);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len);
  }
}

// 阻塞式打印：轮询 HAL_UART_Transmit，不依赖 DMA / RTOS 原语
// 适用场景：调度器启动前、临界区、断言失败后
// 注意：不要在 ISR 中调用 —— 轮询会长时间占用 CPU
#define BLOCKING_PRINT_BUF_SIZE 256
void uart_print_blocking(const char *fmt, ...) {
  char buf[BLOCKING_PRINT_BUF_SIZE];

  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len <= 0) return;
  if ((size_t)len >= sizeof(buf)) len = (int)(sizeof(buf) - 1);

  HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
}
