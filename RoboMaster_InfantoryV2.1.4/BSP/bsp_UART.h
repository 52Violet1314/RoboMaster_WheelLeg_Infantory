#ifndef __BSP_UART_H__
#define __BSP_UART_H__
#include <stdint.h>
#include "stm32h7xx_hal.h"  // 包含HAL基础定义
#define UART_BUFFER_SIZE 512
#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define SBUS_FRAME_SIZE 25
#define SBUS_BUFFER_SIZE 250  // 10帧SBUS数据的大小

// SBUS数据状态枚举
typedef enum {
  Data_Error = 0,
  Data_Ok = 1,
} SBUS_Status_t;

// SBUS数据结构体
typedef struct {
  int16_t Channel[16];
  SBUS_Status_t SBUS_Status;
} SBUS_Data_t;

// 环形缓冲区结构体
typedef struct {
  uint8_t buffer[SBUS_BUFFER_SIZE];  // 缓冲区数组
  uint16_t head;                     // 写入指针
  uint16_t tail;                     // 读取指针
  uint16_t length;                   // 当前数据长度
  uint8_t overflow;                  // 溢出标志
  uint16_t frame_count;              // 解析成功的帧数
  uint16_t error_count;              // 解析错误的帧数
} RingBuffer_t;

// 外部变量声明
extern SBUS_Data_t SBUS_Data;
extern RingBuffer_t sbus_ring_buffer;

#ifdef __cplusplus
extern "C" {
#endif

// 环形缓冲区基础操作
void RingBuffer_Init(RingBuffer_t *rb);
uint8_t RingBuffer_IsEmpty(RingBuffer_t *rb);
uint8_t RingBuffer_IsFull(RingBuffer_t *rb);
uint16_t RingBuffer_GetLength(RingBuffer_t *rb);
uint8_t RingBuffer_WriteByte(RingBuffer_t *rb, uint8_t data);
uint8_t RingBuffer_ReadByte(RingBuffer_t *rb, uint8_t *data);
uint16_t RingBuffer_WriteData(RingBuffer_t *rb, uint8_t *data, uint16_t len);
uint16_t RingBuffer_ReadData(RingBuffer_t *rb, uint8_t *data, uint16_t len);
void RingBuffer_Clear(RingBuffer_t *rb);

// SBUS数据处理
void SBUS_RingBuffer_Init(void);
void SBUS_Data_Input(RingBuffer_t *rb, uint8_t *data, uint16_t len);
uint8_t SBUS_Frame_Parse_From_Buffer(RingBuffer_t *rb, SBUS_Data_t *sbus_data);
void SBUS_Debug_Print(RingBuffer_t *rb);

// 原有函数保持不变
void bsp_UART_Init(void);
void uart_print(const char *fmt, ...);

// 阻塞式打印：轮询 HAL_UART_Transmit，不依赖 DMA/RTOS，适合调度器启动前和临界区
void uart_print_blocking(const char *fmt, ...);

void sbus_frame_parse(SBUS_Data_t *sbus_data, uint8_t *rx_buff);

#ifdef __cplusplus
}
#endif

#endif