#include "Ring_Buffer.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/** 清零 UART5 环形缓冲及其读写状态。 */
void Remote_RingBuffer_Init(Remote_RingBuffer_t *rb)
{
  portENTER_CRITICAL();
  memset(rb, 0, sizeof(*rb));
  portEXIT_CRITICAL();
}

/** 原子读取 UART5 环形缓冲当前长度。 */
uint16_t Remote_RingBuffer_Length(Remote_RingBuffer_t *rb)
{
  uint16_t length;
  portENTER_CRITICAL();
  length = rb->length;
  portEXIT_CRITICAL();
  return length;
}

/** 环形写入的内部实现，供普通上下文和 ISR 包装函数共用。 */
static uint16_t write_bytes(Remote_RingBuffer_t *rb,
                            const uint8_t *data, uint16_t length)
{
  uint16_t written = 0U;
  while (written < length) {
    /* 实时控制输入只关心最新状态。满时先丢弃最老一个字节，
     * 解析器随后通过帧头/帧尾重新同步，绝不让旧数据堵住新数据。 */
    if (rb->length == REMOTE_SBUS_BUFFER_SIZE) {
      rb->tail = (uint16_t)((rb->tail + 1U) % REMOTE_SBUS_BUFFER_SIZE);
      rb->length--;
      rb->overflow_count++;
    }
    rb->data[rb->head] = data[written++];
    rb->head = (uint16_t)((rb->head + 1U) % REMOTE_SBUS_BUFFER_SIZE);
    rb->length++;
  }
  return written;
}

/** 在任务上下文写入 UART5 数据。 */
uint16_t Remote_RingBuffer_Write(Remote_RingBuffer_t *rb,
                                  const uint8_t *data, uint16_t length)
{
  uint16_t written;
  portENTER_CRITICAL();
  written = write_bytes(rb, data, length);
  portEXIT_CRITICAL();
  return written;
}

/** 在 UART5 ISR 上下文写入数据并保护共享索引。 */
uint16_t Remote_RingBuffer_WriteFromISR(Remote_RingBuffer_t *rb,
                                        const uint8_t *data, uint16_t length)
{
  UBaseType_t state = portSET_INTERRUPT_MASK_FROM_ISR();
  uint16_t written = write_bytes(rb, data, length);
  portCLEAR_INTERRUPT_MASK_FROM_ISR(state);
  return written;
}

/** 从 UART5 环形缓冲取出数据并推进读指针。 */
uint16_t Remote_RingBuffer_Read(Remote_RingBuffer_t *rb,
                                 uint8_t *data, uint16_t length)
{
  uint16_t read = 0U;
  portENTER_CRITICAL();
  while (read < length && rb->length != 0U) {
    data[read++] = rb->data[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) % REMOTE_SBUS_BUFFER_SIZE);
    rb->length--;
  }
  portEXIT_CRITICAL();
  return read;
}

/** 查看 UART5 环形缓冲指定偏移的字节，不改变读指针。 */
uint8_t Remote_RingBuffer_Peek(Remote_RingBuffer_t *rb,
                               uint16_t offset, uint8_t *value)
{
  uint8_t ok = 0U;
  portENTER_CRITICAL();
  if (offset < rb->length) {
    *value = rb->data[(rb->tail + offset) % REMOTE_SBUS_BUFFER_SIZE];
    ok = 1U;
  }
  portEXIT_CRITICAL();
  return ok;
}

/** 清空 UART5 环形缓冲。 */
void Remote_RingBuffer_Clear(Remote_RingBuffer_t *rb)
{
  portENTER_CRITICAL();
  rb->head = rb->tail = rb->length = 0U;
  portEXIT_CRITICAL();
}

/** 初始化 UART7 图传环形缓冲。 */
void Referee_RingBuffer_Init(Referee_RingBuffer_t *rb)
{
  portENTER_CRITICAL();
  memset(rb, 0, sizeof(*rb));
  portEXIT_CRITICAL();
}

/** 在 UART7 DMA ISR 中写入字节；满时丢最旧字节，保留最新图传状态。 */
uint16_t Referee_RingBuffer_WriteFromISR(Referee_RingBuffer_t *rb, const uint8_t *data, uint16_t length)
{
  uint16_t written = 0U;
  UBaseType_t state = portSET_INTERRUPT_MASK_FROM_ISR();
  while (written < length) {
    if (rb->length == REFEREE_RING_BUFFER_SIZE) {
      rb->tail = (uint16_t)((rb->tail + 1U) % REFEREE_RING_BUFFER_SIZE);
      rb->length--;
      rb->overflow_count++;
    }
    rb->data[rb->head] = data[written++];
    rb->head = (uint16_t)((rb->head + 1U) % REFEREE_RING_BUFFER_SIZE);
    rb->length++;
  }
  portCLEAR_INTERRUPT_MASK_FROM_ISR(state);
  return written;
}

/** 从任务上下文读取一个 UART7 图传字节。 */
uint8_t Referee_RingBuffer_ReadByte(Referee_RingBuffer_t *rb, uint8_t *value)
{
  uint8_t ok = 0U;
  portENTER_CRITICAL();
  if (rb->length != 0U) {
    *value = rb->data[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) % REFEREE_RING_BUFFER_SIZE);
    rb->length--;
    ok = 1U;
  }
  portEXIT_CRITICAL();
  return ok;
}

/** 初始化 USART10 视觉环形缓冲。 */
void Vision_RingBuffer_Init(Vision_RingBuffer_t *rb)
{
  portENTER_CRITICAL(); memset(rb, 0, sizeof(*rb)); portEXIT_CRITICAL();
}

/** 在 USART10 DMA 回调中写入视觉数据，满时丢最旧字节以保留最新目标。 */
uint16_t Vision_RingBuffer_WriteFromISR(Vision_RingBuffer_t *rb, const uint8_t *data, uint16_t length)
{
  uint16_t written = 0U;
  UBaseType_t state = portSET_INTERRUPT_MASK_FROM_ISR();
  while (written < length) {
    if (rb->length == VISION_RING_BUFFER_SIZE) {
      rb->tail = (uint16_t)((rb->tail + 1U) % VISION_RING_BUFFER_SIZE);
      rb->length--;
      rb->overflow_count++;
    }
    rb->data[rb->head] = data[written++];
    rb->head = (uint16_t)((rb->head + 1U) % VISION_RING_BUFFER_SIZE);
    rb->length++;
  }
  portCLEAR_INTERRUPT_MASK_FROM_ISR(state);
  return written;
}

/** 从视觉环形缓冲读取一个字节。 */
uint8_t Vision_RingBuffer_ReadByte(Vision_RingBuffer_t *rb, uint8_t *value)
{
  uint8_t ok = 0U;
  portENTER_CRITICAL();
  if (rb->length != 0U) {
    *value = rb->data[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) % VISION_RING_BUFFER_SIZE);
    rb->length--;
    ok = 1U;
  }
  portEXIT_CRITICAL();
  return ok;
}
