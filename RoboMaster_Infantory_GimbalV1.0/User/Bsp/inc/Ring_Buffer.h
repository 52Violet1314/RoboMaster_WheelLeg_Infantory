#ifndef USER_BSP_RING_BUFFER_H
#define USER_BSP_RING_BUFFER_H
#include <stdint.h>

#define REMOTE_SBUS_FRAME_SIZE 25U
#define REMOTE_SBUS_BUFFER_SIZE 250U
#define REFEREE_RING_BUFFER_SIZE 2048U
#define VISION_RING_BUFFER_SIZE 512U

/** UART5 SBUS 遥控器字节环形缓冲。 */
typedef struct {
  uint8_t data[REMOTE_SBUS_BUFFER_SIZE];
  uint16_t head;
  uint16_t tail;
  uint16_t length;
  uint32_t overflow_count;
} Remote_RingBuffer_t;

/** 初始化 UART5 遥控器环形缓冲。 */
void Remote_RingBuffer_Init(Remote_RingBuffer_t *rb);
/** 返回 UART5 缓冲中当前待读字节数。 */
uint16_t Remote_RingBuffer_Length(Remote_RingBuffer_t *rb);
/** 在任务上下文写入 UART5 遥控器数据。 */
uint16_t Remote_RingBuffer_Write(Remote_RingBuffer_t *rb,
                                  const uint8_t *data, uint16_t length);
/** 在中断上下文写入 UART5 遥控器数据。 */
uint16_t Remote_RingBuffer_WriteFromISR(Remote_RingBuffer_t *rb,
                                        const uint8_t *data, uint16_t length);
/** 从 UART5 缓冲读取指定数量字节。 */
uint16_t Remote_RingBuffer_Read(Remote_RingBuffer_t *rb,
                                 uint8_t *data, uint16_t length);
/** 不移除数据，查看 UART5 缓冲中指定偏移处的字节。 */
uint8_t Remote_RingBuffer_Peek(Remote_RingBuffer_t *rb,
                               uint16_t offset, uint8_t *value);
/** 清空 UART5 遥控器环形缓冲。 */
void Remote_RingBuffer_Clear(Remote_RingBuffer_t *rb);

/** UART7 图传协议字节环形缓冲，由 DMA 回调写入、Refree_Process 读取。 */
typedef struct {
  uint8_t data[REFEREE_RING_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint16_t length;
  volatile uint32_t overflow_count;
} Referee_RingBuffer_t;
/** 初始化 UART7 图传环形缓冲。 */
void Referee_RingBuffer_Init(Referee_RingBuffer_t *rb);
/** 在 UART7 DMA 中断回调中追加字节，返回实际写入数。 */
uint16_t Referee_RingBuffer_WriteFromISR(Referee_RingBuffer_t *rb, const uint8_t *data, uint16_t length);
/** 在任务上下文读取一个图传字节，成功返回 1。 */
uint8_t Referee_RingBuffer_ReadByte(Referee_RingBuffer_t *rb, uint8_t *value);

/** USART10 视觉链路字节环形缓冲。 */
typedef struct {
  uint8_t data[VISION_RING_BUFFER_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint16_t length;
  volatile uint32_t overflow_count;
} Vision_RingBuffer_t;
/** 初始化视觉链路环形缓冲。 */
void Vision_RingBuffer_Init(Vision_RingBuffer_t *rb);
/** 在视觉串口 ISR 中写入 DMA 收到的字节。 */
uint16_t Vision_RingBuffer_WriteFromISR(Vision_RingBuffer_t *rb, const uint8_t *data, uint16_t length);
/** 从任务上下文读取一个视觉链路字节。 */
uint8_t Vision_RingBuffer_ReadByte(Vision_RingBuffer_t *rb, uint8_t *value);
#endif
