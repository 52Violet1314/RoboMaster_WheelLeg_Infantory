#include "Remote.h"
#include "main.h"
#include "Interrupt.h"
#include "string.h"

/* 本模块私有原始遥控快照；消费者只能通过队列取得完整副本。 */
static Remote_Data_t s_remote_data;
static Remote_RingBuffer_t s_remote_ring_buffer;

/* 所有断线路径共用的安全原始快照：摇杆、开关和旋钮均清零。
 * 不清 frame_count/error_count，方便故障后仍能查看累计诊断信息。 */
static void remote_apply_safe_state(uint8_t failsafe)
{
  memset((void *)s_remote_data.channel, 0, sizeof(s_remote_data.channel));
  memset((void *)s_remote_data.channel_raw, 0, sizeof(s_remote_data.channel_raw));
  s_remote_data.frame_valid = 0U;
  s_remote_data.failsafe = failsafe;
}

void Remote_Init(void)
{
  Remote_RingBuffer_Init(&s_remote_ring_buffer);
  memset((void *)&s_remote_data, 0, sizeof(s_remote_data));
  Remote_Uart5_Start_Receive();
}

void Remote_Sbus_Parse(void)
{
  uint8_t frame[REMOTE_SBUS_FRAME_SIZE];
  uint8_t value;
  uint8_t frame_found = 0U;
  uint16_t i;

  if (Remote_RingBuffer_Length(&s_remote_ring_buffer) < REMOTE_SBUS_FRAME_SIZE)
    return;

  /* 丢弃帧头前的噪声，保证 tail 对齐到完整 SBUS 帧。 */
  for (i = 0U; i <= Remote_RingBuffer_Length(&s_remote_ring_buffer) - REMOTE_SBUS_FRAME_SIZE; i++) {
    if (Remote_RingBuffer_Peek(&s_remote_ring_buffer, i, &value) && value == REMOTE_SBUS_HEAD &&
        Remote_RingBuffer_Peek(&s_remote_ring_buffer, i + REMOTE_SBUS_FRAME_SIZE - 1U, &value) &&
        value == REMOTE_SBUS_END) {
      frame_found = 1U;
      break;
    }
  }
  if (frame_found == 0U) {
    /* 最后 24 字节仍可能是下一帧的前半段，重同步时必须保留。 */
    const uint16_t discard_count = (uint16_t)(Remote_RingBuffer_Length(&s_remote_ring_buffer) -
                                               (REMOTE_SBUS_FRAME_SIZE - 1U));
    uint8_t discard[REMOTE_SBUS_FRAME_SIZE];
    uint16_t remaining = discard_count;
    while (remaining != 0U) {
      const uint16_t chunk = (remaining > sizeof(discard)) ? sizeof(discard) : remaining;
      (void)Remote_RingBuffer_Read(&s_remote_ring_buffer, discard, chunk);
      remaining = (uint16_t)(remaining - chunk);
    }
    s_remote_data.error_count++;
    return;
  }
  if (i != 0U) {
    uint8_t discard[REMOTE_SBUS_FRAME_SIZE];
    Remote_RingBuffer_Read(&s_remote_ring_buffer, discard, i);
  }
  if (Remote_RingBuffer_Read(&s_remote_ring_buffer, frame, REMOTE_SBUS_FRAME_SIZE) != REMOTE_SBUS_FRAME_SIZE)
    return;

  /* byte23 是标志位：bit2=frame lost，bit3=failsafe。 */
  s_remote_data.failsafe = (frame[23] & 0x0CU) ? 1U : 0U;
  if (s_remote_data.failsafe) {
    remote_apply_safe_state(1U);
    s_remote_data.error_count++;
    return;
  }
  s_remote_data.channel[0] = (int16_t)(((frame[1] | frame[2] << 8) & 0x07FFU) - 1024);
  s_remote_data.channel[1] = (int16_t)(((frame[2] >> 3 | frame[3] << 5) & 0x07FFU) - 1024);
  s_remote_data.channel[2] = (int16_t)(((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FFU) - 1024);
  s_remote_data.channel[3] = (int16_t)(((frame[5] >> 1 | frame[6] << 7) & 0x07FFU) - 1024);
  s_remote_data.channel[4] = (int16_t)(((frame[6] >> 4 | frame[7] << 4) & 0x07FFU) - 1024);
  s_remote_data.channel[5] = (int16_t)(((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FFU) - 1024);
  s_remote_data.channel[6] = (int16_t)(((frame[9] >> 2 | frame[10] << 6) & 0x07FFU) - 1024);
  s_remote_data.channel[7] = (int16_t)(((frame[10] >> 5 | frame[11] << 3) & 0x07FFU) - 1024);
  s_remote_data.channel[8] = (int16_t)(((frame[12] | frame[13] << 8) & 0x07FFU) - 1024);
  s_remote_data.channel[9] = (int16_t)(((frame[13] >> 3 | frame[14] << 5) & 0x07FFU) - 1024);
  s_remote_data.channel[10] = (int16_t)(((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FFU) - 1024);
  s_remote_data.channel[11] = (int16_t)(((frame[16] >> 1 | frame[17] << 7) & 0x07FFU) - 1024);
  s_remote_data.channel[12] = (int16_t)(((frame[17] >> 4 | frame[18] << 4) & 0x07FFU) - 1024);
  s_remote_data.channel[13] = (int16_t)(((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FFU) - 1024);
  s_remote_data.channel[14] = (int16_t)(((frame[20] >> 2 | frame[21] << 6) & 0x07FFU) - 1024);
  s_remote_data.channel[15] = (int16_t)(((frame[21] >> 5 | frame[22] << 3) & 0x07FFU) - 1024);
  /* 同时保留 SBUS 原始值，旋钮和底盘端映射使用 0~2047 的真实量程。 */
  for (i = 0U; i < REMOTE_CHANNEL_COUNT; i++) {
    s_remote_data.channel_raw[i] = (uint16_t)((int32_t)s_remote_data.channel[i] +
                                              REMOTE_SBUS_CENTER);
  }
  s_remote_data.frame_valid = 1U;
  s_remote_data.update_tick = HAL_GetTick();
  s_remote_data.frame_count++;
}

void Remote_Process(void)
{
  Remote_Data_t published;
  /* s_remote_data 仅由 InputTask 读写；ModeTask 只消费随后写入队列的完整副本。
   * 环形缓冲的任务读与 UART ISR 写已在 Ring_Buffer 内部分别保护，因此此处
   * 不需要 osKernelLock()，更不能用长时间 critical 包围整个 SBUS 解析循环。 */
  while (Remote_RingBuffer_Length(&s_remote_ring_buffer) >= REMOTE_SBUS_FRAME_SIZE) {
    Remote_Sbus_Parse();
  }
  memcpy(&published, &s_remote_data, sizeof(published));
  /* 已校验的原始 SBUS 数据只发送给 ModeTask，绝不绕过仲裁直达执行任务。 */
  if (Mode_Remote_QueueHandle != NULL) {
    Remote_Mode_Input_t input = { .data = published };
    if (osMessageQueuePut(Mode_Remote_QueueHandle, &input, 0U, 0U) != osOK) {
      Remote_Mode_Input_t discarded;
      (void)osMessageQueueGet(Mode_Remote_QueueHandle, &discarded, NULL, 0U);
      (void)osMessageQueuePut(Mode_Remote_QueueHandle, &input, 0U, 0U);
    }
  }
}

void Remote_RxDmaEvent(const uint8_t *data, uint16_t size)
{
  if (data == NULL || size == 0U) return;
  (void)Remote_RingBuffer_WriteFromISR(&s_remote_ring_buffer, data, size);
}
