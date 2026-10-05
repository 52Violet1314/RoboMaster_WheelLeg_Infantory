#include "Vision.h"
#include "Ring_Buffer.h"
#include "Interrupt.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "core_cm7.h"
#include <string.h>

#define VISION_RX_HEAD0 0x51U
#define VISION_RX_HEAD1 0x59U
#define VISION_TX_HEAD0 0x47U
#define VISION_TX_HEAD1 0x44U
#define VISION_RX_FRAME_LEN 14U
#define VISION_TX_FRAME_LEN 26U

static Vision_RingBuffer_t s_vision_ring;
static uint8_t s_vision_frame[VISION_RX_FRAME_LEN];
static uint16_t s_vision_frame_length;
static uint8_t s_vision_initialized;
static Vision_Data_t s_vision_data;
static uint8_t s_vision_tx[VISION_TX_FRAME_LEN] __attribute__((aligned(32), section(".RAM_D2")));

/** 视觉协议 CRC16；实际多项式/初值应与视觉端 tools/crc.hpp 一致。 */
static uint16_t vision_crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  while (length-- != 0U) {
    crc ^= *data++;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
      crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0x8408U) : (uint16_t)(crc >> 1U);
  }
  return crc;
}

/** 将 USART10 DMA 回调收到的原始字节写入视觉环形缓冲。 */
void Vision_RxDmaEvent(const uint8_t *data, uint16_t size)
{
  if (data == NULL || size == 0U) return;
  (void)Vision_RingBuffer_WriteFromISR(&s_vision_ring, data, size);
  s_vision_data.stats.rx_bytes += size;
  s_vision_data.stats.ring_overflow_count = s_vision_ring.overflow_count;
}

/** 解析并保存一帧 CRC 正确的 VisionToGimbal 数据。 */
static void vision_dispatch(const uint8_t *frame)
{
  uint16_t received = (uint16_t)(frame[12] | ((uint16_t)frame[13] << 8U));
  if (vision_crc16(frame, 12U) != received) { s_vision_data.stats.crc_error_count++; return; }
  s_vision_data.input.mode = frame[2];
  memcpy(&s_vision_data.input.yaw_rad, &frame[3], sizeof(float));
  memcpy(&s_vision_data.input.pitch_rad, &frame[7], sizeof(float));
  s_vision_data.input.id = frame[11];
  s_vision_data.input.update_tick = HAL_GetTick();
  s_vision_data.input.frame_count++;
  s_vision_data.stats.frame_count++;
  if (Mode_Vision_QueueHandle != NULL) {
    VisionToGimbal_t discarded;
    if (osMessageQueuePut(Mode_Vision_QueueHandle, &s_vision_data.input, 0U, 0U) != osOK) {
      (void)osMessageQueueGet(Mode_Vision_QueueHandle, &discarded, NULL, 0U);
      (void)osMessageQueuePut(Mode_Vision_QueueHandle, &s_vision_data.input, 0U, 0U);
    }
  }
}

/** 初始化视觉状态、组帧缓存和 USART10 DMA 接收。 */
void Vision_Init(void)
{
  Vision_RingBuffer_Init(&s_vision_ring);
  memset(s_vision_frame, 0, sizeof(s_vision_frame));
  memset(&s_vision_data, 0, sizeof(s_vision_data));
  s_vision_frame_length = 0U; s_vision_initialized = 1U;
  Vision_Usart10_Start_Receive();
}

/** 任务入口：逐字节找 0x51 0x59，收满 14 字节后校验 CRC16。 */
void Vision_Process(void)
{
  uint8_t byte;
  if (s_vision_initialized == 0U) { Vision_Init(); return; }
  while (Vision_RingBuffer_ReadByte(&s_vision_ring, &byte) != 0U) {
    if (s_vision_frame_length == 0U) { if (byte == VISION_RX_HEAD0) s_vision_frame[s_vision_frame_length++] = byte; continue; }
    if (s_vision_frame_length == 1U && byte != VISION_RX_HEAD1) {
      s_vision_data.stats.header_error_count++;
      s_vision_frame_length = 0U;
      continue;
    }
    s_vision_frame[s_vision_frame_length++] = byte;
    if (s_vision_frame_length == VISION_RX_FRAME_LEN) { vision_dispatch(s_vision_frame); s_vision_frame_length = 0U; }
  }
}

/** 组装并通过 USART10 DMA 发送 26 字节 GimbalToVision 状态帧。 */
uint8_t Vision_SendGimbalToVision(const GimbalToVision_t *status)
{
  if (status == NULL) return 0U;
  s_vision_tx[0] = VISION_TX_HEAD0; s_vision_tx[1] = VISION_TX_HEAD1;
  s_vision_tx[2] = status->mode;
  s_vision_tx[3] = status->own_color;
  memcpy(&s_vision_tx[4], &status->roll_rad, sizeof(float));
  memcpy(&s_vision_tx[8], &status->pitch_rad, sizeof(float));
  memcpy(&s_vision_tx[12], &status->yaw_rad, sizeof(float));
  memcpy(&s_vision_tx[16], &status->bullet_speed, sizeof(float));
  memcpy(&s_vision_tx[20], &status->mcu_timestamp, sizeof(uint32_t));
  uint16_t crc = vision_crc16(s_vision_tx, 24U);
  s_vision_tx[24] = (uint8_t)crc; s_vision_tx[25] = (uint8_t)(crc >> 8U);
  SCB_CleanDCache_by_Addr((uint32_t *)s_vision_tx, sizeof(s_vision_tx));
  return (HAL_UART_Transmit_DMA(&huart10, s_vision_tx, VISION_TX_FRAME_LEN) == HAL_OK) ? 1U : 0U;
}
