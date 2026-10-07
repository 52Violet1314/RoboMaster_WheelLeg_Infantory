#include "Refree.h"
#include "Interrupt.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include <string.h>

#define REFEREE_SOF                    0xA5U /* 裁判协议固定帧头字节。 */
#define REFEREE_FRAME_HEADER_LEN        5U /* 帧头长度：SOF、data_length、seq、CRC8。 */
#define REFEREE_FRAME_OVERHEAD_LEN      9U /* 除 data 外的固定长度：5B 帧头 + 2B cmd_id + 2B CRC16。 */
#define REFEREE_MAX_DATA_LEN            300U /* 允许接收的最大 data 字段长度。 */
#define REFEREE_MAX_FRAME_LEN           (REFEREE_MAX_DATA_LEN + REFEREE_FRAME_OVERHEAD_LEN) /* 组帧缓存最大容量。 */

/* UART7 ISR 是唯一生产者，InputTask 中的 Refree_Process 是唯一消费者。 */
static Referee_RingBuffer_t s_vtm_ring; /* UART7 DMA 回调写入的图传字节环形缓冲。 */
static uint8_t s_frame_buffer[REFEREE_MAX_FRAME_LEN]; /* 当前正在组装的一帧协议数据。 */
static uint16_t s_frame_length; /* 当前已写入 s_frame_buffer 的字节数。 */
static uint8_t s_initialized; /* Refree_Init 是否已经完成：0 未初始化，1 已初始化。 */
static Referee_VtmInput_t s_vtm_input; /* 已解析的键鼠、自定义数据和诊断统计快照。 */
extern osMessageQueueId_t Referee_Data_QueueHandle;

/** @brief 返回软件环形缓冲中的下一个索引。 */
/**
 * @brief 从 ISR 生产、InputTask 消费的软件环形缓冲取出一个字节。
 * @retval 1 成功，0 当前无待处理字节。
 */
static uint8_t ring_pop(uint8_t *value)
{
  return Referee_RingBuffer_ReadByte(&s_vtm_ring, value);
}

/* DJI 裁判协议 CRC8：初值 0xFF，反射多项式 0x8C。 */
static uint8_t referee_crc8(const uint8_t *data, uint16_t length)
{
  uint8_t crc = 0xFFU;
  while (length-- != 0U) {
    crc ^= *data++;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
      crc = (crc & 1U) ? (uint8_t)((crc >> 1U) ^ 0x8CU) : (uint8_t)(crc >> 1U);
  }
  return crc;
}

/* DJI 裁判协议 CRC16：初值 0xFFFF，反射多项式 0x8408。 */
static uint16_t referee_crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  while (length-- != 0U) {
    crc ^= *data++;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
      crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0x8408U) : (uint16_t)(crc >> 1U);
  }
  return crc;
}

/**
 * @brief 用一帧有效图传控制载荷原子语义地更新对应的“最新状态”。
 * @note 本函数在 InputTask 上下文运行；载荷业务含义由控制层解释。
 */
static void update_custom_rx(Referee_VtmCustomRx_t *target,
                             const uint8_t *data,
                             uint16_t length,
                             uint8_t sequence)
{
  if (length > REFEREE_VTM_CUSTOM_DATA_MAX_LEN) return;
  memset(target->data, 0, sizeof(target->data));
  memcpy(target->data, data, length);
  target->length = (uint8_t)length;
  target->sequence = sequence;
  target->update_tick = HAL_GetTick();
  target->frame_count++;
}

static uint8_t key_down(uint16_t keys, uint16_t key_mask)
{
  return ((keys & key_mask) != 0U) ? 1U : 0U;
}

static void publish_custom_event(const uint8_t *data, uint16_t length,
                                 uint8_t sequence)
{
  Referee_Event_t event;
  if (length > sizeof(event.data) || Referee_Data_QueueHandle == NULL) return;
  memset(&event, 0, sizeof(event));
  event.type = 1U;
  event.sequence = sequence;
  event.length = (uint8_t)length;
  event.timestamp_ms = HAL_GetTick();
  memcpy(event.data, data, length);
  /* 必须保持事件顺序：队列满时不能为了新事件丢弃旧事件。 */
  if (osMessageQueuePut(Referee_Data_QueueHandle, &event, 0U, 0U) != osOK)
    s_vtm_input.stats.event_queue_overflow_count++;
}

/** 保存 V1.7.0 的 12 字节 0x0304 键鼠原始语义。
 * 该函数逐项保留 mouse_x/y/z、左右键和整个 keyboard_value 位图；业务映射
 * 只能在 ModeTask 进行。 */
static void update_keyboard_mouse(const uint8_t *data)
{
  Referee_VtmKeyboardMouse_t *k = &s_vtm_input.keyboard_mouse;
  k->mouse_x = (int16_t)(data[0] | ((uint16_t)data[1] << 8U));
  k->mouse_y = (int16_t)(data[2] | ((uint16_t)data[3] << 8U));
  k->mouse_z = (int16_t)(data[4] | ((uint16_t)data[5] << 8U));
  k->left_button_down = data[6]; k->right_button_down = data[7];
  k->keyboard_value = (uint16_t)(data[8] | ((uint16_t)data[9] << 8U));
  k->reserved = (uint16_t)(data[10] | ((uint16_t)data[11] << 8U));
  k->key_w = key_down(k->keyboard_value, REFEREE_KEY_W);
  k->key_s = key_down(k->keyboard_value, REFEREE_KEY_S);
  k->key_a = key_down(k->keyboard_value, REFEREE_KEY_A);
  k->key_d = key_down(k->keyboard_value, REFEREE_KEY_D);
  k->key_shift = key_down(k->keyboard_value, REFEREE_KEY_SHIFT);
  k->key_ctrl = key_down(k->keyboard_value, REFEREE_KEY_CTRL);
  k->key_q = key_down(k->keyboard_value, REFEREE_KEY_Q);
  k->key_e = key_down(k->keyboard_value, REFEREE_KEY_E);
  k->key_r = key_down(k->keyboard_value, REFEREE_KEY_R);
  k->key_f = key_down(k->keyboard_value, REFEREE_KEY_F);
  k->key_g = key_down(k->keyboard_value, REFEREE_KEY_G);
  k->key_z = key_down(k->keyboard_value, REFEREE_KEY_Z);
  k->key_x = key_down(k->keyboard_value, REFEREE_KEY_X);
  k->key_c = key_down(k->keyboard_value, REFEREE_KEY_C);
  k->key_v = key_down(k->keyboard_value, REFEREE_KEY_V);
  k->key_b = key_down(k->keyboard_value, REFEREE_KEY_B);
  k->update_tick = HAL_GetTick(); k->frame_count++;
}

/**
 * @brief 校验完整裁判帧的 CRC16，并按 cmd_id 更新轮腿步兵所需输入。
 * @note 处理 V1.7.0 图传入站命令：0x0304 为官方键鼠，0x0302 为可选自定义载荷。
 * 二者分别保存，不共享字段、不互相覆盖。
 */
static void dispatch_frame(const uint8_t *frame, uint16_t frame_length)
{
  const uint16_t data_length = (uint16_t)(frame[1] | ((uint16_t)frame[2] << 8U));
  const uint16_t command_id = (uint16_t)(frame[5] | ((uint16_t)frame[6] << 8U));
  const uint8_t *data = &frame[7];
  const uint16_t received_crc = (uint16_t)(frame[frame_length - 2U]
                                           | ((uint16_t)frame[frame_length - 1U] << 8U));

  if (referee_crc16(frame, (uint16_t)(frame_length - 2U)) != received_crc) {
    s_vtm_input.stats.crc16_error_count++;
    return;
  }

  s_vtm_input.stats.frame_count++;
  switch (command_id) {
  case REFEREE_VTM_CMD_CUSTOM_CONTROLLER:
    if (data_length != REFEREE_VTM_CUSTOM_DATA_MAX_LEN) {
      s_vtm_input.stats.length_error_count++;
      return;
    }
    update_custom_rx(&s_vtm_input.custom_controller, data, data_length, frame[3]);
    publish_custom_event(data, data_length, frame[3]);
    break;
  case REFEREE_VTM_CMD_KEYBOARD_MOUSE:
    if (data_length != REFEREE_VTM_KEYBOARD_MOUSE_LEN) {
      s_vtm_input.stats.length_error_count++;
      return;
    }
    update_keyboard_mouse(data);
    break;
  default:
    /* 轮腿步兵暂不使用其他兵种专用图传命令，保留统计以便抓包诊断。 */
    s_vtm_input.stats.unsupported_cmd_count++;
    break;
  }
  if (Mode_Referee_QueueHandle != NULL) {
    Referee_VtmInput_t discarded;
    if (osMessageQueuePut(Mode_Referee_QueueHandle, &s_vtm_input, 0U, 0U) != osOK) {
      (void)osMessageQueueGet(Mode_Referee_QueueHandle, &discarded, NULL, 0U);
      (void)osMessageQueuePut(Mode_Referee_QueueHandle, &s_vtm_input, 0U, 0U);
    }
  }
}

/**
 * @brief 初始化图传的软件环形缓冲、组帧状态和统计量，随后挂起 UART7 DMA 接收。
 * @note 必须在任务上下文调用，不能从 UART ISR 调用。
 */
void Refree_Init(void)
{
  portENTER_CRITICAL();
  Referee_RingBuffer_Init(&s_vtm_ring);
  memset(s_frame_buffer, 0, sizeof(s_frame_buffer));
  memset(&s_vtm_input, 0, sizeof(s_vtm_input));
  s_frame_length = 0U;
  s_initialized = 1U;
  portEXIT_CRITICAL();

  Refree_Uart7_Start_Receive();
}

/**
 * @brief 将 UART7 DMA 已接收字节复制到软件环形缓冲。
 * @note 运行于 UART 中断回调；UART7 是唯一生产者，Refree_Process 是唯一消费者。
 */
void Refree_UART7_DmaRxEvent(const uint8_t *data, uint16_t size)
{
  if (data == NULL || size == 0U) return;

  s_vtm_input.stats.dma_bytes += Referee_RingBuffer_WriteFromISR(&s_vtm_ring, data, size);
  s_vtm_input.stats.ring_overflow_count = s_vtm_ring.overflow_count;
}

/**
 * @brief 解析所有待处理 UART7 字节流。
 * @details 处理粘包、半包和错误重同步：查找 SOF，校验 CRC8，检查 data_length，
 *          收满 data_length + 9 字节后校验 CRC16 并分发 cmd_id。
 */
void Refree_Process(void)
{
  uint8_t byte;

  /* 第一次执行时初始化图传环形缓冲、组帧缓存和 UART7 DMA。 */
  if (s_initialized == 0U) {
    Refree_Init();
    return;
  }

  /* 一次处理环形缓冲中当前已有的全部字节，直到缓冲区为空。 */
  while (ring_pop(&byte) != 0U) {
    /* 尚未开始组帧：丢弃无关字节，只等待协议规定的 0xA5 帧头。 */
    if (s_frame_length == 0U) {
      if (byte == REFEREE_SOF) s_frame_buffer[s_frame_length++] = byte;
      continue;
    }

    /* 防止异常长度导致组帧缓存越界；丢弃当前帧并重新寻找帧头。 */
    if (s_frame_length >= sizeof(s_frame_buffer)) {
      s_vtm_input.stats.length_error_count++;
      s_frame_length = 0U;
      if (byte == REFEREE_SOF) s_frame_buffer[s_frame_length++] = byte;
      continue;
    }

    /* 将当前字节追加到正在组装的协议帧。 */
    s_frame_buffer[s_frame_length++] = byte;

    /* 收齐 5 字节帧头后，校验 SOF、长度、seq 对应的 CRC8。 */
    if (s_frame_length == REFEREE_FRAME_HEADER_LEN
        && referee_crc8(s_frame_buffer, 4U) != s_frame_buffer[4]) {
      s_vtm_input.stats.crc8_error_count++;
      s_frame_length = 0U;
      continue;
    }

    if (s_frame_length >= REFEREE_FRAME_HEADER_LEN) {
      /* 从帧头读取 data 长度；协议长度字段为小端格式。 */
      const uint16_t data_length = (uint16_t)(s_frame_buffer[1]
                                              | ((uint16_t)s_frame_buffer[2] << 8U));
      /* 完整帧长度 = 5 字节帧头 + 2 字节 cmd_id + data + 2 字节 CRC16。 */
      const uint16_t expected_length = (uint16_t)(data_length + REFEREE_FRAME_OVERHEAD_LEN);

      /* 超过接收缓存允许的最大 data 长度，当前帧作废并重新同步。 */
      if (data_length > REFEREE_MAX_DATA_LEN) {
        s_vtm_input.stats.length_error_count++;
        s_frame_length = 0U;
        continue;
      }

      /* 当前帧还没有收齐：保留已有字节，等待下一次 DMA 回调继续补齐。 */
      if (s_frame_length < expected_length) continue;

      /* 收齐完整帧后，由 dispatch_frame() 校验 CRC16 并按 cmd_id 分类。 */
      if (s_frame_length == expected_length) {
        dispatch_frame(s_frame_buffer, expected_length);
        /* 当前帧处理结束，下一字节从新的协议帧开始。 */
        s_frame_length = 0U;
      }
    }
  }
}

osStatus_t Refree_GetEvent(Referee_Event_t *out, uint32_t timeout_ms)
{
  if (out == NULL || Referee_Data_QueueHandle == NULL) return osErrorParameter;
  return osMessageQueueGet(Referee_Data_QueueHandle, out, NULL, timeout_ms);
}
