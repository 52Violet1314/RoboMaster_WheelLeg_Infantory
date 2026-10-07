#ifndef USER_COMMUNICATION_VISION_H
#define USER_COMMUNICATION_VISION_H
#include <stdint.h>
#include "cmsis_os2.h"

/** 小电脑发给云台主控的 14 字节控制数据。
 * 线协议中的 yaw/pitch 必须为 IEEE754 little-endian rad，严禁在消费者重复换算。 */
typedef struct __attribute__((packed)) {
  uint8_t mode;
  float yaw_rad;
  float pitch_rad;
  uint8_t id;
  uint32_t update_tick;
  uint32_t frame_count;
} VisionToGimbal_t;

/** 云台主控发给小电脑的 26 字节状态数据；姿态字段均为 rad。 */
typedef struct __attribute__((packed)) {
  uint8_t mode;
  uint8_t own_color;
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
  float bullet_speed;
  uint32_t mcu_timestamp;
} GimbalToVision_t;

/** 视觉链路接收统计。 */
typedef struct {
  uint32_t rx_bytes;
  uint32_t frame_count;
  uint32_t crc_error_count;
  uint32_t header_error_count;
  uint32_t ring_overflow_count;
} VisionStats_t;

/** 视觉链路输入输出快照。 */
typedef struct {
  VisionToGimbal_t input;
  VisionStats_t stats;
} Vision_Data_t;

/** 初始化视觉链路软件状态并启动 USART10 DMA 接收。 */
void Vision_Init(void);
/** 任务调用入口：消费 USART10 环形缓冲并解析 VisionToGimbal。 */
void Vision_Process(void);
/** USART10 DMA 回调向视觉模块提交一段原始接收字节。 */
void Vision_RxDmaEvent(const uint8_t *data, uint16_t size);
/** 发送当前主控状态给视觉小电脑。 */
uint8_t Vision_SendGimbalToVision(const GimbalToVision_t *status);
extern osMessageQueueId_t Mode_Vision_QueueHandle;

#endif
