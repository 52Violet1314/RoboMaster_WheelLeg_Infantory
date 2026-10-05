#ifndef USER_COMMUNICATION_REMOTE_H
#define USER_COMMUNICATION_REMOTE_H
#include <stdint.h>
#include "cmsis_os2.h"
#include "Ring_Buffer.h"

#define REMOTE_SBUS_HEAD 0x0FU
#define REMOTE_SBUS_END  0x00U
#define REMOTE_CHANNEL_COUNT 16U
#define REMOTE_SBUS_CENTER 1024
#define REMOTE_SBUS_MAX_VALUE 2047
/* 当前发射机确认按 1 kHz 输出完整遥控状态。该宏只描述生产端数据节拍；
 * 遥控是否掉线由 ModeTask 按 MODE_REMOTE_INPUT_TIMEOUT_MS 判断。
 * 注意：SBUS 为 25 字节、8E2，每帧在线路上占 300 bit。UART5 必须配置为与
 * 发射机一致的 100 kbit/s、8E2；在此物理速率下完整帧最高约 333 Hz，不能以
 * 1 kHz 连续传输完整 SBUS 帧。 */
#define REMOTE_EXPECTED_UPDATE_HZ        (1000U)
#define REMOTE_EXPECTED_UPDATE_PERIOD_MS (1000U / REMOTE_EXPECTED_UPDATE_HZ)

/*
 * 通道/遥控器位置（按当前 Infantory 的 Mode-2 通道配置约定）：
 * CH1=右摇杆左右（底盘左右横移），CH2=右摇杆上下（底盘前后），
 * CH3=左摇杆上下（云台 pitch），CH4=左摇杆左右（云台 yaw）。
 * 左侧四个拨杆从左到右：SwA=CH7 摩擦轮，SwB=CH8 视觉自瞄，
 * SwC=CH9 小陀螺模式（三档，仅最大档开启），SwD=CH10 总使能与腿模式合并。
 * VrA=CH5 底盘小陀螺速度，VrB=CH6 腿长，CH11~CH16 当前未使用。
 * SBUS 本身只提供通道编号，若发射机重映射，物理位置需按发射机配置校准。
 */

typedef struct {
  int16_t channel[16];
  uint16_t channel_raw[16];
  uint8_t frame_valid;
  uint8_t failsafe;
  uint32_t update_tick; /* last valid SBUS frame arrival, HAL ms base */
  uint32_t frame_count;
  uint32_t error_count;
} Remote_Data_t;

/* 通信模块只发布已完整解出的原始 SBUS 数值；所有死区、物理单位映射、
 * 拨杆含义和模式仲裁只能由 ModeTask 完成。 */
typedef struct {
  Remote_Data_t data;
} Remote_Mode_Input_t;
_Static_assert(sizeof(Remote_Mode_Input_t) == 80U,
               "Mode_Remote_Queue element size must remain 80 bytes");

void Remote_Init(void);
void Remote_Process(void);
void Remote_Sbus_Parse(void);
/** UART5 DMA 中断入口；只将原始字节追加到本模块私有缓冲。 */
void Remote_RxDmaEvent(const uint8_t *data, uint16_t size);
extern osMessageQueueId_t Mode_Remote_QueueHandle;
#endif
