#include "Imu.h"
#include "cmsis_os2.h"
#include "main.h"
#include "tim.h"
#include <string.h>
#include <math.h>

/*
| 字节范围 | 长度 | 内容 | 格式 |
|---|---:|---|---|
| `0` | 1 | 帧头 `0x5A` | `uint8_t` |
| `1` | 1 | 帧头 `0xA5` | `uint8_t` |
| `2` | 1 | Payload 长度低字节 | `uint8_t` |
| `3` | 1 | Payload 长度高字节 | `uint8_t` |
| `4` | 1 | CRC 低字节 | `uint8_t` |
| `5` | 1 | CRC 高字节 | `uint8_t` |
| `6` | 1 | 数据标签 `0x91` | `uint8_t` |
| `7-8` | 2 | `main_status` | `uint16_t` |
| `9` | 1 | 温度 | `int8_t` |
| `10-13` | 4 | 气压 | `float` |
| `14-17` | 4 | 传感器系统时间 | `uint32_t` |
| `18-29` | 12 | 三轴加速度 `acc[3]` | 3 个 `float` |
| `30-41` | 12 | 三轴角速度 `gyr[3]` | 3 个 `float` |
| `42-53` | 12 | 三轴磁场 `mag[3]` | 3 个 `float` |
| `54-57` | 4 | roll | `float` |
| `58-61` | 4 | pitch | `float` |
| `62-65` | 4 | yaw | `float` |
| `66-81` | 16 | 四元数 `quat[4]` | 4 个 `float` |
*/ 

/* HI91 字节流解析状态，不能在每个 DMA 数据块后清零。 */
static Imu_Parser_t g_parser;
static volatile uint32_t g_last_sample_timestamp_us;
static volatile uint32_t g_valid_sample_count;
extern osThreadId_t CanTaskHandle;
extern osThreadId_t Gimbal_TaskHandle;
extern osMessageQueueId_t IMU_QueueHandle;
extern osMessageQueueId_t Gimbal_Imu_QueueHandle;

/**
 * @brief 把 pitch 角+角速度塞入云台专用队列, 满则丢旧覆盖最新.
 */
static void imu_dispatch_gimbal(const Imu_Data_t *sample)
{
  Imu_Pitch_Packet_t pkt = { .pitch_rad = sample->pitch_rad,
                              .gyro_pitch_rad_s = sample->gyro_rad_s[0],
                              .timestamp_us = sample->timestamp_us };
  if (osMessageQueuePut(Gimbal_Imu_QueueHandle, &pkt, 0U, 0U) != osOK) {
    Imu_Pitch_Packet_t dummy;
    (void)osMessageQueueGet(Gimbal_Imu_QueueHandle, &dummy, NULL, 0U);
    (void)osMessageQueuePut(Gimbal_Imu_QueueHandle, &pkt, 0U, 0U);
  }

}

/**
 * @brief 计算 CRC-16/CCITT 校验值。
 * @param data   待校验数据首地址。
 * @param length 参与校验的字节数。
 * @return CRC-16 结果。
 */
static uint16_t crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0U;
  for (uint16_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8U; ++bit)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
  }
  return crc;
}

int Imu_Check_Crc(const uint8_t *frame, uint16_t total_length)
{
  if ((frame == NULL) || (total_length < 6U)) return 0;
  uint16_t payload = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
  if ((uint32_t)payload + 6U != total_length) return 0;
  uint16_t expected = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
  uint16_t crc = crc16(frame, 4U);
  /* crc16() 采用固定初值，因此这里按协议逐段连续计算。 */
  for (uint16_t i = 0; i < payload; ++i) {
    crc ^= (uint16_t)frame[6 + i] << 8;
    for (uint8_t bit = 0; bit < 8U; ++bit)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
  }
  return crc == expected;
}

int Imu_Parse_Byte(Imu_Parser_t *p, uint8_t byte, Imu_Data_t *out)
{
  if (!p || !out) return 0;
  if (p->count == 0U) { if (byte != 0x5AU) return 0; p->frame[p->count++] = byte; return 0; }
  if (p->count == 1U && byte != 0xA5U) { p->count = 0U; return 0; }
  if (p->count >= sizeof(p->frame)) { p->count = 0U; return 0; }
  p->frame[p->count++] = byte;
  if (p->count == 6U) {
    p->payload_length = (uint16_t)p->frame[2] | ((uint16_t)p->frame[3] << 8);
    /* HI91 官方输出为固定 76 字节 payload，总帧长固定为 82 字节。 */
    if (p->payload_length != 76U) {
      p->count = 0U;
      return 0;
    }
  }
  if ((p->count >= 6U) && (p->count == p->payload_length + 6U)) {
    /* 统一调用校验函数，只有长度正确且 CRC 通过才继续解析。 */
    if (Imu_Check_Crc(p->frame, (uint16_t)(p->payload_length + 6U))) {
      const uint8_t *q = &p->frame[6];
      /* 0x91 标签后按官方 HI91 布局读取姿态、角速度和加速度。 */
      if (q[0] == 0x91U) {
        memcpy(&out->accel[0], q + 12, 12);
        memcpy(&out->gyro_rad_s[0], q + 24, 12);
        /* HI91 布局：acc 12~23，gyr 24~35，roll/pitch/yaw 48/52/56。 */
        memcpy(&out->roll_rad, q + 48, 4);
        memcpy(&out->pitch_rad, q + 52, 4);
        memcpy(&out->yaw_rad, q + 56, 4);
        /* HI91 线协议原始值为 deg/deg/s；这里只做单位换算为 SI rad/rad/s。
         * 不交换轴、不反号、不用欧拉角二次计算角速度：roll/pitch/yaw 与 gyr[3]
         * 均保持模块配置后的原生输出。当前安装依赖 HI91 已保存 URFR134：
         * X 左、Y 后、Z 上；若模块配置被改，必须先恢复模块配置，不能在此处
         * 叠加软件坐标变换。 */
        const float deg_to_rad = 0.01745329251994329577f;
        out->roll_rad *= deg_to_rad;
        out->pitch_rad *= deg_to_rad;
        out->yaw_rad *= deg_to_rad;
        out->gyro_rad_s[0] *= deg_to_rad;
        out->gyro_rad_s[1] *= deg_to_rad;
        out->gyro_rad_s[2] *= deg_to_rad;
        p->count = 0U;
        return 1;
      }
    }
    p->count = 0U;
  }
  return 0;
}

void Imu_Coordinate_Process(Imu_Data_t *d)
{
  /* 安装坐标：X 向左、Y 向后、Z 向上；设备应配置 URFR 134。 */
  float sp = sinf(d->pitch_rad), cp = cosf(d->pitch_rad);
  float sr = sinf(d->roll_rad), cr = cosf(d->roll_rad);
  float ax=d->accel[0], ay=d->accel[1], az=d->accel[2];
  float wx=cp*ax + sp*sr*ay + sp*cr*az;
  float wy=cr*ay - sr*az;
  float wz=-sp*ax + cp*sr*ay + cp*cr*az;
  /* 官方 accel 单位是 G，因此重力分量应减 1 G，而不是 9.81 m/s^2。 */
  d->accel_v = wz - 1.0f;
  d->accel_h = sqrtf(wx*wx + wy*wy);
}

/** @brief 清零协议状态、历史数据和时间基准。 */
void Imu_Init(void)
{
  memset(&g_parser, 0, sizeof(g_parser));
  g_last_sample_timestamp_us = 0U;
  g_valid_sample_count = 0U;
}

uint32_t Imu_GetLastSampleTimestampUs(void)
{
  return g_last_sample_timestamp_us;
}

uint32_t Imu_GetValidSampleCount(void)
{
  return g_valid_sample_count;
}

/**
 * @brief IMU 任务的一次处理循环。
 * @note 队列中一个元素是一个 DMA 接收块，不保证恰好对应一帧；状态机负责跨块拼帧。
 */
void Imu_Task_Process(void)
{
  Imu_Usart2_Service();
  Imu_Rx_Block_t block;
  if (osMessageQueueGet(IMU_QueueHandle, &block, NULL, 0U) != osOK) return;
  Imu_Data_t sample;
  /* 只处理本次DMA空闲回调实际收到的字节，避免把补零数据送入状态机。 */
  for (uint16_t i=0; i<block.length; ++i) {
    if (Imu_Parse_Byte(&g_parser, block.data[i], &sample)) {
      /* 官方 gyro 随帧直接解析；首帧也可以立即下发，不再等待差分历史。 */
      sample.timestamp_us = TIM7_GetTimestampUs();
      Imu_Coordinate_Process(&sample);
      g_last_sample_timestamp_us = sample.timestamp_us;
      g_valid_sample_count++;
      imu_dispatch_gimbal(&sample);
    }
  }
}
