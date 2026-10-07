#ifndef USER_COMMUNICATION_IMU_H
#define USER_COMMUNICATION_IMU_H
#include <stdint.h>
#include "Interrupt.h"
#include "cmsis_os2.h"

/* IMU 任务自身判定姿态数据新鲜度的阈值。 */
#define IMU_TASK_SAMPLE_TIMEOUT_MS (1000U)

/**
 * @brief 一帧经过协议校验和算法处理后的 IMU 快照。
 * @note HI91 原始协议的欧拉角/角速度为 deg/deg/s；解析完成后在本模块
 *       一次性转换为 rad/rad/s。加速度保持官方 G 单位。
 */
typedef struct
{
  float roll_rad, pitch_rad, yaw_rad; /* 机体欧拉角，rad */
  float gyro_rad_s[3]; /* 机体三轴角速度，rad/s */
  float accel[3];// HI91直接发送的三轴加速度，单位 G
  float accel_h, accel_v;// 坐标变换后得到的水平、竖直加速度
  uint32_t timestamp_us;// 基于 TIM7 的时间戳，单位为微秒
} Imu_Data_t;

/**
 * @brief HI91 字节流解析器状态。
 * @note 状态跨越多个 DMA 接收块保存，用于处理半帧、整帧和多帧数据。
 */
typedef struct
{
  uint8_t frame[164];
  uint16_t count, payload_length;
} Imu_Parser_t;

/** @brief 初始化解析器状态和共享 IMU 快照。 */
void Imu_Init(void);
/** @brief 从 IMU 队列取出接收块并完成解析、算法处理和派发到消费者队列。 */
void Imu_Task_Process(void);
/** @brief 最近一帧通过 HI91 校验并完成解析的 TIM7 微秒时间戳；供 ImuTask 判超时。 */
uint32_t Imu_GetLastSampleTimestampUs(void);
/** @brief 从上电起累计的、CRC 正确且完成 HI91 解析的有效帧数；仅供调试统计。 */
uint32_t Imu_GetValidSampleCount(void);
/** @brief 向协议状态机输入一个字节，成功得到完整帧时返回 1。 */
int Imu_Parse_Byte(Imu_Parser_t *parser, uint8_t byte, Imu_Data_t *out);
/** @brief 校验一帧 HI91 数据的长度和 CRC，正确返回 1。 */
int Imu_Check_Crc(const uint8_t *frame, uint16_t total_length);
/** @brief 将机体加速度转换到工程坐标并计算水平/竖直分量。 */
void Imu_Coordinate_Process(Imu_Data_t *data);

/**
 * @brief 云台 pitch 任务专用 IMU 数据包, 只含 pitch 角与角速度.
 * @note 用于 Gimbal_Imu_Queue 队列元素, 不暴露用不到的字段.
 *      所有角变量均为 rad/rad/s。
 */
typedef struct
{
  float pitch_rad;        /* IMU pitch 欧拉角，rad */
  float gyro_pitch_rad_s; /* IMU pitch 轴角速度，rad/s */
  uint32_t timestamp_us; /* 与 pitch 对应的 TIM7 时间戳 us */
} Imu_Pitch_Packet_t;

extern osMessageQueueId_t IMU_QueueHandle;
/* 云台 pitch 专用队列: Imu_Task -> Gimbal_Task (只含 rad/rad/s, 满则覆盖最新) */
extern osMessageQueueId_t Gimbal_Imu_QueueHandle;
#endif
