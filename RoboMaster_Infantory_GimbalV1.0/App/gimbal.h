#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stdint.h>
#include <stddef.h>

typedef struct
{
  uint8_t header[2];   /* 'Q','Y' 帧头 */
  uint8_t mode;
  float   roll;
  float   pitch;
  float   yaw;
  uint16_t crc16;      /* CRC16 校验值 */
} GimbalState_Send_t;

typedef struct
{
  uint8_t header[2];   /* 'Q','Y' 帧头 */
  uint8_t mode;
  float   yaw;
  float   pitch;
  uint16_t crc16;      /* CRC16 校验值 */
} GimbalState_Recv_t;

#define GIMBAL_CRC_OFFSET      offsetof(GimbalState_Send_t, crc16)
#define GIMBAL_RECV_CRC_OFFSET offsetof(GimbalState_Recv_t, crc16)

/* USB 接收解包结果，主循环可直接读取 */
extern GimbalState_Recv_t gimbal_recv_state;

/* ---- 调试用全局变量（在 Watch 窗口查看） ---- */
extern uint8_t  gimbal_rx_raw[64];      /* 最近一次 USB 接收的原始数据 */
extern uint32_t gimbal_rx_raw_len;      /* 最近一次实际接收长度 */
extern uint32_t gimbal_rx_cb_cnt;       /* CDC 回调被调用的次数 */
extern uint32_t gimbal_unpack_ok_cnt;   /* 解包成功次数 */
extern uint32_t gimbal_unpack_fail_cnt; /* 解包失败次数 */
extern uint8_t  gimbal_fail_reason;     /* 1=len 2=header 3=crc */

/* USB CDC 接收回调（注册于 usbd_cdc_if.c 的类回调表） */
int8_t CDC_Receive_HS(uint8_t *Buf, uint32_t *Len);

/* 通过 USB CDC 虚拟串口发送云台姿态（roll/pitch/yaw） */
void usb_send_gimbal_state(float roll, float pitch, float yaw);

#endif /* __GIMBAL_H */
