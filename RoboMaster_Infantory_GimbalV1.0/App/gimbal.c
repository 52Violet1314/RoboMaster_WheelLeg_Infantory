#include "gimbal.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "CRC.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

GimbalState_Recv_t gimbal_recv_state;

/* ---- 调试用全局变量，可直接在 Watch/Expression 中查看 ---- */
uint8_t  gimbal_rx_raw[64];      /* 最近一次 USB 接收到的原始数据 */
uint32_t gimbal_rx_raw_len;      /* 最近一次实际接收长度 */
uint32_t gimbal_rx_cb_cnt;       /* CDC 回调被调用的次数 */
uint32_t gimbal_unpack_ok_cnt;   /* 解包成功次数 */
uint32_t gimbal_unpack_fail_cnt; /* 解包失败次数 */
uint8_t  gimbal_fail_reason;     /* 最近一次失败原因: 1=len 2=header 3=crc */

/**
 * @brief 解包上位机下发的云台指令帧
 * @param buf 原始数据缓冲区
 * @param len 数据长度
 * @retval 1 解包成功，0 失败
 */
static uint8_t usb_unpack_gimbal_state(uint8_t *buf, uint32_t len)
{
    uint16_t crc, rx_crc;

    gimbal_rx_cb_cnt++;

    /* 先把原始数据拷出来，方便调试（buf 为 NULL 或超长时 len 置 0） */
    if (buf != NULL && len > 0 && len <= sizeof(gimbal_rx_raw))
    {
        gimbal_rx_raw_len = len;
        memcpy(gimbal_rx_raw, buf, len);
    }
    else
    {
        gimbal_rx_raw_len = 0;
    }

    if (buf == NULL || len < sizeof(GimbalState_Recv_t))
    {
        gimbal_fail_reason = 1; /* 长度不足或指针为空 */
        gimbal_unpack_fail_cnt++;
        return 0;
    }
    if (buf[0] != 0x51 || buf[1] != 0x59)
    {
        gimbal_fail_reason = 2; /* 帧头错误 */
        gimbal_unpack_fail_cnt++;
        return 0;
    }
    crc = Get_CRC16_Check_Sum(buf, GIMBAL_RECV_CRC_OFFSET, CRC_INIT);
    memcpy(&rx_crc, buf + GIMBAL_RECV_CRC_OFFSET, sizeof(rx_crc));
    if (crc != rx_crc)
    {
        gimbal_fail_reason = 3; /* CRC 校验失败 */
        gimbal_unpack_fail_cnt++;
        return 0;
    }
    memcpy(&gimbal_recv_state, buf, sizeof(GimbalState_Recv_t));
    gimbal_fail_reason = 0;
    gimbal_unpack_ok_cnt++;
    return 1;
}

/**
 * @brief USB CDC 接收回调（由 usbd_cdc_if.c 的类回调表调用）
 * @param Buf 接收数据缓冲区
 * @param Len 接收数据长度
 */
int8_t CDC_Receive_HS(uint8_t *Buf, uint32_t *Len)
{
    usb_unpack_gimbal_state(Buf, *Len);
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);
    return (USBD_OK);
}

/**
 * @brief 通过 USB CDC 虚拟串口发送云台姿态
 * @param roll  横滚角
 * @param pitch 俯仰角
 * @param yaw   偏航角
 */
void usb_send_gimbal_state(float roll, float pitch, float yaw)
{
    GimbalState_Send_t state;

    state.header[0] = 0x51;
    state.header[1] = 0x59;
    state.mode = 0;
    state.roll = roll;
    state.pitch = pitch;
    state.yaw = yaw;
    state.crc16 = 0;
    state.crc16 = Get_CRC16_Check_Sum((uint8_t *)&state, GIMBAL_CRC_OFFSET, CRC_INIT);
    CDC_Transmit_HS((uint8_t *)&state, sizeof(state));
}
