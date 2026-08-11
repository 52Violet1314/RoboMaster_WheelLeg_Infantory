#include "bsp_fdcan.h"
#include "DMIMU.h"
#include "bsp_uart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "portmacro.h"
#include "stm32h7xx_hal_tim.h"
#include "task.h"
#include "main.h"
#include "tim.h"
/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
static uint8_t   Motor_Data_ReadyBit       = 0;
static TickType_t Motor_Data_LastComplete  = 0;
static uint8_t   Motor_Data_HasCompleted   = 0;

void bsp_can_init(void) {

  can_filter_init();
  HAL_FDCAN_Start(&hfdcan1); // 开启FDCAN
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_Start(&hfdcan3);
  /* FDCAN1: 逐帧 RX + 错误通知 */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan1,
      FDCAN_IT_BUS_OFF | FDCAN_IT_ARB_PROTOCOL_ERROR |
          FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_PASSIVE |
          FDCAN_IT_ERROR_WARNING,
      0);
  /* FDCAN2: 逐帧 RX + 错误通知 */
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2,
      FDCAN_IT_BUS_OFF | FDCAN_IT_ARB_PROTOCOL_ERROR |
          FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_PASSIVE |
          FDCAN_IT_ERROR_WARNING,
      0);
  /* FDCAN3: 逐帧 RX + 错误通知 */
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_BUFFER_NEW_MESSAGE, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3,
      FDCAN_IT_BUS_OFF | FDCAN_IT_ARB_PROTOCOL_ERROR |
          FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_PASSIVE |
          FDCAN_IT_ERROR_WARNING,
      0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void) {
  FDCAN_FilterTypeDef fdcan_filter;

  fdcan_filter.IdType = FDCAN_STANDARD_ID; // 标准ID
  fdcan_filter.FilterIndex = 0;            // 滤波器索引
  fdcan_filter.FilterType = FDCAN_FILTER_RANGE;
  fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter.FilterID1 = 0x090;
  fdcan_filter.FilterID2 = 0x09F;          // 只接收电机反馈 0x091-0x096
  FDCAN_FilterTypeDef fdcan_filter2;

  fdcan_filter2.IdType = FDCAN_STANDARD_ID; // 标准ID
  fdcan_filter2.FilterIndex = 0;            // 滤波器索引
  fdcan_filter2.FilterType = FDCAN_FILTER_MASK;
  fdcan_filter2.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // 过滤器0关联到FIFO1
  fdcan_filter2.FilterID1 = 0x00;
  fdcan_filter2.FilterID2 = 0x7FF;

  FDCAN_FilterTypeDef fdcan_filter3;

  fdcan_filter3.IdType = FDCAN_STANDARD_ID;             // 标准ID
  fdcan_filter3.FilterIndex = 0;                        // 滤波器索引
  fdcan_filter3.FilterType = FDCAN_FILTER_RANGE;        // 范围过滤模式
  fdcan_filter3.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter3.FilterID1 = 0x00;                       // 起始ID
  fdcan_filter3.FilterID2 = 0x7FF; // 结束ID，允许接收所有标准ID

  HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); // 接收ID2
  // 允许匹配的消息进入FIFO0，拒绝不匹配的标准ID和扩展ID,不接受远程帧
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  // 配置FIFO0为阻塞模式，FIFO已满时拒绝新消息，避免旧消息被覆盖
  HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_BLOCKING);
  HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 5);
  HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter2); // 接收ID2
  // 拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1);

  HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter3); // FDCAN3滤波器配置
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE,
                               FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);
}

void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud) {
  uint32_t nom_brp = 0, nom_seg1 = 0, nom_seg2 = 0, nom_sjw = 0;
  uint32_t dat_brp = 0, dat_seg1 = 0, dat_seg2 = 0, dat_sjw = 0;

  /*	nominal_baud = 80M/brp/(1+seg1+seg2)
          sample point = (1+seg1)/(1+seg1+sjw)  */
  if (mode == CAN_CLASS) {
    switch (baud) {
    case CAN_BR_125K:
      nom_brp = 4;
      nom_seg1 = 139;
      nom_seg2 = 20;
      nom_sjw = 20;
      break; // sample point 87.5%
    case CAN_BR_200K:
      nom_brp = 2;
      nom_seg1 = 174;
      nom_seg2 = 25;
      nom_sjw = 25;
      break; // sample point 87.5%
    case CAN_BR_250K:
      nom_brp = 2;
      nom_seg1 = 139;
      nom_seg2 = 20;
      nom_sjw = 20;
      break; // sample point 87.5%
    case CAN_BR_500K:
      nom_brp = 1;
      nom_seg1 = 139;
      nom_seg2 = 20;
      nom_sjw = 20;
      break; // sample point 87.5%
    case CAN_BR_1M:
      nom_brp = 1;
      nom_seg1 = 59;
      nom_seg2 = 20;
      nom_sjw = 20;
      break; // sample point 75%
    }
    dat_brp = 1;
    dat_seg1 = 29;
    dat_seg2 = 10;
    dat_sjw = 10; // 仲裁域默认1M
    hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  }
  /*	data_baud	 = 80M/brp/(1+seg1+seg2)
          sample point = (1+seg1)/(1+seg1+sjw)  */
  if (mode == CAN_FD_BRS) {
    switch (baud) {
    case CAN_BR_2M:
      dat_brp = 1;
      dat_seg1 = 29;
      dat_seg2 = 10;
      dat_sjw = 10;
      break; // sample point 75%
    case CAN_BR_2M5:
      dat_brp = 1;
      dat_seg1 = 25;
      dat_seg2 = 6;
      dat_sjw = 6;
      break; // sample point 81.25%
    case CAN_BR_3M2:
      dat_brp = 1;
      dat_seg1 = 19;
      dat_seg2 = 5;
      dat_sjw = 5;
      break; // sample point 80%
    case CAN_BR_4M:
      dat_brp = 1;
      dat_seg1 = 14;
      dat_seg2 = 5;
      dat_sjw = 5;
      break; // sample point 75%
    case CAN_BR_5M:
      dat_brp = 1;
      dat_seg1 = 12;
      dat_seg2 = 3;
      dat_sjw = 3;
      break; // sample point 81.25%
    }
    nom_brp = 1;
    nom_seg1 = 59;
    nom_seg2 = 20;
    nom_sjw = 20; // 数据域默认1M
    hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  }

  HAL_FDCAN_DeInit(hfdcan);

  hfdcan->Init.NominalPrescaler = nom_brp;
  hfdcan->Init.NominalTimeSeg1 = nom_seg1;
  hfdcan->Init.NominalTimeSeg2 = nom_seg2;
  hfdcan->Init.NominalSyncJumpWidth = nom_sjw;

  hfdcan->Init.DataPrescaler = dat_brp;
  hfdcan->Init.DataTimeSeg1 = dat_seg1;
  hfdcan->Init.DataTimeSeg2 = dat_seg2;
  hfdcan->Init.DataSyncJumpWidth = dat_sjw;

  HAL_FDCAN_Init(hfdcan);
}

/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id,
*uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
static uint32_t len_to_dlc(uint32_t len) {
  static const uint8_t dlc_map[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  if (len <= 8) return dlc_map[len];
  if (len == 12) return FDCAN_DLC_BYTES_12;
  if (len == 16) return FDCAN_DLC_BYTES_16;
  if (len == 20) return FDCAN_DLC_BYTES_20;
  if (len == 24) return FDCAN_DLC_BYTES_24;
  if (len == 32) return FDCAN_DLC_BYTES_32;
  if (len == 48) return FDCAN_DLC_BYTES_48;
  if (len == 64) return FDCAN_DLC_BYTES_64;
  return len;
}

static FDCAN_TxHeaderTypeDef pTxHeaderBase = {
    .Identifier = 0,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = 0,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_ON,
    .FDFormat = FDCAN_FD_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0,
};

uint32_t fdcan_tx_fail_count = 0;

uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data,
                         uint32_t len) {
  FDCAN_TxHeaderTypeDef pTxHeader = pTxHeaderBase;
  pTxHeader.Identifier = id;
  pTxHeader.DataLength = len_to_dlc(len);
  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK) {
    fdcan_tx_fail_count++;
    return 1;
  }
  return 0;
}

uint8_t fdcanx_send_data_fast(hcan_t *hfdcan, uint16_t id, uint8_t *data,
                              uint32_t len, FDCAN_TxHeaderTypeDef *header) {
  header->Identifier = id;
  header->DataLength = len_to_dlc(len);
  return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, header, data) != HAL_OK);
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf) {
  FDCAN_RxHeaderTypeDef pRxHeader;
  uint8_t len = 0;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) ==
      HAL_OK) {
    *rec_id = pRxHeader.Identifier;
    if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_8)
      len = pRxHeader.DataLength;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_12)
      len = 12;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_16)
      len = 16;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_20)
      len = 20;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_24)
      len = 24;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_32)
      len = 32;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_48)
      len = 48;
    else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_64)
      len = 64;

    return len; // 接收数据
  }
  return 0;
}

void fdcan1_rx_callback(void) {
  uint16_t rec_id;
  uint8_t buf[64];

  // 解析接收数据
  FDCAN_RxHeaderTypeDef pRxHeader;
  // 在一次中断中处理FIFO中的所有可用消息，提高处理效率
  while (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &pRxHeader, buf) ==
         HAL_OK) {
    rec_id = pRxHeader.Identifier;


  }

  /* 超时强制清零：5ms 内未凑齐 6 个电机数据则丢弃当前累积，防止死锁
   * 去掉 Motor_Data_HasCompleted 前置条件：首次凑不齐时也会超时清零 */

}

void fdcan2_rx_callback(void) {}

void fdcan3_rx_callback(void) {
  uint8_t buf[64];
  uint16_t rec_id;
  // 解析接收数据
  FDCAN_RxHeaderTypeDef pRxHeader;
  
  while (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &pRxHeader, buf) ==
         HAL_OK) {
    rec_id = pRxHeader.Identifier;    
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (hfdcan == &hfdcan1) {
    fdcan1_rx_callback();
  }
  if (hfdcan == &hfdcan2) {
    fdcan2_rx_callback();
  }
  if (hfdcan == &hfdcan3) {
    fdcan3_rx_callback();
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  if (hfdcan == &hfdcan2) {
    fdcan2_rx_callback();
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  if (ErrorStatusITs & FDCAN_IR_BO) {
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  }
  if (ErrorStatusITs & FDCAN_IR_EP) {
    // MX_FDCAN1_Init();
    // bsp_can_init();
  }
}
