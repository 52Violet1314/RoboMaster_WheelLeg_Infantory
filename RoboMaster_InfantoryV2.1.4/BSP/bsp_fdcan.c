#include "bsp_fdcan.h"
#include "app_motor.h"
#include "bsp_uart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "portmacro.h"
#include "timers.h"
#include "stm32h7xx_hal_tim.h"
#include "task.h"
#include "main.h"
#include "app_Task.hpp"
#include "CalculateTask.hpp"
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
volatile DM_Motor_Fault_Record_t dm_motor_fault_record = {0};
volatile uint8_t dm_motor_feedback_timeout_flag = 1U;

#define DM_MOTOR_FEEDBACK_COUNT       8U
#define DM_MOTOR_FEEDBACK_TIMEOUT_MS  10U

static volatile TickType_t dm_motor_last_feedback_tick[DM_MOTOR_FEEDBACK_COUNT] = {0};
static StaticTimer_t dm_motor_timeout_timer_buffer;
static TimerHandle_t dm_motor_timeout_timer = NULL;

static void dm_motor_timeout_timer_callback(TimerHandle_t xTimer) {
  TickType_t now = xTaskGetTickCount();
  (void)xTimer;

  for (uint32_t i = 0; i < DM_MOTOR_FEEDBACK_COUNT; ++i) {
    TickType_t last = dm_motor_last_feedback_tick[i];
    if ((last == 0U) ||
        ((now - last) > pdMS_TO_TICKS(DM_MOTOR_FEEDBACK_TIMEOUT_MS))) {
      dm_motor_feedback_timeout_flag = 1U;
      return;
    }
  }
  dm_motor_feedback_timeout_flag = 0U;
}

static void dm_motor_feedback_mark_received(uint32_t index) {
  dm_motor_last_feedback_tick[index] = xTaskGetTickCountFromISR();
}

static void dm_motor_record_fault(uint16_t feedback_id, uint8_t motor_id,
                                   uint8_t status,
                                   DM_Motor_Fault_Reason_t reason) {
  dm_motor_fault_record.feedback_id = feedback_id;
  dm_motor_fault_record.motor_id = motor_id;
  dm_motor_fault_record.status = status;
  dm_motor_fault_record.reason = (uint8_t)reason;
  dm_motor_fault_record.count++;
}

/* 达妙反馈帧：标准数据帧、固定 8 字节，首字节低四位为电机 ID。 */
static uint8_t dm_motor_feedback_valid(const FDCAN_RxHeaderTypeDef *header,
                                       const uint8_t *data,
                                       uint16_t expected_feedback_id,
                                       uint8_t expected_motor_id) {
  uint8_t status;

  if ((header->IdType != FDCAN_STANDARD_ID) ||
      (header->RxFrameType != FDCAN_DATA_FRAME) ||
      (header->DataLength != FDCAN_DLC_BYTES_8) ||
      (header->Identifier != expected_feedback_id) ||
      ((data[0] & 0x0F) != expected_motor_id)) {
    dm_motor_record_fault(header->Identifier, expected_motor_id,
                          (header->DataLength == FDCAN_DLC_BYTES_8)
                              ? (uint8_t)(data[0] >> 4)
                              : 0xFF,
                          DM_MOTOR_FAULT_INVALID_FRAME);
    return 0;
  }

  status = (uint8_t)(data[0] >> 4);
  if ((status != 0U) && (status != 1U)) {
    dm_motor_record_fault(expected_feedback_id, expected_motor_id, status,
                          DM_MOTOR_FAULT_DRIVER_STATUS);
    return 0;
  }

  return 1;
}

void bsp_can_init(void) {

  can_filter_init();
  HAL_FDCAN_Start(&hfdcan1); // 开启FDCAN
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_Start(&hfdcan3);
  /* FDCAN1: 逐帧 RX + 错误通知 */
  HAL_FDCAN_ActivateNotification(&hfdcan1,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan1,
      FDCAN_IT_BUS_OFF | FDCAN_IT_ARB_PROTOCOL_ERROR |
          FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_PASSIVE |
          FDCAN_IT_ERROR_WARNING,
      0);
  /* FDCAN2: 逐帧 RX + 错误通知 */
  HAL_FDCAN_ActivateNotification(&hfdcan2,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2,
      FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan2,
      FDCAN_IT_BUS_OFF | FDCAN_IT_ARB_PROTOCOL_ERROR |
          FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_PASSIVE |
          FDCAN_IT_ERROR_WARNING,
      0);
  /* FDCAN3: 逐帧 RX + 错误通知 */
  HAL_FDCAN_ActivateNotification(&hfdcan3,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3,
      FDCAN_IT_RX_FIFO1_WATERMARK | FDCAN_IT_RX_FIFO1_FULL, 0);
  HAL_FDCAN_ActivateNotification(&hfdcan3,
      FDCAN_IT_BUS_OFF | FDCAN_IT_ARB_PROTOCOL_ERROR |
          FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_PASSIVE |
          FDCAN_IT_ERROR_WARNING,
      0);

  dm_motor_timeout_timer = xTimerCreateStatic(
      "DMFbWd", pdMS_TO_TICKS(1), pdTRUE, NULL,
      dm_motor_timeout_timer_callback, &dm_motor_timeout_timer_buffer);
  if (dm_motor_timeout_timer != NULL) {
    xTimerStart(dm_motor_timeout_timer, 0);
  }
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
  fdcan_filter.FilterID1 = 0x091;
  fdcan_filter.FilterID2 = 0x094;          // 只接收四个髋关节电机反馈
  FDCAN_FilterTypeDef fdcan_filter2;

  fdcan_filter2.IdType = FDCAN_STANDARD_ID; // 标准ID
  fdcan_filter2.FilterIndex = 0;            // 滤波器索引
  fdcan_filter2.FilterType = FDCAN_FILTER_RANGE;
  fdcan_filter2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 两个轮毂反馈进入FIFO0
  fdcan_filter2.FilterID1 = 0x095;
  fdcan_filter2.FilterID2 = 0x096;

  FDCAN_FilterTypeDef fdcan_filter2_fifo1;
  fdcan_filter2_fifo1.IdType = FDCAN_STANDARD_ID;
  fdcan_filter2_fifo1.FilterIndex = 1;
  fdcan_filter2_fifo1.FilterType = FDCAN_FILTER_DUAL;
  fdcan_filter2_fifo1.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  fdcan_filter2_fifo1.FilterID1 = 0x097; // 拨弹 DM_3507 反馈
  fdcan_filter2_fifo1.FilterID2 = 0x098; // yaw DM_4310 反馈

  FDCAN_FilterTypeDef fdcan_filter3;

  fdcan_filter3.IdType = FDCAN_STANDARD_ID;             // 标准ID
  fdcan_filter3.FilterIndex = 0;                        // 滤波器索引
  fdcan_filter3.FilterType = FDCAN_FILTER_RANGE;        // 范围过滤模式
  fdcan_filter3.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
  fdcan_filter3.FilterID1 = 0x00;                       // 起始ID
  fdcan_filter3.FilterID2 = 0x7FF; // 结束ID，允许接收所有标准ID

  HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); // 接收ID2
  // 只允许匹配的消息进入FIFO0，拒绝不匹配的标准ID和扩展ID,不接受远程帧
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  // 配置FIFO0为阻塞模式，FIFO已满时拒绝新消息，避免旧消息被覆盖
  HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_BLOCKING);
  HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 5);
  HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter2); // 接收ID2
  HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter2_fifo1);
  // 拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
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
volatile uint32_t fdcan1_tx_fifo_full_count = 0;
volatile uint32_t fdcan2_tx_fifo_full_count = 0;
volatile uint32_t fdcan3_tx_fifo_full_count = 0;
volatile uint32_t fdcan1_bus_off_count = 0;
volatile uint32_t fdcan2_bus_off_count = 0;
volatile uint32_t fdcan3_bus_off_count = 0;
volatile uint32_t fdcan1_error_warning_count = 0;
volatile uint32_t fdcan2_error_warning_count = 0;
volatile uint32_t fdcan3_error_warning_count = 0;
volatile uint32_t fdcan1_error_passive_count = 0;
volatile uint32_t fdcan2_error_passive_count = 0;
volatile uint32_t fdcan3_error_passive_count = 0;
volatile uint32_t fdcan1_restart_count = 0;
volatile uint32_t fdcan2_restart_count = 0;
volatile uint32_t fdcan3_restart_count = 0;
volatile uint32_t fdcan1_rx_fifo0_full_count = 0;
volatile uint32_t fdcan2_rx_fifo0_full_count = 0;
volatile uint32_t fdcan2_rx_fifo1_full_count = 0;
volatile uint32_t fdcan3_rx_fifo0_full_count = 0;
volatile uint32_t fdcan3_rx_fifo1_full_count = 0;

uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data,
                         uint32_t len) {
  FDCAN_TxHeaderTypeDef pTxHeader = pTxHeaderBase;
  pTxHeader.Identifier = id;
  pTxHeader.DataLength = len_to_dlc(len);
  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK) {
    fdcan_tx_fail_count++;
    if (hfdcan == &hfdcan1) {
      fdcan1_tx_fifo_full_count++;
    } else if (hfdcan == &hfdcan2) {
      fdcan2_tx_fifo_full_count++;
    } else if (hfdcan == &hfdcan3) {
      fdcan3_tx_fifo_full_count++;
    }
    return 1;
  }
  return 0;
}

static void motor_data_ready_check_from_isr(void) {
  BaseType_t pxHighPriorityTaskWoken = pdFALSE;
  if (Motor_Data_ReadyBit == 0x3F) {
    xEventGroupSetBitsFromISR(DataGroup, MOTOR_Data_READY_BIT,
                              &pxHighPriorityTaskWoken);
    if (pxHighPriorityTaskWoken == pdTRUE) {
      portYIELD();
    }
    Motor_Data_ReadyBit = 0;
    Motor_Data_LastComplete = xTaskGetTickCountFromISR();
  }
  /* 该旧六电机同步位保留给其他任务；10ms 内未凑齐则丢弃当前累积。
   * 首次凑不齐时也会超时清零。 */
  else if (Motor_Data_ReadyBit != 0) {
    TickType_t now = xTaskGetTickCountFromISR();
    if (Motor_Data_LastComplete == 0) {
      Motor_Data_LastComplete = now;  /* 初始计时起点 */
    } else if ((now - Motor_Data_LastComplete) > pdMS_TO_TICKS(10)) {
      Motor_Data_ReadyBit = 0;
      Motor_Data_LastComplete = 0;    /* 重置，下一轮重新计时 */
    }
  }
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

    // 软件过滤特定ID
    switch (rec_id) {
      case 0x091: //  8009电机1ID
        if (dm_motor_feedback_valid(&pRxHeader, buf, 0x091, 0x01)) {
          DM_Motor_8009_Data_Get(&DM_8009P1, buf);
          dm_motor_feedback_mark_received(0);
          Motor_Data_ReadyBit |= (1 << 0);
        }
        break;
      case 0x092: //  8009电机2ID
        if (dm_motor_feedback_valid(&pRxHeader, buf, 0x092, 0x02)) {
          DM_Motor_8009_Data_Get(&DM_8009P2, buf);
          dm_motor_feedback_mark_received(1);
          Motor_Data_ReadyBit |= (1 << 1);
        }
        break;
      case 0x093: //  8009电机3ID
        if (dm_motor_feedback_valid(&pRxHeader, buf, 0x093, 0x03)) {
          DM_Motor_8009_Data_Get(&DM_8009P3, buf);
          dm_motor_feedback_mark_received(2);
          Motor_Data_ReadyBit |= (1 << 2);
        }
        break;
      case 0x094: //  8009电机4ID
        if (dm_motor_feedback_valid(&pRxHeader, buf, 0x094, 0x04)) {
          DM_Motor_8009_Data_Get(&DM_8009P4, buf);
          dm_motor_feedback_mark_received(3);
          Motor_Data_ReadyBit |= (1 << 3);
        }
        break;
      default:
        // 忽略其他ID
        break;
    }
  }
  motor_data_ready_check_from_isr();
}

void fdcan2_rx_callback(void) {
  uint8_t buf[64];
  FDCAN_RxHeaderTypeDef pRxHeader;

  while (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &pRxHeader, buf) ==
         HAL_OK) {
    switch (pRxHeader.Identifier) {
      case 0x095:
        if (dm_motor_feedback_valid(&pRxHeader, buf, 0x095, 0x05)) {
          DM_Motor_3519_Data_Get(&DM_3519L, buf);
          dm_motor_feedback_mark_received(4);
          Motor_Data_ReadyBit |= (1 << 4);
        }
        break;
      case 0x096:
        if (dm_motor_feedback_valid(&pRxHeader, buf, 0x096, 0x06)) {
          DM_Motor_3519_Data_Get(&DM_3519R, buf);
          dm_motor_feedback_mark_received(5);
          Motor_Data_ReadyBit |= (1 << 5);
        }
        break;
      default:
        break;
    }
  }
  motor_data_ready_check_from_isr();
}

void fdcan2_rx_fifo1_callback(void) {
  uint8_t buf[64];
  FDCAN_RxHeaderTypeDef pRxHeader;

  while (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &pRxHeader, buf) ==
         HAL_OK) {
    switch (pRxHeader.Identifier) {
      case 0x097: // 拨弹 DM_3507
        if (dm_motor_feedback_valid(&pRxHeader, buf, 0x097, 0x07)) {
          DM_Motor_3507_Data_Get(&DM_3507, buf);
          dm_motor_feedback_mark_received(6);
        }
        break;
      case 0x098: // yaw DM_4310
        if (dm_motor_feedback_valid(&pRxHeader, buf, 0x098, 0x08)) {
          DM_Motor_4310_Data_Get(&DM_4310, buf);
          dm_motor_feedback_mark_received(7);
        }
        break;
      default:
        break;
    }
  }
}

void fdcan3_rx_callback(void) {
  uint8_t buf[64];
  uint16_t rec_id;
  // 解析接收数据
  FDCAN_RxHeaderTypeDef pRxHeader;
  
  while (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &pRxHeader, buf) ==
         HAL_OK) {
    rec_id = pRxHeader.Identifier;    
    switch (rec_id) {
    default:
      // 忽略其他ID
      break;
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) {
    if (hfdcan == &hfdcan1) {
      fdcan1_rx_fifo0_full_count++;
    } else if (hfdcan == &hfdcan2) {
      fdcan2_rx_fifo0_full_count++;
    } else if (hfdcan == &hfdcan3) {
      fdcan3_rx_fifo0_full_count++;
    }
  }
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
  if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL) {
    if (hfdcan == &hfdcan2) {
      fdcan2_rx_fifo1_full_count++;
    } else if (hfdcan == &hfdcan3) {
      fdcan3_rx_fifo1_full_count++;
    }
  }
  if (hfdcan == &hfdcan2) {
    fdcan2_rx_fifo1_callback();
  }
}

static void fdcan_restart_from_error_isr(FDCAN_HandleTypeDef *hfdcan) {
  /* 保留 CubeMX 的时序、滤波器和 Message RAM 配置，仅重启该控制器。 */
  SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);

  if (hfdcan == &hfdcan1) {
    fdcan1_restart_count++;
  } else if (hfdcan == &hfdcan2) {
    fdcan2_restart_count++;
  } else if (hfdcan == &hfdcan3) {
    fdcan3_restart_count++;
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs) {
  uint8_t restart_required = 0;

  if (ErrorStatusITs & FDCAN_IR_BO) {
    if (hfdcan == &hfdcan1) {
      fdcan1_bus_off_count++;
    } else if (hfdcan == &hfdcan2) {
      fdcan2_bus_off_count++;
    } else if (hfdcan == &hfdcan3) {
      fdcan3_bus_off_count++;
    }
    restart_required = 1;
  }
  if (ErrorStatusITs & FDCAN_IR_EP) {
    if (hfdcan == &hfdcan1) {
      fdcan1_error_passive_count++;
    } else if (hfdcan == &hfdcan2) {
      fdcan2_error_passive_count++;
    } else if (hfdcan == &hfdcan3) {
      fdcan3_error_passive_count++;
    }
    restart_required = 1;
  }
  if (ErrorStatusITs & FDCAN_IR_EW) {
    if (hfdcan == &hfdcan1) {
      fdcan1_error_warning_count++;
    } else if (hfdcan == &hfdcan2) {
      fdcan2_error_warning_count++;
    } else if (hfdcan == &hfdcan3) {
      fdcan3_error_warning_count++;
    }
    restart_required = 1;
  }

  if (restart_required) {
    fdcan_restart_from_error_isr(hfdcan);
  }
}
