#include "Interrupt.h"
#include "usart.h"
#include "Can_Motor.h"
#include "Remote.h"
#include "Refree.h"
#include "Vision.h"
#include "cmsis_os2.h"
#include "core_cm7.h"
#include <string.h>

uint8_t Imu_Dma_Rx_Buffer[IMU_DMA_BUFFER_SIZE]
    __attribute__((aligned(32), section(".RAM_D2")));
/* 遥控器标称 1 kHz；DMA 每次收一整帧即可触发回调并立即重新挂接，避免两个
 * 连续帧合并后才通知 InputTask。协议解析仍在任务中完成，不在 ISR 内解码。 */
static uint8_t Remote_Uart5_Dma_Rx_Buffer[REMOTE_SBUS_FRAME_SIZE]
    __attribute__((aligned(32), section(".RAM_D2")));
static uint8_t Referee_Uart7_Dma_Rx_Buffer[REFEREE_UART7_DMA_BUFFER_SIZE]
    __attribute__((aligned(32), section(".RAM_D2")));
static uint8_t Vision_Usart10_Dma_Rx_Buffer[VISION_USART10_DMA_BUFFER_SIZE]
    __attribute__((aligned(32), section(".RAM_D2")));
extern osMessageQueueId_t IMU_QueueHandle;
/* USART2 出错时先终止当前 DMA，再由 AbortReceiveCpltCallback 重新启动。 */
static volatile uint8_t s_imu_usart2_recovering = 0U;
static volatile uint8_t s_imu_usart2_restart_pending = 0U;
static volatile uint8_t s_remote_uart5_recovering = 0U;
static volatile uint8_t s_remote_uart5_restart_pending = 0U;
static volatile uint8_t s_referee_uart7_recovering = 0U;
static volatile uint8_t s_referee_uart7_restart_pending = 0U;
static volatile uint8_t s_vision_usart10_recovering = 0U;
static volatile uint8_t s_vision_usart10_restart_pending = 0U;
static volatile uint32_t s_uart_recovery_flags;
/* 中断内只递增这些计数，不调用 printf、队列或任何阻塞 HAL 函数。 */
static volatile uint32_t s_fdcan_fifo0_callback_count;
static volatile uint32_t s_fdcan_fifo1_callback_count;
static volatile uint32_t s_fdcan_fifo0_read_entry_count;
static volatile uint32_t s_fdcan_fifo1_read_entry_count;
static volatile uint32_t s_fdcan_fifo0_message_count;
static volatile uint32_t s_fdcan_fifo1_message_count;
/* FIFO0 接到但不是 0x11/0x12/0x13 的标准帧：用于发现电机 MST_ID 配错，
 * 仅供 DebugTask 读取，不参与任何控制决策。 */
static volatile uint32_t s_fdcan_fifo0_unhandled_standard_count;
static volatile uint32_t s_fdcan_last_fifo0_unhandled_standard_id;
static volatile uint32_t s_fdcan_last_fifo0_unhandled_data_length;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == NULL) return;
  if (huart->Instance == UART5) {
    if (s_remote_uart5_recovering != 0U) return;
    if (Size == 0U) {
      Remote_Uart5_Start_Receive();
      return;
    }
    if (Size > sizeof(Remote_Uart5_Dma_Rx_Buffer))
      Size = sizeof(Remote_Uart5_Dma_Rx_Buffer);
    SCB_InvalidateDCache_by_Addr((uint32_t *)Remote_Uart5_Dma_Rx_Buffer,
                                 (Size + 31U) & ~31U);
    Remote_Uart_Dma_RxEvent(huart, Remote_Uart5_Dma_Rx_Buffer, Size);
    Remote_Uart5_Start_Receive();
    return;
  }
  if (huart->Instance == UART7) {
    if (s_referee_uart7_recovering != 0U) return;
    if (Size > sizeof(Referee_Uart7_Dma_Rx_Buffer))
      Size = sizeof(Referee_Uart7_Dma_Rx_Buffer);
    if (Size != 0U) {
      SCB_InvalidateDCache_by_Addr((uint32_t *)Referee_Uart7_Dma_Rx_Buffer,
                                   (Size + 31U) & ~31U);
      Refree_UART7_DmaRxEvent(Referee_Uart7_Dma_Rx_Buffer, Size);
    }
    Refree_Uart7_Start_Receive();
    return;
  }
  if (huart->Instance == USART10) {
    if (s_vision_usart10_recovering != 0U) return;
    if (Size > sizeof(Vision_Usart10_Dma_Rx_Buffer)) Size = sizeof(Vision_Usart10_Dma_Rx_Buffer);
    if (Size != 0U) {
      SCB_InvalidateDCache_by_Addr((uint32_t *)Vision_Usart10_Dma_Rx_Buffer, (Size + 31U) & ~31U);
      Vision_RxDmaEvent(Vision_Usart10_Dma_Rx_Buffer, Size);
    }
    Vision_Usart10_Start_Receive();
    return;
  }
  if (huart->Instance != USART2) return;
  /* 错误恢复期间收到的旧事件不能再次启动 DMA。 */
  if (s_imu_usart2_recovering != 0U) return;
  if (Size == 0U) {
    Imu_Usart2_Start_Receive();
    return;
  }
  if (Size > IMU_DMA_BUFFER_SIZE) Size = IMU_DMA_BUFFER_SIZE;
  SCB_InvalidateDCache_by_Addr((uint32_t *)Imu_Dma_Rx_Buffer,
                                (Size + 31U) & ~31U);
  Imu_Rx_Block_t block = {0};
  memcpy(block.data, Imu_Dma_Rx_Buffer, Size);
  block.length = Size;
  if (osMessageQueuePut(IMU_QueueHandle, &block, 0U, 0U) != osOK) {
    /* 队列满时保留最新 DMA 块，避免处理任务继续消费过时数据。
     * 这是 ISR 中的单生产者替换操作，任务不会在 ISR 内抢占。 */
    Imu_Rx_Block_t old_block;
    (void)osMessageQueueGet(IMU_QueueHandle, &old_block, NULL, 0U);
    (void)osMessageQueuePut(IMU_QueueHandle, &block, 0U, 0U);
  }
  Imu_Usart2_Start_Receive();
}

/**
 * @brief 任一接收 UART 出错后的统一恢复入口。
 * @details ISR 内只请求异步 Abort；Abort 完成后重启 DMA。若硬件仍忙，
 *          pending 标志交给 InputTask/IMU_Task 后续重试，避免中断中忙等。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  volatile uint8_t *recovering;
  volatile uint8_t *pending;
  if (huart == NULL) return;
  if (huart->Instance == USART2) {
    recovering = &s_imu_usart2_recovering; pending = &s_imu_usart2_restart_pending;
    s_uart_recovery_flags |= COMM_UART_FAULT_IMU;
  } else if (huart->Instance == UART5) {
    recovering = &s_remote_uart5_recovering; pending = &s_remote_uart5_restart_pending;
    s_uart_recovery_flags |= COMM_UART_FAULT_REMOTE;
  } else if (huart->Instance == UART7) {
    recovering = &s_referee_uart7_recovering; pending = &s_referee_uart7_restart_pending;
    s_uart_recovery_flags |= COMM_UART_FAULT_REFEREE;
  } else if (huart->Instance == USART10) {
    recovering = &s_vision_usart10_recovering; pending = &s_vision_usart10_restart_pending;
    s_uart_recovery_flags |= COMM_UART_FAULT_VISION;
  } else return;

  if (*recovering != 0U) return;
  *recovering = 1U;
  if (HAL_UART_AbortReceive_IT(huart) != HAL_OK) {
    *recovering = 0U;
    *pending = 1U;
  }
}

/** @brief DMA 接收终止完成后重新武装对应 Receive-to-Idle DMA。 */
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL) return;
  if (huart->Instance == USART2) {
    s_imu_usart2_recovering = 0U;
    if (Imu_Usart2_Start_Receive() != HAL_OK) s_imu_usart2_restart_pending = 1U;
  } else if (huart->Instance == UART5) {
    s_remote_uart5_recovering = 0U;
    Remote_Uart5_Start_Receive();
  } else if (huart->Instance == UART7) {
    s_referee_uart7_recovering = 0U;
    Refree_Uart7_Start_Receive();
  } else if (huart->Instance == USART10) {
    s_vision_usart10_recovering = 0U;
    Vision_Usart10_Start_Receive();
  }
}

HAL_StatusTypeDef Imu_Usart2_Start_Receive(void)
{
  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2,
                                                          Imu_Dma_Rx_Buffer,
                                                          IMU_DMA_BUFFER_SIZE);
  if (huart2.hdmarx != NULL)
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  if (status == HAL_OK)
    s_imu_usart2_restart_pending = 0U;
  else
    s_imu_usart2_restart_pending = 1U;
  if (status == HAL_OK) s_uart_recovery_flags &= ~COMM_UART_FAULT_IMU;
  else s_uart_recovery_flags |= COMM_UART_FAULT_IMU;
  return status;
}

/** @brief 在任务上下文重试 USART2 DMA 启动，避免错误中断中忙等。 */
void Imu_Usart2_Service(void)
{
  if (s_imu_usart2_restart_pending == 0U ||
      s_imu_usart2_recovering != 0U) return;
  if (Imu_Usart2_Start_Receive() == HAL_OK)
    s_imu_usart2_restart_pending = 0U;
}

/*
 * 遥控器 UART DMA 接收入口。
 * UART5 在 HAL_UARTEx_RxEventCallback() 的 UART5 分支调用本函数。
 * 这里不绑定其他串口，避免覆盖 IMU、图传和视觉接收链路。
 */
void Remote_Uart_Dma_RxEvent(UART_HandleTypeDef *huart,
                             const uint8_t *data, uint16_t size)
{
  (void)huart;
  if (data == NULL || size == 0U) return;
  Remote_RxDmaEvent(data, size);
}

/** @brief 启动或重启 UART5 的 SBUS Receive-to-Idle DMA 接收。 */
void Remote_Uart5_Start_Receive(void)
{
  const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart5, Remote_Uart5_Dma_Rx_Buffer, sizeof(Remote_Uart5_Dma_Rx_Buffer));
  if (huart5.hdmarx != NULL)
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
  s_remote_uart5_restart_pending = (status == HAL_OK) ? 0U : 1U;
  if (status == HAL_OK) s_uart_recovery_flags &= ~COMM_UART_FAULT_REMOTE;
  else s_uart_recovery_flags |= COMM_UART_FAULT_REMOTE;
}

/**
 * @brief 启动或重启 UART7 裁判系统 VTM 的 Receive-to-Idle DMA 接收。
 * @details 关闭半传输中断，避免同一 DMA 块被半传输和空闲事件重复上报；保留
 *          全传输中断作为连续数据流兜底。DMA 缓冲位于 RAM_D2 且按缓存行对齐，
 *          启动前和完成后均失效缓存，以确保后续开启 D-Cache 时仍然正确。
 */
void Refree_Uart7_Start_Receive(void)
{
  SCB_InvalidateDCache_by_Addr((uint32_t *)Referee_Uart7_Dma_Rx_Buffer,
                               sizeof(Referee_Uart7_Dma_Rx_Buffer));
  const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart7, Referee_Uart7_Dma_Rx_Buffer, sizeof(Referee_Uart7_Dma_Rx_Buffer));
  if (huart7.hdmarx != NULL)
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
  s_referee_uart7_restart_pending = (status == HAL_OK) ? 0U : 1U;
  if (status == HAL_OK) s_uart_recovery_flags &= ~COMM_UART_FAULT_REFEREE;
  else s_uart_recovery_flags |= COMM_UART_FAULT_REFEREE;
}

/** 启动或重启 USART10 视觉链路的 Receive-to-Idle DMA。 */
void Vision_Usart10_Start_Receive(void)
{
  SCB_InvalidateDCache_by_Addr((uint32_t *)Vision_Usart10_Dma_Rx_Buffer, sizeof(Vision_Usart10_Dma_Rx_Buffer));
  const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart10, Vision_Usart10_Dma_Rx_Buffer, sizeof(Vision_Usart10_Dma_Rx_Buffer));
  if (huart10.hdmarx != NULL) __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
  s_vision_usart10_restart_pending = (status == HAL_OK) ? 0U : 1U;
  if (status == HAL_OK) s_uart_recovery_flags &= ~COMM_UART_FAULT_VISION;
  else s_uart_recovery_flags |= COMM_UART_FAULT_VISION;
}

void Communication_Uart_Service(void)
{
  if (s_remote_uart5_restart_pending != 0U && s_remote_uart5_recovering == 0U)
    Remote_Uart5_Start_Receive();
  if (s_referee_uart7_restart_pending != 0U && s_referee_uart7_recovering == 0U)
    Refree_Uart7_Start_Receive();
  if (s_vision_usart10_restart_pending != 0U && s_vision_usart10_recovering == 0U)
    Vision_Usart10_Start_Receive();
}

uint32_t Communication_UartGetRecoveryFlags(void)
{
  return s_uart_recovery_flags;
}


/**
 * @brief 读取指定 FDCAN FIFO 中的全部待处理消息并分发解析。
 * @param hfdcan FDCAN 外设句柄。
 * @param fifo   FIFO0 或 FIFO1 标识。
 * @details 该函数运行在 FDCAN 中断回调上下文，只做快速取帧和数据更新，
 *          不执行 PID、阻塞等待或复杂业务逻辑。
 */
static void read_fifo(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];
  if (fifo == FDCAN_RX_FIFO0) s_fdcan_fifo0_read_entry_count++;
  else if (fifo == FDCAN_RX_FIFO1) s_fdcan_fifo1_read_entry_count++;
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo) != 0U) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &header, data) != HAL_OK) break;
    if (fifo == FDCAN_RX_FIFO0) s_fdcan_fifo0_message_count++;
    else if (fifo == FDCAN_RX_FIFO1) s_fdcan_fifo1_message_count++;
    /* ISR 只识别标准反馈 ID 并投递轻量事件。三台电机的时间戳、超时判断、
     * 驱动器状态机和重使能全部由 CanTask 的 Can_Motor_Process() 统一处理。 */
    /* HAL 已经将 Message RAM 的 DLC 字段右移 16 位，DataLength 此时就是
     * DLC 值（经典 CAN 的 8 字节帧为 8），不能再右移，否则会错误变为 0。 */
    if (header.Identifier == DM_MOTOR_LEFT_FB_ID)
      Can_Motor_OnFeedbackFrame(DM_MOTOR_LEFT, data, (uint8_t)header.DataLength);
    else if (header.Identifier == DM_MOTOR_RIGHT_FB_ID)
      Can_Motor_OnFeedbackFrame(DM_MOTOR_RIGHT, data, (uint8_t)header.DataLength);
    else if (header.Identifier == DM_MOTOR_PITCH_FB_ID)
      Can_Motor_OnFeedbackFrame(DM_MOTOR_PITCH, data, (uint8_t)header.DataLength);
    else if (fifo == FDCAN_RX_FIFO0 && header.IdType == FDCAN_STANDARD_ID) {
      s_fdcan_fifo0_unhandled_standard_count++;
      s_fdcan_last_fifo0_unhandled_standard_id = header.Identifier;
      s_fdcan_last_fifo0_unhandled_data_length = header.DataLength;
    }
  }
}

/**
 * @brief FDCAN FIFO0 新消息 HAL 回调。
 * @param hfdcan    触发回调的 FDCAN 句柄。
 * @param RxFifo0ITs HAL 提供的 FIFO0 中断原因位。
 * @details FIFO0 接收两个摩擦轮反馈帧。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  (void)RxFifo0ITs;
  if (hfdcan == &hfdcan1) {
    s_fdcan_fifo0_callback_count++;
    read_fifo(hfdcan, FDCAN_RX_FIFO0);
  }
}

/**
 * @brief FDCAN FIFO1 新消息 HAL 回调。
 * @param hfdcan    触发回调的 FDCAN 句柄。
 * @param RxFifo1ITs HAL 提供的 FIFO1 中断原因位。
 * @details FIFO1 接收 Pitch 的反馈帧。
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  (void)RxFifo1ITs;
  if (hfdcan == &hfdcan1) {
    s_fdcan_fifo1_callback_count++;
    read_fifo(hfdcan, FDCAN_RX_FIFO1);
  }
}

void Fdcan_GetRxDiagnostic(Fdcan_Rx_Diagnostic_t *diagnostic)
{
  FDCAN_ProtocolStatusTypeDef protocol_status = {0};
  FDCAN_ErrorCountersTypeDef error_counters = {0};
  /* 临时调试读取：只复制 ISR 计数，绝不影响 FIFO、解析或电机控制。 */
  if (diagnostic == NULL) return;
  /* Cortex-M7 对齐的 32 位读写是原子的；统计值允许与 ISR 同时更新。 */
  diagnostic->fifo0_callback_count = s_fdcan_fifo0_callback_count;
  diagnostic->fifo1_callback_count = s_fdcan_fifo1_callback_count;
  diagnostic->fifo0_read_entry_count = s_fdcan_fifo0_read_entry_count;
  diagnostic->fifo1_read_entry_count = s_fdcan_fifo1_read_entry_count;
  diagnostic->fifo0_message_count = s_fdcan_fifo0_message_count;
  diagnostic->fifo1_message_count = s_fdcan_fifo1_message_count;
  diagnostic->fifo0_unhandled_standard_count = s_fdcan_fifo0_unhandled_standard_count;
  diagnostic->last_fifo0_unhandled_standard_id = s_fdcan_last_fifo0_unhandled_standard_id;
  diagnostic->last_fifo0_unhandled_data_length = s_fdcan_last_fifo0_unhandled_data_length;
  diagnostic->fifo0_fill_level = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
  diagnostic->fifo1_fill_level = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1);
  diagnostic->tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
  diagnostic->hal_error_code = hfdcan1.ErrorCode;
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK) {
    diagnostic->last_error_code = protocol_status.LastErrorCode;
    diagnostic->error_passive = protocol_status.ErrorPassive;
    diagnostic->warning = protocol_status.Warning;
    diagnostic->bus_off = protocol_status.BusOff;
  }
  if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
    diagnostic->tx_error_count = error_counters.TxErrorCnt;
    diagnostic->rx_error_count = error_counters.RxErrorCnt;
    diagnostic->error_logging_count = error_counters.ErrorLogging;
  }
}

/** 仅 Bus-Off 请求恢复；普通帧错误不停止、不重启 FDCAN。 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_ProtocolStatusTypeDef protocol_status;
  if (hfdcan != &hfdcan1) return;
  if (HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status) == HAL_OK &&
      protocol_status.BusOff != 0U)
    Can_Motor_RequestBusOffRecovery(hfdcan);
}
