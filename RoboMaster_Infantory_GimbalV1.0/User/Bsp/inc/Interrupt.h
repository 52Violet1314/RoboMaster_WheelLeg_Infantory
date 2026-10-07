#ifndef USER_BSP_INTERRUPT_H
#define USER_BSP_INTERRUPT_H
#include <stdint.h>
#include "main.h"
#include "fdcan.h"
#define IMU_DMA_BUFFER_SIZE 164U
#define REFEREE_UART7_DMA_BUFFER_SIZE 512U
#define VISION_USART10_DMA_BUFFER_SIZE 256U
#define COMM_UART_FAULT_IMU     (1U << 0)
#define COMM_UART_FAULT_REMOTE  (1U << 1)
#define COMM_UART_FAULT_REFEREE (1U << 2)
#define COMM_UART_FAULT_VISION  (1U << 3)
/* 队列消息：DMA 缓冲区字节 + 本次空闲回调实际收到的有效字节数。 */
typedef struct {
  uint8_t data[IMU_DMA_BUFFER_SIZE];
  uint16_t length;
} Imu_Rx_Block_t;

/**
 * @brief FDCAN 接收中断路径诊断快照。
 * @details 所有计数均从上电开始累计：callback 表示 HAL 已分发 FIFO 中断，
 * read_entry 表示已进入 read_fifo，message 表示已成功从 FIFO 取出一帧。
 * 临时仅供 DebugTask 打印，不能据此参与任何控制或掉线保护。电机调试完成
 * 后可连同 Fdcan_GetRxDiagnostic() 和 DebugTask 中的 printf 一并删除。
 */
typedef struct {
  uint32_t fifo0_callback_count;
  uint32_t fifo1_callback_count;
  uint32_t fifo0_read_entry_count;
  uint32_t fifo1_read_entry_count;
  uint32_t fifo0_message_count;
  uint32_t fifo1_message_count;
  uint32_t fifo0_unhandled_standard_count;
  uint32_t last_fifo0_unhandled_standard_id;
  uint32_t last_fifo0_unhandled_data_length;
  uint32_t fifo0_fill_level;
  uint32_t fifo1_fill_level;
  /* FDCAN 物理链路状态：用于区分“程序没有发帧”和“总线帧没有被 ACK”。 */
  uint32_t tx_fifo_free_level;
  uint32_t tx_error_count;
  uint32_t rx_error_count;
  uint32_t error_logging_count;
  uint32_t last_error_code;
  uint32_t error_passive;
  uint32_t warning;
  uint32_t bus_off;
  uint32_t hal_error_code;
} Fdcan_Rx_Diagnostic_t;
extern uint8_t Imu_Dma_Rx_Buffer[IMU_DMA_BUFFER_SIZE];
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);
HAL_StatusTypeDef Imu_Usart2_Start_Receive(void);
void Imu_Usart2_Service(void);
void Remote_Uart_Dma_RxEvent(UART_HandleTypeDef *huart,
                             const uint8_t *data, uint16_t size);
/** 启动或重启 UART5 遥控器 SBUS 的 Receive-to-Idle DMA。 */
void Remote_Uart5_Start_Receive(void);
void Refree_Uart7_Start_Receive(void);
void Vision_Usart10_Start_Receive(void);
/** @brief InputTask 周期调用：重试 UART5/UART7/USART10 的错误后 DMA 重启。 */
void Communication_Uart_Service(void);
/** 当前 UART DMA 恢复状态位；只有对应 DMA 接收器成功重新启动后才会清除。 */
uint32_t Communication_UartGetRecoveryFlags(void);
/** 临时读取 FDCAN1 接收诊断快照；仅可由 DebugTask 在任务上下文调用。
 * @note 电机调试完成后可删除。 */
void Fdcan_GetRxDiagnostic(Fdcan_Rx_Diagnostic_t *diagnostic);
#endif
