#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

#define CAN_CLASS   0
#define CAN_FD_BRS  1

#define CAN_BR_125K 0
#define CAN_BR_200K 1
#define CAN_BR_250K 2
#define CAN_BR_500K 3
#define CAN_BR_1M   4
#define CAN_BR_2M   5
#define CAN_BR_2M5  6
#define CAN_BR_3M2  7
#define CAN_BR_4M   8
#define CAN_BR_5M   9

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t fdcan_tx_fail_count;
/* 仅用于诊断：发送 FIFO 当前无法入队、Bus-Off 及 RX FIFO 满次数。 */
extern volatile uint32_t fdcan1_tx_fifo_full_count;
extern volatile uint32_t fdcan2_tx_fifo_full_count;
extern volatile uint32_t fdcan3_tx_fifo_full_count;
extern volatile uint32_t fdcan1_bus_off_count;
extern volatile uint32_t fdcan2_bus_off_count;
extern volatile uint32_t fdcan3_bus_off_count;
extern volatile uint32_t fdcan1_error_warning_count;
extern volatile uint32_t fdcan2_error_warning_count;
extern volatile uint32_t fdcan3_error_warning_count;
extern volatile uint32_t fdcan1_error_passive_count;
extern volatile uint32_t fdcan2_error_passive_count;
extern volatile uint32_t fdcan3_error_passive_count;
extern volatile uint32_t fdcan1_restart_count;
extern volatile uint32_t fdcan2_restart_count;
extern volatile uint32_t fdcan3_restart_count;
extern volatile uint32_t fdcan1_rx_fifo0_full_count;
extern volatile uint32_t fdcan2_rx_fifo0_full_count;
extern volatile uint32_t fdcan2_rx_fifo1_full_count;
extern volatile uint32_t fdcan3_rx_fifo0_full_count;
extern volatile uint32_t fdcan3_rx_fifo1_full_count;

typedef enum {
  DM_MOTOR_FAULT_NONE = 0,
  DM_MOTOR_FAULT_INVALID_FRAME,
  DM_MOTOR_FAULT_DRIVER_STATUS,
} DM_Motor_Fault_Reason_t;

/* 所有达妙电机共用的一份故障记录；由接收 ISR 写入，后续故障处理读取。 */
typedef struct {
  uint32_t count;
  uint16_t feedback_id;
  uint8_t motor_id;
  uint8_t status;
  uint8_t reason;
} DM_Motor_Fault_Record_t;

extern volatile DM_Motor_Fault_Record_t dm_motor_fault_record;
/* 八台电机各自的驱动状态故障锁存；任一非零时总标志置位。 */
extern volatile uint8_t dm_motor_driver_fault_state[8];
extern volatile uint8_t dm_motor_driver_fault_flag;
/* 八台底盘达妙电机任一台超过 10 ms 未收到有效反馈时置位。 */
extern volatile uint8_t dm_motor_feedback_timeout_flag;

void bsp_can_init(void);
void can_filter_init(void);
void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan2_rx_fifo1_callback(void);
void fdcan3_rx_callback(void);

#ifdef __cplusplus
}
#endif


#endif /* __BSP_FDCAN_H_ */

