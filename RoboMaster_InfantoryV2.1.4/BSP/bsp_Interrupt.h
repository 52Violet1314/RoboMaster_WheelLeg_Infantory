#ifndef BSP_INTERRUPT_H
#define BSP_INTERRUPT_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* UART DMA Receive-to-Idle and error callbacks for remote control and HI14. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* BSP_INTERRUPT_H */
