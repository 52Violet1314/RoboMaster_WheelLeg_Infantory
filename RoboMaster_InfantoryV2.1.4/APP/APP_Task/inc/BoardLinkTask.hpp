#ifndef BOARD_LINK_TASK_HPP
#define BOARD_LINK_TASK_HPP
#include <stdint.h>
#include "board_link_protocol.h"
#ifdef __cplusplus
extern "C" {
#endif
void BoardLinkTask(void *pvParameters);
void BoardLink_OnRx(const uint8_t *data, uint16_t size);
void BoardLink_RestartRx(void);
extern uint8_t BoardLink_DmaRxBuffer[BOARD_LINK_RX_BUFFER_SIZE];
extern volatile BoardLink_GimbalState_t BoardLink_LastGimbalState;
extern volatile uint32_t BoardLink_RxOkCount, BoardLink_RxErrorCount;
#ifdef __cplusplus
}
#endif
#endif
