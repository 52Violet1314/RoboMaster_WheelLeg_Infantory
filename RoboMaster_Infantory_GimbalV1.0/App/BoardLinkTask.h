#ifndef BOARD_LINK_TASK_H
#define BOARD_LINK_TASK_H
#include <stdint.h>
#include "board_link_protocol.h"
void BoardLinkTask(void *argument);
void BoardLink_OnRx(const uint8_t *data, uint16_t size);
void BoardLink_RestartRx(void);
extern uint8_t BoardLink_DmaRxBuffer[BOARD_LINK_RX_BUFFER_SIZE];
extern volatile BoardLink_ChassisState_t BoardLink_LastChassisState;
extern volatile uint32_t BoardLink_RxOkCount, BoardLink_RxErrorCount;
#endif
