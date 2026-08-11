#ifndef __SBUS_H
#define __SBUS_H
#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"

void SBUS_DataGet(uint8_t* sbus_rx_buf,int16_t* channels,Data_Frame_Status_Typedef* SBUSFrameStatus);
void SBUS_DataSend(uint8_t* sbus_rx_buf,QueueHandle_t SBUSQueue,Data_Frame_Status_Typedef* SBUSFrameStatus);
#endif
