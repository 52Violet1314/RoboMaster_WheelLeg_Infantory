#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

#include "can.h"

void CAN_Init(void);

uint8_t Can_TxMessage(CAN_HandleTypeDef *hcan,uint32_t id,uint8_t len,uint8_t *data);

#endif

