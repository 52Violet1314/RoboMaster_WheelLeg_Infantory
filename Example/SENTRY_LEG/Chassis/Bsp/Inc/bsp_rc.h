#ifndef __BSP_RC_H
#define __BSP_RC_H

#include "main.h"

void RC_Unable(void);
void RC_Restart(uint16_t dma_buf_num);
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
