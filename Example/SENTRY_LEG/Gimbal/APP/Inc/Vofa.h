#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"

//INCLUDE²¿·Ö
#include "usbd_cdc_if.h"

typedef union
{
	float v_f;
	uint8_t v_u8[4];
}TypedefVofa;

void Vofa_Init(void);
void Vofa_Send_Message(void);

#endif


