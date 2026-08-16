#ifndef __VT03_H
#define __VT03_H

#include "main.h"

//INCLUDE部分
#include "stdbool.h"

#define VT03_RX_BUF_NUM   36u

#define VT03_FRAME_LENGTH  21u

typedef __packed struct
{
	uint8_t sof_1;
	uint8_t sof_2;
	uint64_t ch_0:11;
	uint64_t ch_1:11;
	uint64_t ch_2:11;
	uint64_t ch_3:11;
	uint64_t mode_sw:2;
	uint64_t pause:1;
	uint64_t fn_1:1;
	uint64_t fn_2:1;
	uint64_t wheel:11;
	uint64_t trigger:1;

	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_left:2;
	uint8_t mouse_right:2;
	uint8_t mouse_middle:2;
	uint16_t key;
	uint16_t crc16;
}remote_data_t;

void VT03_Init(void);
void RX_TO_VT03(void);

//EXTERN部分
extern int vt_link[3];
extern remote_data_t VT03;

#endif


