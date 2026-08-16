#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#include "main.h"

//INCLUDE部分
#include "bsp_can.h"
#include "stdbool.h"

typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	int dji_link[3];//连接检测数组
} motor_measure_t;

void Get_Motor_Measure(motor_measure_t *motor_measure, uint8_t *rx_data);
void Dji_Motor_Ctrl(CAN_HandleTypeDef* hcan,uint32_t id,int16_t one,int16_t two,int16_t tree,int16_t four);

#endif
