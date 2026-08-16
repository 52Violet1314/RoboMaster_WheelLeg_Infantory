#ifndef __CAN_FEEDBACK_H
#define __CAN_FEEDBACK_H

#include "main.h"

//INCLUDE部分
#include "can.h"
#include "VT03.h"
#include "Super_Cap.h"
#include "Dji_Motor.h"
#include "Damiao_Motor.h"
#include "Remote_Control.h"

typedef struct
{
	motor_fbpara_t DM_8009[4];
	motor_measure_t Dji_3508[2];
}Chassis_Motor_t;

typedef struct
{
	int Friction_Speed; //摩擦轮转速
	
	bool Auto_Aim_Flag;    //自瞄锁定标志位
	bool Friction_Status;  //摩擦轮状态
	
	uint16_t up_err_num;   //上板报错码
	
	float ecd_yaw_6020;    //YAW6020的转子位置
}Up_Cboard_Info_t;

//EXTERN部分
extern Chassis_Motor_t Chassis_Motor;
extern Up_Cboard_Info_t Up_Cboard_Info;

#endif
