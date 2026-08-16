#ifndef __CHECK_TASK_H
#define __CHECK_TASK_H

#include "main.h"

//INCLUDE部分
#include "vofa.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "Can_Feedback.h"
#include "Remote_Control.h"
#include "Some_Functions.h"

//超时时间定义
#define vt_unlink_t            20
#define rc_unlink_t            20
#define dm_unlink_t            20
#define dji_unlink_t           20
#define cubermars_unlink_t     20

typedef enum
{
	UNLINK,
	DT7,
	VT_03
}Remote_Select_t; //遥控器选择

typedef struct
{
	bool tv; //图传链路
	bool rc;
	bool shoot[3];  //发射机构
	bool gimbal[3]; //云台
	
	uint16_t err_num; //报错码(4位16进制数)
}Link_Sit_t; //车体各部分外设连接情况结构体(0为在线 1为离线)

typedef enum
{
	ERO,
	STOP,
	RC,
	MOUSE,
	AUTO
}Controlled_State_t; //整车控制状态

void Check_Init(void);
void Check_Task(void);
void Check_Control(Link_Sit_t *link,Controlled_State_t *cs);
void Check_Peripheral_Link(Link_Sit_t *link,Gimbal_Motor_t *cm);

//EXTERN部分
extern bool all_ready_flag;

extern Link_Sit_t Link_Sit;
extern Remote_Select_t Remote_Select;
extern Controlled_State_t Controlled_State;

#endif
