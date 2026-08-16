#ifndef __LIMIT_TASK_H
#define __LIMIT_TASK_H

#include "main.h"

//INCLUDE部分
#include "stdbool.h"

#define HEAT_17MM  10.0f //17mm弹丸热量
#define HEAT_42MM 100.0f //42mm弹丸热量

typedef struct //Q0:射击热量上限 Q1:当前射击热量
{	 
	float pin_dt;      //延迟时间计算
	float heat_dt;     //本地热量计算计时
	
	uint32_t dwt_pin;  //计算DT
	uint32_t dwt_heat; 
	
	uint16_t cooling_value;  //射击热量每秒冷却值
	
	uint16_t Air_Q0; //机载端数据
	uint16_t Air_Q1;
	uint16_t Last_Air_Q1;
	
	float Local_Q1; //本地计算数据
	
	int bullet_num; //剩余热量的允许发弹量
}Heat_Control_t;

void Limit_Init(void);
void Limit_Task(void);
void Heat_Limit_Control(Heat_Control_t *hc);

//EXTERN部分
extern bool limit_ready_flag;

extern Heat_Control_t Heat_Control;

#endif

