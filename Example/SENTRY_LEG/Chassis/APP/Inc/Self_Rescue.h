#ifndef __SELF_RESCUE_H
#define __SELF_RESCUE_H

#include "main.h"

//INCLUDE部分
#include "stdbool.h"

typedef struct
{
	int step[2][3];
	bool run_flag; //启动标志
	bool col_flag[2]; //收腿标志
	
	float init_pos[2]; //初始位置
	float goal_pos[2]; //目标位置
	
	unsigned short int run_first;    //先收哪条腿[0左 1右]
	unsigned short int downed_state; //倒地状态[0车身正 1车身倒头着地 2屁股着地]
}Self_Rescue_t;

//EXTERN部分
extern Self_Rescue_t Self_Rescue; 

#endif


