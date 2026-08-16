#ifndef __DAMIAO_MOTOR_H
#define __DAMIAO_MOTOR_H

#include "main.h"

//INCLUDE部分
#include "bsp_can.h"

#define DM4310 10
#define DM4340 20
#define DM8009 30

/*DM_4340*/
#define P_MIN_4340   -3.1451926f
#define P_MAX_4340    3.1415926f
#define V_MIN_4340  -10.0f
#define V_MAX_4340   10.0f 
#define KP_MIN_4340   0.0f
#define KP_MAX_4340 500.0f
#define KD_MIN_4340   0.0f
#define KD_MAX_4340   5.0f
#define T_MIN_4340  -28.0f
#define T_MAX_4340   28.0f

/*DM_4310*/
#define P_MIN_4310   -3.1451926f
#define P_MAX_4310    3.1415926f
#define V_MIN_4310  -30.0f
#define V_MAX_4310   30.0f 
#define KP_MIN_4310   0.0f
#define KP_MAX_4310 500.0f
#define KD_MIN_4310   0.0f
#define KD_MAX_4310   5.0f
#define T_MIN_4310  -10.0f
#define T_MAX_4310   10.0f

/*DM_8009*/
#define P_MIN_8009  -3.141593f
#define P_MAX_8009   3.141593f
#define V_MIN_8009  -45.0f
#define V_MAX_8009   45.0f
#define KP_MIN_8009   0.0f
#define KP_MAX_8009 500.0f
#define KD_MIN_8009   0.0f
#define KD_MAX_8009   5.0f
#define T_MIN_8009  -54.0f
#define T_MAX_8009   54.0f

// 电机回传信息结构体
typedef struct 
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
	int dm_link[3];//连接检测数组
}motor_fbpara_t;

void Clear_Err(CAN_HandleTypeDef *hcan, uint32_t motor_id);
void Save_Pos_Zero(CAN_HandleTypeDef *hcan, uint32_t motor_id);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void Enable_Motor_Mode(CAN_HandleTypeDef *hcan, uint32_t motor_id);
void Disable_Motor_Mode(CAN_HandleTypeDef *hcan, uint32_t motor_id);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void Receive_Motor_Data(motor_fbpara_t *motor_fbpara, uint8_t *rx_data,uint8_t model);
void Mit_Ctrl(CAN_HandleTypeDef* hcan, uint32_t motor_id, float pos, float vel,float kp, float kd, float torq,uint8_t model);

#endif
