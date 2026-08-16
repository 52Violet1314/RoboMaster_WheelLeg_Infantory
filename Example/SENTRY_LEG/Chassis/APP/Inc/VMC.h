#ifndef __VMC_H
#define __VMC_H

#include "main.h"

//INCLUDE部分
#include "arm_math.h"
#include "Some_Functions.h"

//大腿长度(1 4)
#define Thigh_L  0.210f //单位 m
//小腿长度(2 3)
#define Calf_L   0.248f //单位 m

typedef struct
{
	float F0;        //腿部关节转矩以及向下的支持力
	float Tp;
	float T_Wheel;   //轮毅电机输出扭矩及电流的目标值
	float A;
	float T_A;       //关节电机输出扭矩(逆时针为正)
	float T_E;
}
Joint_Motor_Status_t;

typedef struct
{
	float phi0;          //角度
	float phi1;
	float phi2;
	float phi3;
	float phi4;

	float d_phi0;        //角加速度
	float d_phi1;
	float d_phi4;

	float L0;            //腿长以及腿长加速度
	float d_L0;
	
	float alpha[2];      //用于计算气弹簧分解力的角度
}
Vmc_Five_Link_Parameter_t;

void Leg_Calc(Vmc_Five_Link_Parameter_t *five_link);
void VMC_Calc(Vmc_Five_Link_Parameter_t *five_link,float F0,float Tp,float *Tp1,float *Tp2);

//EXTERN部分
extern Joint_Motor_Status_t Joint_Motor_Status[2];        
extern Vmc_Five_Link_Parameter_t Five_Link_Parameter[2];  

#endif
