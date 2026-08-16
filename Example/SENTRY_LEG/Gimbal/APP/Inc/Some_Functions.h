#ifndef __SOME_FUNCTIONS_H
#define __SOME_FUNCTIONS_H

#include "main.h"

//INCLUDE²¿·Ö
#include "stdbool.h"
#include "arm_math.h"

int Whole_Circle_ANGLE(int num);
float Half_Circle_ANGLE(float num);
float Half_Circle_RADIAN(float num);
float Whole_Circle_RADIAN(float num);
float Positive_Number_Out(float num);
float Max_Output(float num,float max);
float Min_Output(float num,float min);
int16_t Limit_Min(int16_t num,int min);
float Mean_Filtering(float input,float *aver);
bool Check_If_Unchange(int *num,int time_thre);
float Find_Min_Angle(float measure, float ref);
float Find_Min_RADIAN(float measure, float ref);
float Ramp_Function(float ref,float *out,float sens);

#endif
