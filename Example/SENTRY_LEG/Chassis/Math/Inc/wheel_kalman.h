#ifndef __WHEEL_KALMAN_H
#define __WHEEL_KALMAN_H

//INCLUDE部分
#include "stdbool.h"
#include "kalman_filter.h"

typedef struct
{
	float actual_vel;      //车体实际速度
	float actual_acc;      //车体实际加速度
	float filter_vel;      //滤波后的速度
	float filter_acc;      //滤波后的加速度
	
	bool Change_QR_Flag;
	uint32_t dwt_v_kf;     //速度与加速度融合滤波的时间微分量
} 
Speed_Fusion_Parameter_t;

void V_Kf_Init(KalmanFilter_t *EstimateKF);
void adjustQR(KalmanFilter_t *EstimateKF,Speed_Fusion_Parameter_t *sp_fusion);
void V_Kf_Update(KalmanFilter_t *EstimateKF,Speed_Fusion_Parameter_t *sp_fusion,float dt);													 											 

//EXTERN部分
extern KalmanFilter_t V_kf;
extern Speed_Fusion_Parameter_t Speed_Fusion_Parameter;

#endif 



