#include "wheel_kalman.h"

//INCLUDE部分
#include "Chassis_Task.h"
//全局变量定义部分
KalmanFilter_t V_kf;
Speed_Fusion_Parameter_t Speed_Fusion_Parameter;

float vaEstimateKF_K[4];
//状态转移矩阵
float vaEstimateKF_F[4] = { 1.0f, 0.0f,  
                            0.0f, 1.0f };	    
//后验估计协方差初始值
float vaEstimateKF_P[4] = { 1.0f, 0.0f,
                            0.0f, 1.0f };       
//过程噪声Q(越小越预测)
float vaEstimateKF_Q[4] = {  0.1f,     0.0f, 
                             0.0f,  2000.0f };       
//测量噪声R(越大越预测)
float vaEstimateKF_R[4] = { 100.0f,    0.0f, 
                              0.0f,    1.0f }; 					
// 设置矩阵H为常量
const float vaEstimateKF_H[4] = { 1.0f, 0.0f,
                                  0.0f, 1.0f };	

//EXTERN部分
extern Flag_Bit_t Flag;
																	
void V_Kf_Init(KalmanFilter_t *EstimateKF)
{
	Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维

	memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
	memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
	memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
	memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}																 

//动态调整Q R
void adjustQR(KalmanFilter_t *EstimateKF,Speed_Fusion_Parameter_t *sp_fusion) 
{		
	if(Flag.slip_flag[0] == 1 || Flag.slip_flag[1] == 1) { sp_fusion->Change_QR_Flag = 1; }
	else { sp_fusion->Change_QR_Flag = 0; }
	
	if(sp_fusion->Change_QR_Flag == 1)
	{
		EstimateKF->Q_data[0] =    0.01f;
		EstimateKF->Q_data[3] =  4000.0f;
		EstimateKF->R_data[0] =  9000.0f;
		EstimateKF->R_data[3] =   100.0f;
		EstimateKF->MeasuredVector[0] = sp_fusion->filter_vel;
		EstimateKF->MeasuredVector[1] = sp_fusion->actual_acc;
	}
	else if(sp_fusion->Change_QR_Flag == 0)
	{
		EstimateKF->Q_data[0] =    0.1f;
		EstimateKF->Q_data[3] = 2000.0f;
		EstimateKF->R_data[0] =  100.0f;
		EstimateKF->R_data[3] =    1.0f;
		EstimateKF->MeasuredVector[0] =	sp_fusion->actual_vel;
		EstimateKF->MeasuredVector[1] = sp_fusion->actual_acc;
	}
}

void V_Kf_Update(KalmanFilter_t *EstimateKF,Speed_Fusion_Parameter_t *sp_fusion,float dt)
{	
	//传入dt
	EstimateKF->F_data[1] = dt;
	
	//动态调整Q R
	adjustQR(EstimateKF,sp_fusion);
	
	Kalman_Filter_Update(EstimateKF);
	
	sp_fusion->filter_vel = EstimateKF->FilteredValue[0];
	sp_fusion->filter_acc = EstimateKF->FilteredValue[1];
}																 
