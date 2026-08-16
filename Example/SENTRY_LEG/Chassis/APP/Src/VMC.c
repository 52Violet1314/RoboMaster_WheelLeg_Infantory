#include "VMC.h"

/*

   ___x
  |      A_____E  
  |y   1 /     \ 4    / \
        /       \    / | \
 			 B\ 			/D     |    F0向上为正
 				 \     /       |
        2 \   / 3      |
           \ /
            C
     ----->   逆时针Tp为正
*/


//全局变量定义部分(左腿[0] 右腿[1])
Joint_Motor_Status_t Joint_Motor_Status[2]; //关节电机输出参数
Vmc_Five_Link_Parameter_t Five_Link_Parameter[2]; //五连杆参数

/*******************************************************************************************************
腿部数据获取(注:此部分解算去除了Joint长度)
********************************************************************************************************/
void Leg_Calc(Vmc_Five_Link_Parameter_t *five_link)
{
	static float d_x_b = 0;
	static float d_y_b = 0;

	static float A_0 = 0;   static float A_1 = 0;
	static float B_0 = 0;
	static float C_0 = 0;
	
	static float x_db = 0; static float x_c = 0;
	static float y_db = 0; static float y_c = 0;
	
	x_db = Thigh_L*arm_cos_f32(five_link->phi4) - Thigh_L*arm_cos_f32(five_link->phi1);
	y_db = Thigh_L*arm_sin_f32(five_link->phi4) - Thigh_L*arm_sin_f32(five_link->phi1);
	
	A_0 = 2.0f*Calf_L*x_db;
	B_0 = 2.0f*Calf_L*y_db;
	C_0 = x_db*x_db + y_db*y_db;
	
	five_link->phi2 = 2.0f*atan2f(B_0 + sqrtf(A_0*A_0 + B_0*B_0 - C_0*C_0),A_0 + C_0);
	
	x_c = Thigh_L*arm_cos_f32(five_link->phi1) + Calf_L*arm_cos_f32(five_link->phi2);
	y_c = Thigh_L*arm_sin_f32(five_link->phi1) + Calf_L*arm_sin_f32(five_link->phi2);
	
	five_link->L0   = sqrtf(x_c*x_c + y_c*y_c);
	five_link->phi0 = atan2f(y_c,x_c);
	five_link->phi3 = atan2f(-y_db + Calf_L*arm_sin_f32(five_link->phi2),-x_db + Calf_L*arm_cos_f32(five_link->phi2));

	A_1   = (Thigh_L*five_link->d_phi1*arm_sin_f32(five_link->phi1 - five_link->phi3) + Thigh_L*five_link->d_phi4*arm_sin_f32(five_link->phi3 - five_link->phi4))/arm_sin_f32(five_link->phi3 - five_link->phi2);
	d_x_b = -Thigh_L*five_link->d_phi1*arm_sin_f32(five_link->phi1);
	d_y_b =  Thigh_L*five_link->d_phi1*arm_cos_f32(five_link->phi1);
	
	five_link->d_L0   = (y_c*(d_y_b + A_1*arm_cos_f32(five_link->phi2)) + x_c*(d_x_b - A_1*arm_sin_f32(five_link->phi2)))/(five_link->L0);
	five_link->d_phi0 = (x_c*(d_y_b + A_1*arm_cos_f32(five_link->phi2)) - y_c*(d_x_b - A_1*arm_sin_f32(five_link->phi2)))/(five_link->L0*five_link->L0);
	
	five_link->alpha[0] = acos((Thigh_L*Thigh_L + Calf_L*Calf_L - five_link->L0*five_link->L0)/(2.0f*Thigh_L*Calf_L));
	five_link->alpha[1] = five_link->phi1 - five_link->phi4; 
	if(five_link->alpha[1]>2*PI) {five_link->alpha[1] -= 2*PI;}
}

/*******************************************************************************************************
五连杆虚拟力正逆解
********************************************************************************************************/
void VMC_Calc(Vmc_Five_Link_Parameter_t *five_link,float F0,float Tp,float *Tp1,float *Tp2) //四连杆特化
{
	static float cos_beta;
	static float sin_beta;
	static float tan_beta;
	
	static float T1,T2;
	
 	cos_beta = (five_link->L0*five_link->L0 + Calf_L*Calf_L - Thigh_L*Thigh_L)/(2.0f*five_link->L0*Calf_L);
	sin_beta = sqrtf(1 - cos_beta*cos_beta);
	tan_beta = sin_beta/cos_beta;
	
	T1 = 0.5f*F0*five_link->L0*tan_beta;
	T2 = 0.5f*Tp;
	
	*Tp1 = -T1 + T2;
	*Tp2 =  T1 + T2;
}

//void VMC_Calc(Vmc_Five_Link_Parameter_t *five_link,float F0,float Tp,float *Tp1,float *Tp2) //传统五连杆VMC
//{
//	static float phi12 = 0;
//	static float phi32 = 0;
//	static float phi34 = 0;
//	static float A_Matrix[2][2];
//	
//	phi12 = arm_sin_f32(five_link->phi1 - five_link->phi2);
//	phi32 = arm_sin_f32(five_link->phi3 - five_link->phi2);
//	phi34 = arm_sin_f32(five_link->phi3 - five_link->phi4);
//	
//	A_Matrix[0][0] = (Thigh_L*arm_sin_f32(five_link->phi0 - five_link->phi3)*phi12)/phi32;
//	A_Matrix[0][1] = (Thigh_L*arm_cos_f32(five_link->phi0 - five_link->phi3)*phi12)/(five_link->L0*phi32);
//	A_Matrix[1][0] = (Thigh_L*arm_sin_f32(five_link->phi0 - five_link->phi2)*phi34)/phi32;
//	A_Matrix[1][1] = (Thigh_L*arm_cos_f32(five_link->phi0 - five_link->phi2)*phi34)/(five_link->L0*phi32);
//	
//	*Tp1 = A_Matrix[0][0]*F0 + A_Matrix[0][1]*Tp;
//	*Tp2 = A_Matrix[1][0]*F0 + A_Matrix[1][1]*Tp;
//}

/*******************************************************************************************************
五连杆逆运动学解算
********************************************************************************************************/
void Inverse_Kinematics_Calc(float L0,float phi0,float *Inverse_phi1,float *Inverse_phi4)
{
	static float A_1 = 0;
	static float B_1 = 0;
	static float C_1 = 0;
	static float D_1 = 0;
	
	static float A_2 = 0;
	static float B_2 = 0;
	static float C_2 = 0;
	static float D_2 = 0;
	static float E_2 = 0;
	static float F_2 = 0;

	static float x_c = 0;
	static float y_c = 0;
	
	x_c = L0*arm_cos_f32(phi0);
	y_c = L0*arm_sin_f32(phi0);
	
	A_1 = Thigh_L + x_c;
	B_1 = Thigh_L*Thigh_L - x_c*x_c;
	C_1 = Calf_L*Calf_L - y_c*y_c;
	D_1 = Calf_L*Calf_L + y_c*y_c;
	
	*Inverse_phi1 = 2*atan2f(2*Thigh_L*y_c + sqrtf(2*Thigh_L*Thigh_L*D_1 + 2*x_c*x_c*C_1 - B_1*B_1 - C_1*C_1),A_1*A_1 - C_1);
	
	A_2 = Calf_L + Thigh_L;
	B_2 = -x_c;
	C_2 = Calf_L - Thigh_L;
	D_2 = x_c + Thigh_L;
	E_2 = A_2*A_2 - B_2*B_2 - y_c*y_c;
	F_2 = B_2*B_2 - C_2*C_2 + y_c*y_c;
	
	*Inverse_phi4 = Half_Circle_RADIAN(2*atan2f(2*Thigh_L*y_c - sqrtf(E_2*F_2),D_2*D_2 + y_c*y_c - Calf_L*Calf_L));
}




