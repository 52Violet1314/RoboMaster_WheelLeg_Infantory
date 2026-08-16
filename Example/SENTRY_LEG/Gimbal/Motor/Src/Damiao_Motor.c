#include "Damiao_Motor.h"

/**
************************************************************************
* @brief:      	Enable_Motor_Mode: 启用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void Enable_Motor_Mode(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
	uint8_t data[8];
	uint32_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	Can_TxMessage(hcan,id,8,data);
}

/**
************************************************************************
* @brief:      	Disable_Motor_Mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void Disable_Motor_Mode(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
	uint8_t data[8];
	uint32_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	Can_TxMessage(hcan,id,8,data);
}

/**
************************************************************************
* @brief:      	Save_Pos_Zero: 保存位置零点函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要保存位置零点的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送保存位置零点的命令
************************************************************************
**/
void Save_Pos_Zero(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
	uint8_t data[8];
	uint32_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	Can_TxMessage(hcan,id,8,data);
}

/**
************************************************************************
* @brief:      	clear_err: 清除电机错误函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要清除错误的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送清除错误的命令。
************************************************************************
**/
void Clear_Err(CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
	uint8_t data[8];
	uint32_t id = motor_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	Can_TxMessage(hcan,id,8,data);
}

/**
************************************************************************
* @brief:      	Mit_Ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @param[in]:   model:		电机型号
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void Mit_Ctrl(CAN_HandleTypeDef* hcan, uint32_t motor_id, float pos, float vel,float kp, float kd, float torq,uint8_t model)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	
	uint32_t id = motor_id;
	
	if(model == DM4310)
	{
		pos_tmp = float_to_uint(pos,  P_MIN_4310,  P_MAX_4310,  16);
    vel_tmp = float_to_uint(vel,  V_MIN_4310,  V_MAX_4310,  12);
		kp_tmp  = float_to_uint(kp,   KP_MIN_4310, KP_MAX_4310, 12);
		kd_tmp  = float_to_uint(kd,   KD_MIN_4310, KD_MAX_4310, 12);
		tor_tmp = float_to_uint(torq, T_MIN_4310,  T_MAX_4310,  12);
	}
	else if(model == DM4340)
	{
		pos_tmp = float_to_uint(pos,  P_MIN_4340,  P_MAX_4340,  16);
		vel_tmp = float_to_uint(vel,  V_MIN_4340,  V_MAX_4340,  12);
		kp_tmp  = float_to_uint(kp,   KP_MIN_4340, KP_MAX_4340, 12);
		kd_tmp  = float_to_uint(kd,   KD_MIN_4340, KD_MAX_4340, 12);
		tor_tmp = float_to_uint(torq, T_MIN_4340,  T_MAX_4340,  12);
	}
	else if(model == DM8009)
	{
		pos_tmp = float_to_uint(pos,  P_MIN_8009,  P_MAX_8009,  16);
		vel_tmp = float_to_uint(vel,  V_MIN_8009,  V_MAX_8009,  12);
		kp_tmp  = float_to_uint(kp,   KP_MIN_8009, KP_MAX_8009, 12);
		kd_tmp  = float_to_uint(kd,   KD_MIN_8009, KD_MAX_8009, 12);
		tor_tmp = float_to_uint(torq, T_MIN_8009,  T_MAX_8009,  12);
	}

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	Can_TxMessage(hcan,id,8,data);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:      	receive_motor_data：接收电机返回的数据信息
* @param:      	motor_t：电机参数结构体
* @param:      	data：接收的数据
* @param[in]:   model：电机型号
* @retval:     	void
* @details:    	逐次接收电机回传的参数信息
************************************************************************
**/
void Receive_Motor_Data(motor_fbpara_t *motor_fbpara, uint8_t *rx_data,uint8_t model)
{
	motor_fbpara->id    = (rx_data[0])&0x0F;
	motor_fbpara->state = (rx_data[0])>>4;
	motor_fbpara->p_int = (rx_data[1]<<8)|rx_data[2];
	motor_fbpara->v_int = (rx_data[3]<<4)|(rx_data[4]>>4);
	motor_fbpara->t_int =((rx_data[4]&0xF)<<8)|rx_data[5];
	
	if(model == DM4310)
	{
		motor_fbpara->pos = uint_to_float(motor_fbpara->p_int, P_MIN_4310, P_MAX_4310, 16);
		motor_fbpara->vel = uint_to_float(motor_fbpara->v_int, V_MIN_4310, V_MAX_4310, 12);
		motor_fbpara->tor = uint_to_float(motor_fbpara->t_int, T_MIN_4310, T_MAX_4310, 12);
	}
	else if(model == DM4340)
	{
		motor_fbpara->pos = uint_to_float(motor_fbpara->p_int, P_MIN_4340, P_MAX_4340, 16);
		motor_fbpara->vel = uint_to_float(motor_fbpara->v_int, V_MIN_4340, V_MAX_4340, 12);
		motor_fbpara->tor = uint_to_float(motor_fbpara->t_int, T_MIN_4340, T_MAX_4340, 12);
	}
	else if(model == DM8009)
	{
		motor_fbpara->pos = uint_to_float(motor_fbpara->p_int, P_MIN_8009, P_MAX_8009, 16);
		motor_fbpara->vel = uint_to_float(motor_fbpara->v_int, V_MIN_8009, V_MAX_8009, 12);
		motor_fbpara->tor = uint_to_float(motor_fbpara->t_int, T_MIN_8009, T_MAX_8009, 12);
	}
	
	motor_fbpara->Tmos  = (float)(rx_data[6]);
	motor_fbpara->Tcoil = (float)(rx_data[7]);
	
	//判断电机是否离线
	if(motor_fbpara->dm_link[0]>=1000) motor_fbpara->dm_link[0]=0;
	else motor_fbpara->dm_link[0]++;
}



