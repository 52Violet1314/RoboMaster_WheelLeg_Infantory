#ifndef __APP_MOTOR_H__
#define __APP_MOTOR_H__

#include "bsp_fdcan.h"
#include "fdcan.h"
#include <stdint.h>

#define P_MAX_RAD       3.14f     
#define V_MAX_RAD_S     30.0f   
#define T_MAX_NM        10.0f   

#define P_MAX_8009     3.50f
#define V_MAX_RAD_S_8009    45.0f
#define T_MAX_NM_8009      54.0f   

#define P_MAX_RAD_3519       12.5f   
#define V_MAX_RAD_S_3519     200.0f   
#define T_MAX_NM_3519        10.0f   

#define P_MAX_RAD_3507       12.566f   
#define V_MAX_RAD_S_3507     50.0f   
#define T_MAX_NM_3507        5.0f   

#define DM_MOTOR_MODE_MIT 0x51
#define DM_MOTOR_MODE_POS_SPEED 0x52
#define DM_MOTOR_MODE_SPEED 0x53

#define DJI_MOTOR_TYPE_M3508 0x01
#define DJI_MOTOR_TYPE_M2006 0x02
#define DJI_MOTOR_TYPE_M6020 0x03

typedef struct {
  uint8_t err; /* 0=正常，见说明书 err 表 8~E */
  float position_rad;
  float velocity_rad_s;
  float torque_Nm;
  float temp_MOS_C;
  float temp_COIL_C;
} DM_Motor_Data_t;

// DM电机结构体
typedef struct {
  uint16_t ID;
  uint8_t MODE;
  FDCAN_HandleTypeDef *hcan;
  DM_Motor_Data_t data;
} DM_Motor_t;

// DJI电机结构体
typedef struct {
  FDCAN_HandleTypeDef *hcan;
} DJi_Motor_t;

extern DM_Motor_t DM_8009P1;
extern DM_Motor_t DM_8009P2;
extern DM_Motor_t DM_8009P3;
extern DM_Motor_t DM_8009P4;
extern DM_Motor_t DM_3519L;
extern DM_Motor_t DM_3519R;
extern DM_Motor_t DM_3507;
extern DM_Motor_t DM_4310;
extern DM_Motor_t DM_3519_Shoot_L;
extern DM_Motor_t DM_3519_Shoot_R;
extern DM_Motor_t DM_3519_Gimbal_Pitch;

// 函数声明
#ifdef __cplusplus
extern "C" {
#endif

// DM电机函数
void DM_Motor_Init(DM_Motor_t *motor, uint16_t id, uint8_t mode,
                   FDCAN_HandleTypeDef *hcan);
void DM_Motor_Enable(DM_Motor_t *motor);
void DM_Motor_MIT_Control(DM_Motor_t *motor, float Kp_des, float Kd_des,
                          float Pos_Des, float Spd_Des, float Tor_Des);
void DM_Motor_Pos_Speed_Control(DM_Motor_t *motor, float Pos_Des,
                                float Spd_Des);
void DM_Motor_Speed_Control(DM_Motor_t *motor, float Spd_Des);

void DM_Motor_8009_Data_Get(DM_Motor_t *motor, uint8_t *data);

void DM_Motor_3519_Data_Get(DM_Motor_t *motor, uint8_t *data);

// DJI电机函数
void DJi_Motor_Init(DJi_Motor_t *motor, FDCAN_HandleTypeDef *hcan);
void DJi_Motor_Current_Control(DJi_Motor_t *motor, uint8_t Motor_Type,
                               uint8_t id_range, int16_t Current1,
                               int16_t Current2, int16_t Current3,
                               int16_t Current4);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MOTOR_H__ */
