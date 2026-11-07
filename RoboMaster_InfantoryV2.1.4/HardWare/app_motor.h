#ifndef __APP_MOTOR_H__
#define __APP_MOTOR_H__

#include "bsp_fdcan.h"
#include "fdcan.h"
#include <stdint.h>

/* MIT 映射范围必须与电调 PMAX/VMAX/TMAX 寄存器一致；
 * 铭牌额定/峰值能力不是 MIT 字段量程，不能直接替代。 */
/* 当前值须由已配置电调的寄存器读回值确认。 */
#define P_MAX_RAD_4310 3.14f
#define V_MAX_RAD_S_4310 20.944f
#define T_MAX_NM_4310 7.0f
#define P_MAX_8009 3.14f
#define V_MAX_RAD_S_8009 16.755f
#define T_MAX_NM_8009 40.0f
#define P_MAX_RAD_3519 12.5f
#define V_MAX_RAD_S_3519 45.553f
#define T_MAX_NM_3519 7.8f
#define P_MAX_RAD_3507 12.566f
#define V_MAX_RAD_S_3507 20.944f
#define T_MAX_NM_3507 7.0f

#define DM_MOTOR_MODE_MIT 0x51
#define DM_MOTOR_MODE_POS_SPEED 0x52
#define DM_MOTOR_MODE_SPEED 0x53

typedef struct {
  uint8_t err;
  float position_rad;
  float velocity_rad_s;
  float torque_Nm;
  float temp_MOS_C;
  float temp_COIL_C;
} DM_Motor_Data_t;

typedef struct {
  uint16_t ID;
  uint8_t MODE;
  uint8_t Enable_Flag;
  FDCAN_HandleTypeDef *hcan;
  DM_Motor_Data_t data;
} DM_Motor_t;

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

#ifdef __cplusplus
extern "C" {
#endif

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
void DM_Motor_3507_Data_Get(DM_Motor_t *motor, uint8_t *data);
void DM_Motor_4310_Data_Get(DM_Motor_t *motor, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
