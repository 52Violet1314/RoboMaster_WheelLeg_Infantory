#ifndef __APP_MOTOR_H__
#define __APP_MOTOR_H__

#include "bsp_fdcan.h"
#include "fdcan.h"
#include <stdint.h>

#define DM_MOTOR_MODE_MIT 0x51
#define DM_MOTOR_MODE_POS_SPEED 0x52
#define DM_MOTOR_MODE_SPEED 0x53

#define P_MAX_RAD_8009 3.14f
#define V_MAX_RAD_S_8009 16.755f
#define T_MAX_NM_8009 40.0f

#define P_MAX_RAD_3519 12.5f
#define V_MAX_RAD_S_3519 45.553f
#define T_MAX_NM_3519 7.8f

#define P_MAX_RAD_3507 12.566f
#define V_MAX_RAD_S_3507 20.944f
#define T_MAX_NM_3507 7.0f

#define P_MAX_RAD_4310 12.5f
#define V_MAX_RAD_S_4310 30.0f
#define T_MAX_NM_4310 10.0f
#define OUTPUT_V_MAX_RAD_S_4310 20.944f
#define OUTPUT_T_MAX_NM_4310 7.0f

typedef struct {
  uint8_t err;
  float position_rad;
  float velocity_rad_s;
  float torque_Nm;
  float temp_MOS_C;
  float temp_COIL_C;
} DM_Motor_Data_t;

typedef enum {
  DM_MOTOR_MODEL_8009 = 0,
  DM_MOTOR_MODEL_3519,
  DM_MOTOR_MODEL_3507,
  DM_MOTOR_MODEL_J4310
} DM_Motor_Model_t;

typedef struct {
  uint16_t ID;
  uint8_t MODE;
  uint8_t Enable_Flag;
  DM_Motor_Model_t model;
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

#ifdef __cplusplus
extern "C" {
#endif

void DM_Motor_Init(DM_Motor_t *motor, DM_Motor_Model_t model,
                   uint16_t id, uint8_t mode,
                   FDCAN_HandleTypeDef *hcan);
void DM_Motor_Enable(DM_Motor_t *motor);
void DM_Motor_MIT_Control(DM_Motor_t *motor, float Kp_des, float Kd_des,
                          float Pos_Des, float Spd_Des, float Tor_Des);
void DM_Motor_Pos_Speed_Control(DM_Motor_t *motor, float Pos_Des,
                                float Spd_Des);
void DM_Motor_Speed_Control(DM_Motor_t *motor, float Spd_Des);
void DM_Motor_FeedbackDecode(DM_Motor_t *motor, const uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif
