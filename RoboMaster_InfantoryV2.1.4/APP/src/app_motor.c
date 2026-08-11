#include "app_motor.h"
#include "CalculateTask.hpp"

DM_Motor_t DM_8009P1;
DM_Motor_t DM_8009P2;
DM_Motor_t DM_8009P3;
DM_Motor_t DM_8009P4;

DM_Motor_t DM_3519L;
DM_Motor_t DM_3519R;

DM_Motor_t DM_3519_Shoot_L;
DM_Motor_t DM_3519_Shoot_R;

DM_Motor_t DM_3519_Gimbal_Pitch;

DM_Motor_t DM_3507;

DM_Motor_t DM_4310;

static int16_t float_to_int16(float x, float x_min, float x_max, uint8_t bits);
static float uint16_to_float(int16_t x, float x_min, float x_max, uint8_t bits);
static int16_t float_to_int12(float x, float x_min, float x_max);
static float int12_to_float(int16_t x, float x_min, float x_max);
static float AngleUnify(float angle);
static float RadUnify(float rad);

static int float_to_uint(float x, float x_min, float x_max, int bits) {
  // Converts a float to an unsigned int, given range and number of bits
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// DM电机初始化函数
void DM_Motor_Init(DM_Motor_t *motor, uint16_t id, uint8_t mode,
                   FDCAN_HandleTypeDef *hcan) {
  motor->ID = id;
  motor->MODE = mode;
  motor->hcan = hcan;
  DM_Motor_Enable(motor);
}

// DM电机使能函数
void DM_Motor_Enable(DM_Motor_t *motor) {
  static const uint8_t enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  uint16_t motor_id;

  switch (motor->MODE) {
  case DM_MOTOR_MODE_MIT:
    motor_id = motor->ID;
    break;
  case DM_MOTOR_MODE_POS_SPEED:
    motor_id = motor->ID | 0x100;
    break;
  case DM_MOTOR_MODE_SPEED:
    motor_id = motor->ID | 0x200;
    break;
  default:
    return;
  }
  fdcanx_send_data(motor->hcan, motor_id, (uint8_t *)enable_data, 8);
}

// MIT控制模式
void DM_Motor_MIT_Control(DM_Motor_t *motor, float Kp_des, float Kd_des,
                          float Pos_Des, float Spd_Des, float Tor_Des) {
  uint8_t motor_send_data[8];
  uint16_t Kp = float_to_uint(Kp_des, 0.0f, 500.0f, 12);
  uint16_t Kd = float_to_uint(Kd_des, 0.0f, 5.0f, 12);
  uint16_t Pos = float_to_uint(Pos_Des, -12.5f, 12.5f, 16);
  uint16_t Spd = float_to_uint(Spd_Des, -45.0f, 45.0f, 12);
  uint16_t Tor = float_to_uint(Tor_Des, -18.0f, 18.0f, 12);

  motor_send_data[0] = (uint8_t)(Pos >> 8);
  motor_send_data[1] = (uint8_t)Pos;
  motor_send_data[2] = (uint8_t)(Spd >> 4);
  motor_send_data[3] = (uint8_t)((Spd & 0xF) << 4) | (Kp >> 8);
  motor_send_data[4] = (uint8_t)Kp;
  motor_send_data[5] = (uint8_t)(Kd >> 4);
  motor_send_data[6] = (uint8_t)((Kd & 0xF) << 4) | (Tor >> 8);
  motor_send_data[7] = (uint8_t)Tor;
  fdcanx_send_data(motor->hcan, motor->ID, motor_send_data, 8);
}

// 速度控制模式
void DM_Motor_Speed_Control(DM_Motor_t *motor, float Spd_Des) {
  uint8_t *vbuf;
  uint8_t motor_send_data[4];
  vbuf = (uint8_t *)&Spd_Des;
  motor_send_data[0] = *vbuf;
  motor_send_data[1] = *(vbuf + 1);
  motor_send_data[2] = *(vbuf + 2);
  motor_send_data[3] = *(vbuf + 3);
  uint16_t Motor_ID = motor->ID | 0x200;
  fdcanx_send_data(motor->hcan, Motor_ID, motor_send_data, 4);
}

// 位置速度控制模式
void DM_Motor_Pos_Speed_Control(DM_Motor_t *motor, float Pos_Des,
                                float Spd_Des) {
  uint8_t motor_send_data[8];
  uint8_t *pbuf, *vbuf;
  pbuf = (uint8_t *)&Pos_Des;
  vbuf = (uint8_t *)&Spd_Des;
  motor_send_data[0] = *pbuf;
  motor_send_data[1] = *(pbuf + 1);
  motor_send_data[2] = *(pbuf + 2);
  motor_send_data[3] = *(pbuf + 3);
  motor_send_data[4] = *vbuf;
  motor_send_data[5] = *(vbuf + 1);
  motor_send_data[6] = *(vbuf + 2);
  motor_send_data[7] = *(vbuf + 3);
  uint16_t Motor_ID = motor->ID | 0x100;
  fdcanx_send_data(motor->hcan, Motor_ID, motor_send_data, 8);
}

void DM_Motor_8009_Data_Get(DM_Motor_t *motor, uint8_t *data) {
  motor->data.err = (data[0] >> 4) & 0x0F;
  int16_t pos_raw = (int16_t)((data[1] << 8) | data[2]);
  int16_t vel_raw = (int16_t)((data[3] << 4) | (data[4] >> 4));
  int16_t tor_raw = (int16_t)(((data[4] & 0x0F) << 8) | data[5]);

  motor->data.position_rad =
      uint16_to_float(pos_raw, -P_MAX_8009, P_MAX_8009, 16);
  motor->data.velocity_rad_s =
      int12_to_float(vel_raw, -V_MAX_RAD_S_8009, V_MAX_RAD_S_8009);
  motor->data.torque_Nm = int12_to_float(tor_raw, -T_MAX_NM_8009, T_MAX_NM_8009);
  motor->data.temp_MOS_C = (float)data[6];
  motor->data.temp_COIL_C = (float)data[7];
}

void DM_Motor_3519_Data_Get(DM_Motor_t *motor, uint8_t *data) {
  motor->data.err = (data[0] >> 4) & 0x0F;
  int16_t pos_raw = (int16_t)((data[1] << 8) | data[2]);
  int16_t vel_raw = (int16_t)((data[3] << 4) | (data[4] >> 4));
  int16_t tor_raw = (int16_t)(((data[4] & 0x0F) << 8) | data[5]);

  motor->data.position_rad =
      uint16_to_float(pos_raw, -P_MAX_RAD_3519, P_MAX_RAD_3519, 16);
  motor->data.velocity_rad_s =
      int12_to_float(vel_raw, -V_MAX_RAD_S_3519, V_MAX_RAD_S_3519);
  motor->data.torque_Nm = int12_to_float(tor_raw, -T_MAX_NM_3519, T_MAX_NM_3519);
  motor->data.temp_MOS_C = (float)data[6];
  motor->data.temp_COIL_C = (float)data[7];
}

// DJI电机初始化函数
void DJi_Motor_Init(DJi_Motor_t *motor, FDCAN_HandleTypeDef *hcan) {
  motor->hcan = hcan;
}

// DJI电机电流控制函数
void DJi_Motor_Current_Control(DJi_Motor_t *motor, uint8_t Motor_Type,
                               uint8_t id_range, int16_t Current1,
                               int16_t Current2, int16_t Current3,
                               int16_t Current4) {
  uint8_t TxBuf[8];

  switch (Motor_Type) {
  case DJI_MOTOR_TYPE_M3508:
    if (id_range == 0) {
      TxBuf[0] = (Current1 >> 8) & 0xff;
      TxBuf[1] = (Current1) & 0xff;
      TxBuf[2] = (Current2 >> 8) & 0xff;
      TxBuf[3] = (Current2) & 0xff;
      TxBuf[4] = (Current3 >> 8) & 0xff;
      TxBuf[5] = (Current3) & 0xff;
      TxBuf[6] = (Current4 >> 8) & 0xff;
      TxBuf[7] = (Current4) & 0xff;
      fdcanx_send_data(motor->hcan, 0x200, TxBuf, 8);
    } else {
      TxBuf[0] = (Current1 >> 8) & 0xff;
      TxBuf[1] = (Current1) & 0xff;
      TxBuf[2] = (Current2 >> 8) & 0xff;
      TxBuf[3] = (Current2) & 0xff;
      TxBuf[4] = (Current3 >> 8) & 0xff;
      TxBuf[5] = (Current3) & 0xff;
      TxBuf[6] = (Current4 >> 8) & 0xff;
      TxBuf[7] = (Current4) & 0xff;
      fdcanx_send_data(motor->hcan, 0x1FF, TxBuf, 8);
    }
    break;

  case DJI_MOTOR_TYPE_M2006:
    if (id_range == 0) {
      TxBuf[0] = (Current1 >> 8) & 0xff;
      TxBuf[1] = (Current1) & 0xff;
      TxBuf[2] = (Current2 >> 8) & 0xff;
      TxBuf[3] = (Current2) & 0xff;
      TxBuf[4] = (Current3 >> 8) & 0xff;
      TxBuf[5] = (Current3) & 0xff;
      TxBuf[6] = (Current4 >> 8) & 0xff;
      TxBuf[7] = (Current4) & 0xff;
      fdcanx_send_data(motor->hcan, 0x200, TxBuf, 8);
    } else {
      TxBuf[0] = (Current1 >> 8) & 0xff;
      TxBuf[1] = (Current1) & 0xff;
      TxBuf[2] = (Current2 >> 8) & 0xff;
      TxBuf[3] = (Current2) & 0xff;
      TxBuf[4] = (Current3 >> 8) & 0xff;
      TxBuf[5] = (Current3) & 0xff;
      TxBuf[6] = (Current4 >> 8) & 0xff;
      TxBuf[7] = (Current4) & 0xff;
      fdcanx_send_data(motor->hcan, 0x1FF, TxBuf, 8);
    }
    break;

  case DJI_MOTOR_TYPE_M6020:
    if (id_range == 0) {
      TxBuf[0] = (Current1 >> 8) & 0xff;
      TxBuf[1] = (Current1) & 0xff;
      TxBuf[2] = (Current2 >> 8) & 0xff;
      TxBuf[3] = (Current2) & 0xff;
      TxBuf[4] = (Current3 >> 8) & 0xff;
      TxBuf[5] = (Current3) & 0xff;
      TxBuf[6] = (Current4 >> 8) & 0xff;
      TxBuf[7] = (Current4) & 0xff;
      fdcanx_send_data(motor->hcan, 0x1FE, TxBuf, 8);
    } else {
      TxBuf[0] = (Current1 >> 8) & 0xff;
      TxBuf[1] = (Current1) & 0xff;
      TxBuf[2] = (Current2 >> 8) & 0xff;
      TxBuf[3] = (Current2) & 0xff;
      TxBuf[4] = (Current3 >> 8) & 0xff;
      TxBuf[5] = (Current3) & 0xff;
      TxBuf[6] = (Current4 >> 8) & 0xff;
      TxBuf[7] = (Current4) & 0xff;
      fdcanx_send_data(motor->hcan, 0x2FE, TxBuf, 8);
    }
    break;

  default:
    break;
  }
}

static int16_t float_to_int16(float x, float x_min, float x_max, uint8_t bits) {
  float span = x_max - x_min;
  float offset = x - x_min;
  if (offset < 0.0f)
    offset = 0.0f;
  if (offset > span)
    offset = span;
  return (int16_t)(offset / span * 65535);
}
static float uint16_to_float(int16_t x, float x_min, float x_max,
                             uint8_t bits) {
  float span = x_max - x_min;
  return x_min + span * (float)x / ((1 << bits) - 1);
}
static int16_t float_to_int12(float x, float x_min, float x_max) {
  float span = x_max - x_min;
  float offset = x - x_min;
  if (offset < 0.0f)
    offset = 0.0f;
  if (offset > span)
    offset = span;
  return (int16_t)(offset / span * 4095.0f);
}
static float int12_to_float(int16_t x, float x_min, float x_max) {
  float span = x_max - x_min;
  return x_min + span * (float)x / 4095.0f;
}
static float AngleUnify(float angle) {
  // 角度范围 [-180, 180]
  if (angle < -3.1415926) {
    angle += 6.2831853;
  }
  angle = angle / 3.1415926 * 180.0f;
  return angle;
}
static float RadUnify(float rad) {
  // 角度范围 [-PI, PI]
  if (rad < -3.1415926) {
    rad += 6.2831853;
  }
  rad = rad / 3.1415926;
  return rad;
}
