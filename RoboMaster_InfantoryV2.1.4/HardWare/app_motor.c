#include "app_motor.h"

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

static float uint16_to_float(uint16_t x, float x_min, float x_max, uint8_t bits);
static int16_t float_to_int12(float x, float x_min, float x_max);
static float int12_to_float(int16_t x, float x_min, float x_max);

/* MIT 力矩字段的量程必须与实际电机型号一致：
 * 四个关节 J8009 的峰值为 40 Nm；两只轮毂 S3519 的峰值为 7.8 Nm。
 * 不能把控制限幅后的 20.5 Nm 继续按 ±18 Nm 编码，否则 12 位字段溢出。 */
typedef struct {
  float speed_rad_s;
  float torque_nm;
} DM_MIT_Limit_t;

/* 除位置字段外，MIT 的速度/力矩字段必须按实际电机型号缩放。
 * 所有数值均为输出端物理单位，和反馈解码使用同一规格。 */
static DM_MIT_Limit_t dm_motor_mit_limits(const DM_Motor_t *motor) {
  if ((motor == &DM_8009P1) || (motor == &DM_8009P2) ||
      (motor == &DM_8009P3) || (motor == &DM_8009P4)) {
    return (DM_MIT_Limit_t){V_MAX_RAD_S_8009, T_MAX_NM_8009};
  }
  if ((motor == &DM_3519L) || (motor == &DM_3519R)) {
    return (DM_MIT_Limit_t){V_MAX_RAD_S_3519, T_MAX_NM_3519};
  }
  if (motor == &DM_3507) {
    return (DM_MIT_Limit_t){V_MAX_RAD_S_3507, T_MAX_NM_3507};
  }
  if (motor == &DM_4310) {
    return (DM_MIT_Limit_t){V_MAX_RAD_S_4310, T_MAX_NM_4310};
  }
  return (DM_MIT_Limit_t){45.0f, 18.0f};
}

static int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DM_Motor_Init(DM_Motor_t *motor, uint16_t id, uint8_t mode,
                   FDCAN_HandleTypeDef *hcan) {
  motor->ID = id;
  motor->MODE = mode;
  motor->hcan = hcan;
  motor->Enable_Flag = 1;
  DM_Motor_Enable(motor);
}

void DM_Motor_Enable(DM_Motor_t *motor) {
  static const uint8_t enable_data[8] = {0xFF, 0xFF, 0xFF, 0xFF,
                                         0xFF, 0xFF, 0xFF, 0xFC};
  uint16_t motor_id;
  switch (motor->MODE) {
  case DM_MOTOR_MODE_MIT: motor_id = motor->ID; break;
  case DM_MOTOR_MODE_POS_SPEED: motor_id = motor->ID | 0x100; break;
  case DM_MOTOR_MODE_SPEED: motor_id = motor->ID | 0x200; break;
  default: return;
  }
  fdcanx_send_data(motor->hcan, motor_id, (uint8_t *)enable_data, 8);
}

void DM_Motor_MIT_Control(DM_Motor_t *motor, float Kp_des, float Kd_des,
                          float Pos_Des, float Spd_Des, float Tor_Des) {
  uint8_t motor_send_data[8];
  const DM_MIT_Limit_t limits = dm_motor_mit_limits(motor);
  if (Spd_Des > limits.speed_rad_s) Spd_Des = limits.speed_rad_s;
  if (Spd_Des < -limits.speed_rad_s) Spd_Des = -limits.speed_rad_s;
  if (Tor_Des > limits.torque_nm) Tor_Des = limits.torque_nm;
  if (Tor_Des < -limits.torque_nm) Tor_Des = -limits.torque_nm;
  uint16_t Kp = float_to_uint(Kp_des, 0.0f, 500.0f, 12);
  uint16_t Kd = float_to_uint(Kd_des, 0.0f, 5.0f, 12);
  uint16_t Pos = float_to_uint(Pos_Des, -12.5f, 12.5f, 16);
  uint16_t Spd = float_to_uint(Spd_Des, -limits.speed_rad_s,
                               limits.speed_rad_s, 12);
  uint16_t Tor = float_to_uint(Tor_Des, -limits.torque_nm,
                               limits.torque_nm, 12);
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

void DM_Motor_Speed_Control(DM_Motor_t *motor, float Spd_Des) {
  uint8_t *vbuf = (uint8_t *)&Spd_Des;
  uint8_t motor_send_data[4] = {vbuf[0], vbuf[1], vbuf[2], vbuf[3]};
  fdcanx_send_data(motor->hcan, motor->ID | 0x200, motor_send_data, 4);
}

void DM_Motor_Pos_Speed_Control(DM_Motor_t *motor, float Pos_Des,
                                float Spd_Des) {
  uint8_t *pbuf = (uint8_t *)&Pos_Des;
  uint8_t *vbuf = (uint8_t *)&Spd_Des;
  uint8_t motor_send_data[8] = {pbuf[0], pbuf[1], pbuf[2], pbuf[3],
                                vbuf[0], vbuf[1], vbuf[2], vbuf[3]};
  fdcanx_send_data(motor->hcan, motor->ID | 0x100, motor_send_data, 8);
}

void DM_Motor_8009_Data_Get(DM_Motor_t *motor, uint8_t *data) {
  motor->data.err = (data[0] >> 4) & 0x0F;
  uint16_t pos_raw = (uint16_t)((data[1] << 8) | data[2]);
  int16_t vel_raw = (int16_t)((data[3] << 4) | (data[4] >> 4));
  int16_t tor_raw = (int16_t)(((data[4] & 0x0F) << 8) | data[5]);
  motor->data.position_rad =
      uint16_to_float(pos_raw, -P_MAX_8009, P_MAX_8009, 16);
  motor->data.velocity_rad_s =
      int12_to_float(vel_raw, -V_MAX_RAD_S_8009, V_MAX_RAD_S_8009);
  motor->data.torque_Nm =
      int12_to_float(tor_raw, -T_MAX_NM_8009, T_MAX_NM_8009);
  motor->data.temp_MOS_C = (float)data[6];
  motor->data.temp_COIL_C = (float)data[7];
}

void DM_Motor_3519_Data_Get(DM_Motor_t *motor, uint8_t *data) {
  motor->data.err = (data[0] >> 4) & 0x0F;
  uint16_t pos_raw = (uint16_t)((data[1] << 8) | data[2]);
  int16_t vel_raw = (int16_t)((data[3] << 4) | (data[4] >> 4));
  int16_t tor_raw = (int16_t)(((data[4] & 0x0F) << 8) | data[5]);
  motor->data.position_rad =
      uint16_to_float(pos_raw, -P_MAX_RAD_3519, P_MAX_RAD_3519, 16);
  motor->data.velocity_rad_s =
      int12_to_float(vel_raw, -V_MAX_RAD_S_3519, V_MAX_RAD_S_3519);
  motor->data.torque_Nm =
      int12_to_float(tor_raw, -T_MAX_NM_3519, T_MAX_NM_3519);
  motor->data.temp_MOS_C = (float)data[6];
  motor->data.temp_COIL_C = (float)data[7];
}
void DM_Motor_3507_Data_Get(DM_Motor_t *motor, uint8_t *data) {
  motor->data.err = (data[0] >> 4) & 0x0F;
  uint16_t pos_raw = (uint16_t)((data[1] << 8) | data[2]);
  uint16_t vel_raw = (uint16_t)((data[3] << 4) | (data[4] >> 4));
  uint16_t tor_raw = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);
  motor->data.position_rad = uint16_to_float(pos_raw, -P_MAX_RAD_3507, P_MAX_RAD_3507, 16);
  motor->data.velocity_rad_s = uint16_to_float(vel_raw, -V_MAX_RAD_S_3507, V_MAX_RAD_S_3507, 12);
  motor->data.torque_Nm = uint16_to_float(tor_raw, -T_MAX_NM_3507, T_MAX_NM_3507, 12);
  motor->data.temp_MOS_C = (float)data[6];
  motor->data.temp_COIL_C = (float)data[7];
}
void DM_Motor_4310_Data_Get(DM_Motor_t *motor, uint8_t *data) {
  motor->data.err = (data[0] >> 4) & 0x0F;
  uint16_t pos_raw = (uint16_t)((data[1] << 8) | data[2]);
  uint16_t vel_raw = (uint16_t)((data[3] << 4) | (data[4] >> 4));
  uint16_t tor_raw = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);
  motor->data.position_rad = uint16_to_float(
      pos_raw, -P_MAX_RAD_4310, P_MAX_RAD_4310, 16);
  motor->data.velocity_rad_s = uint16_to_float(
      vel_raw, -V_MAX_RAD_S_4310, V_MAX_RAD_S_4310, 12);
  motor->data.torque_Nm = uint16_to_float(
      tor_raw, -T_MAX_NM_4310, T_MAX_NM_4310, 12);
  motor->data.temp_MOS_C = (float)data[6];
  motor->data.temp_COIL_C = (float)data[7];
}

static float uint16_to_float(uint16_t x, float x_min, float x_max, uint8_t bits) {
  return x_min + (x_max - x_min) * (float)x / ((1 << bits) - 1);
}
static int16_t float_to_int12(float x, float x_min, float x_max) {
  float span = x_max - x_min;
  float offset = x - x_min;
  if (offset < 0.0f) offset = 0.0f;
  if (offset > span) offset = span;
  return (int16_t)(offset / span * 4095.0f);
}
static float int12_to_float(int16_t x, float x_min, float x_max) {
  return x_min + (x_max - x_min) * (float)x / 4095.0f;
}
