#include "app_motor.h"

DM_Motor_t DM_8009P1;
DM_Motor_t DM_8009P2;
DM_Motor_t DM_8009P3;
DM_Motor_t DM_8009P4;
DM_Motor_t DM_3519L;
DM_Motor_t DM_3519R;
DM_Motor_t DM_3507;
DM_Motor_t DM_4310;

static float uint16_to_float(uint16_t x, float x_min, float x_max, uint8_t bits);
static float uint12_to_float(uint16_t x, float x_min, float x_max);

/* 每个电机型号的 MIT 输出端速度/扭矩映射量程。
 * 同一表供反馈解码、MIT 打包和速度模式限幅共用，防止模式切换后单位变化。
 * MIT 仅在电调 VMAX/TMAX 已配为同一映射量程时才能保证字节含义一致。 */
typedef struct {
  float position_rad;
  float speed_rad_s;
  float torque_nm;
} DM_MIT_Limit_t;

/* 除位置字段外，MIT 的速度/力矩字段必须按实际电机型号缩放。
 * 所有数值均为输出端物理单位，和反馈解码使用同一规格。 */
static DM_MIT_Limit_t dm_motor_command_limits(DM_Motor_Model_t model) {
  switch (model) {
  case DM_MOTOR_MODEL_8009:
    return (DM_MIT_Limit_t){P_MAX_8009, V_MAX_RAD_S_8009, T_MAX_NM_8009};
  case DM_MOTOR_MODEL_3519:
    return (DM_MIT_Limit_t){P_MAX_RAD_3519, V_MAX_RAD_S_3519, T_MAX_NM_3519};
  case DM_MOTOR_MODEL_3507:
    return (DM_MIT_Limit_t){P_MAX_RAD_3507, V_MAX_RAD_S_3507, T_MAX_NM_3507};
  case DM_MOTOR_MODEL_J4310:
    return (DM_MIT_Limit_t){P_MAX_RAD_4310, V_MAX_RAD_S_4310, T_MAX_NM_4310};
  default:
    return (DM_MIT_Limit_t){0.0f, 0.0f, 0.0f};
  }
}

static int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DM_Motor_Init(DM_Motor_t *motor, DM_Motor_Model_t model,
                   uint16_t id, uint8_t mode,
                   FDCAN_HandleTypeDef *hcan) {
  motor->model = model;
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
  if (motor == NULL) return;
  DM_MIT_Limit_t limits = dm_motor_command_limits(motor->model);
  if ((limits.speed_rad_s <= 0.0f) || (limits.torque_nm <= 0.0f)) {
    /* 无效对象：构造零增益、零位置、零速度、零力矩帧，且避免除零编码。 */
    Kp_des = Kd_des = Pos_Des = Spd_Des = Tor_Des = 0.0f;
    limits = (DM_MIT_Limit_t){1.0f, 1.0f, 1.0f};
  }
  if (Pos_Des > limits.position_rad) Pos_Des = limits.position_rad;
  if (Pos_Des < -limits.position_rad) Pos_Des = -limits.position_rad;
  if (Spd_Des > limits.speed_rad_s) Spd_Des = limits.speed_rad_s;
  if (Spd_Des < -limits.speed_rad_s) Spd_Des = -limits.speed_rad_s;
  if (Tor_Des > limits.torque_nm) Tor_Des = limits.torque_nm;
  if (Tor_Des < -limits.torque_nm) Tor_Des = -limits.torque_nm;
  uint16_t Kp = float_to_uint(Kp_des, 0.0f, 500.0f, 12);
  uint16_t Kd = float_to_uint(Kd_des, 0.0f, 5.0f, 12);
  uint16_t Pos = float_to_uint(Pos_Des, -limits.position_rad,
                               limits.position_rad, 16);
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
  if (motor == NULL) return;
  const DM_MIT_Limit_t limits = dm_motor_command_limits(motor->model);
  if (limits.speed_rad_s <= 0.0f) return;
  if (Spd_Des > limits.speed_rad_s) Spd_Des = limits.speed_rad_s;
  if (Spd_Des < -limits.speed_rad_s) Spd_Des = -limits.speed_rad_s;
  uint8_t *vbuf = (uint8_t *)&Spd_Des;
  uint8_t motor_send_data[4] = {vbuf[0], vbuf[1], vbuf[2], vbuf[3]};
  fdcanx_send_data(motor->hcan, motor->ID | 0x200, motor_send_data, 4);
}

void DM_Motor_Pos_Speed_Control(DM_Motor_t *motor, float Pos_Des,
                                float Spd_Des) {
  if (motor == NULL) return;
  const DM_MIT_Limit_t limits = dm_motor_command_limits(motor->model);
  if (limits.position_rad <= 0.0f || limits.speed_rad_s <= 0.0f) return;
  if (Pos_Des > limits.position_rad) Pos_Des = limits.position_rad;
  if (Pos_Des < -limits.position_rad) Pos_Des = -limits.position_rad;
  if (Spd_Des > limits.speed_rad_s) Spd_Des = limits.speed_rad_s;
  if (Spd_Des < -limits.speed_rad_s) Spd_Des = -limits.speed_rad_s;
  uint8_t *pbuf = (uint8_t *)&Pos_Des;
  uint8_t *vbuf = (uint8_t *)&Spd_Des;
  uint8_t motor_send_data[8] = {pbuf[0], pbuf[1], pbuf[2], pbuf[3],
                                vbuf[0], vbuf[1], vbuf[2], vbuf[3]};
  fdcanx_send_data(motor->hcan, motor->ID | 0x100, motor_send_data, 8);
}

void DM_Motor_FeedbackDecode(DM_Motor_t *motor, const uint8_t data[8]) {
  const DM_MIT_Limit_t limits = dm_motor_command_limits(motor->model);
  uint16_t pos_raw;
  uint16_t vel_raw;
  uint16_t tor_raw;
  if (motor == NULL || data == NULL || limits.position_rad <= 0.0f ||
      limits.speed_rad_s <= 0.0f || limits.torque_nm <= 0.0f) return;
  motor->data.err = (data[0] >> 4) & 0x0F;
  pos_raw = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
  vel_raw = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
  tor_raw = (uint16_t)(((uint16_t)(data[4] & 0x0F) << 8) | data[5]);
  motor->data.position_rad = uint16_to_float(pos_raw, -limits.position_rad,
                                               limits.position_rad, 16);
  motor->data.velocity_rad_s = uint12_to_float(vel_raw, -limits.speed_rad_s,
                                                 limits.speed_rad_s);
  motor->data.torque_Nm = uint12_to_float(tor_raw, -limits.torque_nm,
                                           limits.torque_nm);
  motor->data.temp_MOS_C = (float)data[6];
  motor->data.temp_COIL_C = (float)data[7];
}

static float uint16_to_float(uint16_t x, float x_min, float x_max, uint8_t bits) {
  return x_min + (x_max - x_min) * (float)x / ((1 << bits) - 1);
}
static float uint12_to_float(uint16_t x, float x_min, float x_max) {
  return x_min + (x_max - x_min) * (float)x / 4095.0f;
}
