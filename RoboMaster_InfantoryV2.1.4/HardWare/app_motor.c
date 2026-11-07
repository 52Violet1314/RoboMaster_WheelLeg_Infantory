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

typedef struct {
  float feedback_position_rad;
  float mapping_velocity_rad_s;
  float mapping_torque_nm;
  float output_velocity_rad_s;
  float output_torque_nm;
} DM_Motor_Profile_t;

static const DM_Motor_Profile_t s_motor_profiles[] = {
  {P_MAX_RAD_8009, V_MAX_RAD_S_8009, T_MAX_NM_8009,
   V_MAX_RAD_S_8009, T_MAX_NM_8009},
  {P_MAX_RAD_3519, V_MAX_RAD_S_3519, T_MAX_NM_3519,
   V_MAX_RAD_S_3519, T_MAX_NM_3519},
  {P_MAX_RAD_3507, V_MAX_RAD_S_3507, T_MAX_NM_3507,
   V_MAX_RAD_S_3507, T_MAX_NM_3507},
  {P_MAX_RAD_4310, V_MAX_RAD_S_4310, T_MAX_NM_4310,
   OUTPUT_V_MAX_RAD_S_4310, OUTPUT_T_MAX_NM_4310},
};

static const DM_Motor_Profile_t *dm_motor_profile(const DM_Motor_t *motor) {
  if (motor == NULL || motor->model > DM_MOTOR_MODEL_J4310) return NULL;
  return &s_motor_profiles[motor->model];
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
  const DM_Motor_Profile_t *profile = dm_motor_profile(motor);
  if (profile == NULL) return;
  if (Spd_Des > profile->output_velocity_rad_s) Spd_Des = profile->output_velocity_rad_s;
  if (Spd_Des < -profile->output_velocity_rad_s) Spd_Des = -profile->output_velocity_rad_s;
  if (Tor_Des > profile->output_torque_nm) Tor_Des = profile->output_torque_nm;
  if (Tor_Des < -profile->output_torque_nm) Tor_Des = -profile->output_torque_nm;
  uint16_t Kp = float_to_uint(Kp_des, 0.0f, 500.0f, 12);
  uint16_t Kd = float_to_uint(Kd_des, 0.0f, 5.0f, 12);
  uint16_t Pos = float_to_uint(Pos_Des, -12.5f, 12.5f, 16);
  uint16_t Spd = float_to_uint(Spd_Des, -profile->mapping_velocity_rad_s,
                               profile->mapping_velocity_rad_s, 12);
  uint16_t Tor = float_to_uint(Tor_Des, -profile->mapping_torque_nm,
                               profile->mapping_torque_nm, 12);
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
  const DM_Motor_Profile_t *profile = dm_motor_profile(motor);
  if (profile == NULL) return;
  if (Spd_Des > profile->output_velocity_rad_s) Spd_Des = profile->output_velocity_rad_s;
  if (Spd_Des < -profile->output_velocity_rad_s) Spd_Des = -profile->output_velocity_rad_s;
  uint8_t *vbuf = (uint8_t *)&Spd_Des;
  uint8_t motor_send_data[4] = {vbuf[0], vbuf[1], vbuf[2], vbuf[3]};
  fdcanx_send_data(motor->hcan, motor->ID | 0x200, motor_send_data, 4);
}

void DM_Motor_Pos_Speed_Control(DM_Motor_t *motor, float Pos_Des,
                                float Spd_Des) {
  if (motor == NULL) return;
  const DM_Motor_Profile_t *profile = dm_motor_profile(motor);
  if (profile == NULL) return;
  if (Spd_Des > profile->output_velocity_rad_s) Spd_Des = profile->output_velocity_rad_s;
  if (Spd_Des < -profile->output_velocity_rad_s) Spd_Des = -profile->output_velocity_rad_s;
  uint8_t *pbuf = (uint8_t *)&Pos_Des;
  uint8_t *vbuf = (uint8_t *)&Spd_Des;
  uint8_t motor_send_data[8] = {pbuf[0], pbuf[1], pbuf[2], pbuf[3],
                                vbuf[0], vbuf[1], vbuf[2], vbuf[3]};
  fdcanx_send_data(motor->hcan, motor->ID | 0x100, motor_send_data, 8);
}

void DM_Motor_FeedbackDecode(DM_Motor_t *motor, const uint8_t data[8]) {
  const DM_Motor_Profile_t *profile = dm_motor_profile(motor);
  uint16_t pos_raw;
  uint16_t vel_raw;
  uint16_t tor_raw;
  if (motor == NULL || data == NULL || profile == NULL) return;
  motor->data.err = (data[0] >> 4) & 0x0F;
  pos_raw = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
  vel_raw = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
  tor_raw = (uint16_t)(((uint16_t)(data[4] & 0x0F) << 8) | data[5]);
  motor->data.position_rad = uint16_to_float(pos_raw, -profile->feedback_position_rad,
                                               profile->feedback_position_rad, 16);
  motor->data.velocity_rad_s = uint12_to_float(vel_raw, -profile->mapping_velocity_rad_s,
                                                 profile->mapping_velocity_rad_s);
  motor->data.torque_Nm = uint12_to_float(tor_raw, -profile->mapping_torque_nm,
                                           profile->mapping_torque_nm);
  motor->data.temp_MOS_C = (float)data[6];
  motor->data.temp_COIL_C = (float)data[7];
}

static float uint16_to_float(uint16_t x, float x_min, float x_max, uint8_t bits) {
  return x_min + (x_max - x_min) * (float)x / ((1 << bits) - 1);
}
static float uint12_to_float(uint16_t x, float x_min, float x_max) {
  return x_min + (x_max - x_min) * (float)x / 4095.0f;
}
