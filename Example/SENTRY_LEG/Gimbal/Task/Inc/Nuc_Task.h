#ifndef __NUC_TASK_H
#define __NUC_TASK_H

#include "main.h"

//INCLUDE部分
#include "CRCs.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"

#define FH_RX 0x50  
#define FH_TX 0x53

typedef struct 
{
  uint8_t head;
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
	uint8_t enem_color;  //0:red 1:blue 2:未知
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t checksum;	
}__attribute__((packed)) Aim_Tx;

typedef struct
{
  uint8_t head;
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t checksum;	
}__attribute__((packed)) Aim_Rx;

void Nuc_Init(void);
void Nuc_Task(void);
void Send_Packet(void);
void Recieve_Host(uint8_t* buff);

//EXTERN部分
extern Aim_Rx aim_rx;
extern Aim_Tx aim_tx;

extern bool nuc_ready_flag;

#endif

