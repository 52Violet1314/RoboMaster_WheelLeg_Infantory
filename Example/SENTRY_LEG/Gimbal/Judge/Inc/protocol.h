/**
  ******************************************************************************
  * @file    protocol.h
  * @author  Karolance Future
  * @version V1.3.0
  * @date    2022/03/21
  * @brief   依据裁判系统 串口协议附录 V1.3
  ******************************************************************************
  * @attention
	*
  ******************************************************************************
  */

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "stdint.h"

#define HEADER_SOF                  0xA5

#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_HEADER_SIZE    sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE       2
#define REF_PROTOCOL_CRC16_SIZE     2

#define REF_HEADER_CRC_LEN          (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN    (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN        (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
	GAME_STATUS_CMD_ID                     = 0x0001,  //比赛状态数据
	GAME_RESULT_CMD_ID                     = 0x0002,  //比赛结果数据
	GAME_ROBOT_HP_CMD_ID                   = 0x0003,  //机器人血量数据
	
	EVENT_DATA_CMD_ID                      = 0x0101,  //场地事件数据
	REFEREE_WARNING_CMD_ID                 = 0x0104,  //裁判警告信息
	DART_INFO_CMD_ID                       = 0x0105,  //飞镖发射口倒计时
	
	ROBOT_STATUS_CMD_ID                    = 0x0201,  //比赛机器人状态
	POWER_HEAT_DATA_CMD_ID                 = 0x0202,  //实时功率热量数据
	ROBOT_POS_CMD_ID                       = 0x0203,  //机器人位置
	BUFF_CMD_ID                            = 0x0204,  //机器人增益
	HURT_DATA_CMD_ID                       = 0x0206,  //伤害状态
	SHOOT_DAYA_CMD_ID                      = 0x0207,  //实时射击信息
	PROJECTILE_ALLOWANCE_CMD_ID       		 = 0x0208,  //子弹剩余发射数
	RFID_STATUS_CMD_ID               			 = 0x0209,  //机器人RFID状态
	DART_CLIENT_CMD_ID                     = 0x020A,  //飞镖机器人客户端指令数据
	GROUND_ROBOT_POSITION_CMD_ID           = 0x020B,  //地面机器人位置数据
	RADAR_MARK_DATA_CMD_ID                 = 0x020C,  //雷达标记进度
	SENTRY_INFO_CMD_ID                     = 0x020D,  //哨兵自主决策信息同步
	RADAR_INFO_CMD_ID                      = 0x020E,  //雷达自主决策信息同步
	
	ROBOT_INTERACTION_DATA_CMD_ID          = 0x0301,  //机器人间通信
  CUSTOM_ROBOT_DATA_CMD_ID               = 0x0302,  //自定义控制器与机器人交互数据
  MAP_COMMAND_CMD_ID                     = 0x0303,  //选手端小地图交互数据
	REMOTE_CONTROL_CMD_ID                  = 0x0304,  //键鼠遥控数据
	MAP_ROBOT_DATA_CMD_ID						       = 0x0305,	 //选手端小地图接收雷达数据
	CUSTOM_CLIENT_DATA_CMD_ID   					 = 0x0306,	 //自定义控制器与选手端交互数据
	MAP_DATA_CMD_ID   							       = 0x0307,	 //选手端小地图接收哨兵数据
	CUSTOM_INFO_CMD_ID   							     = 0x0308,	 //选手端小地图接收机器人数据
	ROBOT_CUSTOM_DATA                      = 0x0309,   //自定义控制器接受机器人数据
	IDCustomData,
}referee_cmd_id_e;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef __packed struct
{
  uint8_t  SOF;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  CRC8;
} frame_header_struct_t;

typedef __packed struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
