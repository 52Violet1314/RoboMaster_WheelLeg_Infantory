#ifndef __DM_IMU_H
#define __DM_IMU_H

#include "stm32h7xx_hal.h"


#define ACCEL_CAN_MAX (235.2f)
#define ACCEL_CAN_MIN	(-235.2f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#define CMD_READ 0
#define CMD_WRITE 1

typedef enum
{
	COM_USB=0,
	COM_RS485,
	COM_CAN,
	COM_VOFA

}imu_com_port_e;

typedef enum
{
	CAN_BAUD_1M=0,
	CAN_BAUD_500K,
	CAN_BAUD_400K,
	CAN_BAUD_250K,
	CAN_BAUD_200K,
	CAN_BAUD_100K,
	CAN_BAUD_50K,
	CAN_BAUD_25K
	
}imu_baudrate_e;

typedef enum 
{
	REBOOT_IMU=0,
	ACCEL_DATA,
	GYRO_DATA,
	EULER_DATA,
	QUAT_DATA,
	SET_ZERO,
	ACCEL_CALI,
	GYRO_CALI,
	MAG_CALI,
	CHANGE_COM,
	SET_DELAY,
	CHANGE_ACTIVE,
	SET_BAUD,
	SET_CAN_ID,
	SET_MST_ID,
	DATA_OUTPUT_SELECTION,
	SAVE_PARAM=254,
	RESTORE_SETTING=255
}reg_id_e;



typedef struct
{
	uint8_t can_id;
	uint8_t mst_id;
	
	FDCAN_HandleTypeDef *can_handle;
	
	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];
	
	float q[4];

	float cur_temp;

}imu_t;

extern imu_t imu;

#ifdef __cplusplus
extern "C" {
#endif

void IMU_Init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan,imu_t *IMU);
void IMU_RequestEuler(imu_t *IMU);
void IMU_UpdateEuler(uint8_t* pData,imu_t *IMU);

#ifdef __cplusplus
}
#endif



#endif