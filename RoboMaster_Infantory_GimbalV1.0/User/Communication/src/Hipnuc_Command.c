#include "Hipnuc_Command.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define HIPNUC_COMMAND_MAX_LENGTH (96U)

static uint8_t hipnuc_baud_supported(uint32_t baud)
{
  switch (baud) {
  case 4800U: case 9600U: case 19200U: case 38400U: case 57600U:
  case 115200U: case 230400U: case 460800U: case 921600U:
    return 1U;
  default:
    return 0U;
  }
}

HAL_StatusTypeDef Hipnuc_Command_SendRaw(const char *command)
{
  if (command == NULL) return HAL_ERROR;
  const size_t length = strnlen(command, HIPNUC_COMMAND_MAX_LENGTH);
  /* 命令必须完整包含 CRLF；strnlen 到上限说明长度非法或没有结束符。 */
  if (length < 3U || length >= HIPNUC_COMMAND_MAX_LENGTH ||
      command[length - 2U] != '\r' || command[length - 1U] != '\n')
    return HAL_ERROR;
  return HAL_UART_Transmit(&huart2, (const uint8_t *)command,
                           (uint16_t)length, HIPNUC_COMMAND_TIMEOUT_MS);
}

HAL_StatusTypeDef Hipnuc_Command_SetLogEnabled(uint8_t enabled)
{
  return Hipnuc_Command_SendRaw((enabled != 0U) ? "LOG ENABLE\r\n" : "LOG DISABLE\r\n");
}

HAL_StatusTypeDef Hipnuc_Command_SetHi91PeriodMs(uint16_t period_ms)
{
  char command[HIPNUC_COMMAND_MAX_LENGTH];
  if (period_ms == 0U) return Hipnuc_Command_SendRaw("LOG HI91 ONTIME 0\r\n");
  if (period_ms < HIPNUC_HI91_MIN_PERIOD_MS || period_ms > HIPNUC_HI91_MAX_PERIOD_MS)
    return HAL_ERROR;
  const int written = snprintf(command, sizeof(command), "LOG HI91 ONTIME %u.%03u\r\n",
                               (unsigned int)(period_ms / 1000U),
                               (unsigned int)(period_ms % 1000U));
  if (written <= 0 || (size_t)written >= sizeof(command)) return HAL_ERROR;
  return Hipnuc_Command_SendRaw(command);
}

HAL_StatusTypeDef Hipnuc_Command_SetBaud(uint32_t baud)
{
  char command[HIPNUC_COMMAND_MAX_LENGTH];
  if (hipnuc_baud_supported(baud) == 0U) return HAL_ERROR;
  const int written = snprintf(command, sizeof(command), "SERIALCONFIG %lu\r\n",
                               (unsigned long)baud);
  if (written <= 0 || (size_t)written >= sizeof(command)) return HAL_ERROR;
  return Hipnuc_Command_SendRaw(command);
}

HAL_StatusTypeDef Hipnuc_Command_SetUrfr(uint16_t urfr_code)
{
  char command[HIPNUC_COMMAND_MAX_LENGTH];
  const int written = snprintf(command, sizeof(command), "CONFIG IMU URFR %u\r\n",
                               (unsigned int)urfr_code);
  if (written <= 0 || (size_t)written >= sizeof(command)) return HAL_ERROR;
  return Hipnuc_Command_SendRaw(command);
}

HAL_StatusTypeDef Hipnuc_Command_SetWorldCoordinate(Hipnuc_WorldCoordinate_t coordinate)
{
  if (coordinate != HIPNUC_COORD_ENU && coordinate != HIPNUC_COORD_NWU) return HAL_ERROR;
  return Hipnuc_Command_SendRaw((coordinate == HIPNUC_COORD_ENU) ?
      "CONFIG IMU COORD 0\r\n" : "CONFIG IMU COORD 4\r\n");
}

HAL_StatusTypeDef Hipnuc_Command_SetAttitudeMode(Hipnuc_AttitudeMode_t mode)
{
  char command[HIPNUC_COMMAND_MAX_LENGTH];
  if (mode != HIPNUC_ATT_MODE_VRU_6_AXIS && mode != HIPNUC_ATT_MODE_AHRS_9_AXIS &&
      mode != HIPNUC_ATT_MODE_HUMANOID && mode != HIPNUC_ATT_MODE_LOW_SPEED_GROUND &&
      mode != HIPNUC_ATT_MODE_LOW_DYNAMIC)
    return HAL_ERROR;
  const int written = snprintf(command, sizeof(command), "CONFIG ATT MODE %u\r\n",
                               (unsigned int)mode);
  if (written <= 0 || (size_t)written >= sizeof(command)) return HAL_ERROR;
  return Hipnuc_Command_SendRaw(command);
}

HAL_StatusTypeDef Hipnuc_Command_Save(void)
{
  return Hipnuc_Command_SendRaw("SAVECONFIG\r\n");
}

HAL_StatusTypeDef Hipnuc_Command_Reboot(void)
{
  return Hipnuc_Command_SendRaw("REBOOT\r\n");
}

HAL_StatusTypeDef Hipnuc_Command_QueryVersion(void)
{
  return Hipnuc_Command_SendRaw("LOG VERSION\r\n");
}

HAL_StatusTypeDef Hipnuc_Command_QueryComConfig(void)
{
  return Hipnuc_Command_SendRaw("LOG COMCONFIG\r\n");
}

HAL_StatusTypeDef Hipnuc_Command_QueryUserConfig(void)
{
  return Hipnuc_Command_SendRaw("LOG USRCONFIG\r\n");
}

HAL_StatusTypeDef Hipnuc_Command_StartMagCalibration(uint8_t planar_2d)
{
  return Hipnuc_Command_SendRaw((planar_2d != 0U) ?
      "CONFIG MCAL START 2D\r\n" : "CONFIG MCAL START\r\n");
}

HAL_StatusTypeDef Hipnuc_Command_QueryMagCalibration(void)
{
  return Hipnuc_Command_SendRaw("LOG MCAL STAT\r\n");
}
