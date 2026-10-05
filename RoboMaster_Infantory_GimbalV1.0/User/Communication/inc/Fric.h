#ifndef USER_COMMUNICATION_FRIC_H
#define USER_COMMUNICATION_FRIC_H

#include <stdint.h>

/* 左右轮由 GimbalTask 分别按自身健康状态调用。正常运行 1 ms；本轮保护时
 * 仅本轮改为 3 ms 零速度，不影响另一轮。 */
void Fric_Control_UpdateLeft(uint8_t requested_on, uint8_t allowed);
void Fric_Control_UpdateRight(uint8_t requested_on, uint8_t allowed);
void Fric_Control_Stop(void);

#endif
