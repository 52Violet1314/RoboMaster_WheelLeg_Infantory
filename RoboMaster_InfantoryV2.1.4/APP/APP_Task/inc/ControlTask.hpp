#ifndef __CONTROLTASK_HPP
#define __CONTROLTASK_HPP
#include "FreeRTOS.h"
#include "app_Task.hpp"
#include "SBUS.h"
#include "main.h"

extern float DM3519_speed_des[4];

void MotorControlTask(void *pvParameters);
#endif
