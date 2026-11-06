#ifndef __CALCULATETASK_HPP
#define __CALCULATETASK_HPP
#include "FreeRTOS.h"
#include "app_Task.hpp"
#include "SBUS.h"
#include "main.h"
#include "app_Data_Task.hpp"

void CaculateTask(void *pvParameters);
float angle_to_rad(float degrees);
void IMU_Data_Print(Classic_Data_t *pClassicData);
#endif
