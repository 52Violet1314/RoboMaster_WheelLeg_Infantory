#ifndef __CALCULATETASK_HPP
#define __CALCULATETASK_HPP
#include "FreeRTOS.h"
#include "app_Task.hpp"
#include "SBUS.h"
#include "main.h"
#include "app_Data_Task.hpp"

extern uint8_t ControlState;

void CaculateTask(void *pvParameters);
void VMC_Calculate(Classic_Data_t *data, float F0L, float F0R, float TpL, float TpR, float dt);
float angle_to_rad(float degrees);
void Control_Data_Print(Classic_Data_t *pClassicData);
void IMU_Data_Print(Classic_Data_t *pClassicData);
#endif