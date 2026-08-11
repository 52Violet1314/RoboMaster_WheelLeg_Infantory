#ifndef __APP_REMOTE_TASK_HPP__
#define __APP_REMOTE_TASK_HPP__ 
#include "FreeRTOS.h"
#include "app_Task.hpp"
#include "SBUS.h"
#include "main.h"
#include "app_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_Remote_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
