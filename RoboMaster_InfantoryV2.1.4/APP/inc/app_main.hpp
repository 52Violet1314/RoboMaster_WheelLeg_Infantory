#ifndef __APP_MAIN_HPP__
#define __APP_MAIN_HPP__
#include "FreeRTOS.h"
#include "SBUS.h"
#include "WS2812.h"
#include "app_Task.hpp"
#include "bsp_fdcan.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "app_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_main(void);
void uart_print(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif