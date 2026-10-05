/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM7_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief 读取 TIM7 扩展出的单调微秒时间戳。
 * @note TIM7 以 1 MHz 工作；返回值为 uint32_t，会在约 71.6 min 回绕。
 *       只要两个样本间隔远小于该回绕周期，unsigned 相减仍然正确。
 */
uint32_t TIM7_GetTimestampUs(void);

/** @brief 由 HAL_TIM_PeriodElapsedCallback 在 TIM7 溢出时调用。 */
void TIM7_TimestampOverflowCallback(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

