#include "bsp_buzzer.h"

/*******************************************************************************************************
·äÃùÆ÷¿ª¹Øº¯Êý
********************************************************************************************************/
void Buzzer_On(uint16_t psc, uint16_t pwm)
{
	__HAL_TIM_PRESCALER(&htim4, psc);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}

void Buzzer_Off(void)
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

