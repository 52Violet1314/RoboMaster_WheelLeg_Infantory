#include "main.h"
#include "tim.h"

void Buzzer_Init(void)
{
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}
void Buzzer_On(uint16_t freq)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, freq-1);
}

void Buzzer_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
}
