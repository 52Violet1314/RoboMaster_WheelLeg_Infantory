#include "POWER.h"
#include "main.h"
void ClassicPower_Enable(void)
{
  HAL_GPIO_WritePin(ClassicPower_GPIO_Port, ClassicPower_Pin, GPIO_PIN_SET);
}
void ClassicPower_Disable(void)
{
  HAL_GPIO_WritePin(ClassicPower_GPIO_Port, ClassicPower_Pin, GPIO_PIN_RESET);
}