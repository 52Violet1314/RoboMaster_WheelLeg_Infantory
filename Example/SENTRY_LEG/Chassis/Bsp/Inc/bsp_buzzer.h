#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#include "main.h"

//INCLUDE²¿·Ö
#include "tim.h"

#define MAX_PSC             1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000

void Buzzer_Off(void);
void Buzzer_On(uint16_t psc, uint16_t pwm);

#endif
