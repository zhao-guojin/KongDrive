//
// Created by AerialBird on 25-7-3.
//

#ifndef PWM_H
#define PWM_H
#include "sys.h"

void PWM3Phase_TIM1_Init(uint16_t arr, uint16_t psc, uint8_t deadTime);
void SetPWM_Duty(uint8_t channel, uint16_t duty);
#endif //PWM_H
