//
// Created by AerialBird on 25-6-30.
//

#ifndef TIM_H
#define TIM_H
#include "sys.h"
extern int Encoder_Timer_Overflow1; //±àÂëÆ÷Òç³ö´ÎÊı
extern uint16_t Previous_Count1;
extern uint32_t speed1;
extern int circle_count_motor1;

void TIM3_Int_Init(void);
int16_t Encoder_Get(void);
#endif //TIM_H
