//
// Created by AerialBird on 25-7-2.
//

#ifndef ADC_H
#define ADC_H

#include "sys.h"


void Adc_Init(void); 				//ADCͨ����ʼ��
u16  Get_Adc(u8 ch); 				//���ĳ��ͨ��ֵ
u16 Get_Adc_Average(u8 ch,u8 times);//�õ�ĳ��ͨ����������������ƽ��ֵ


#endif //ADC_H
