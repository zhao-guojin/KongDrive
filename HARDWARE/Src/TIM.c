//
// Created by AerialBird on 25-6-30.
//
#include "TIM.h"

int Encoder_Timer_Overflow1=0;
uint16_t Previous_Count1=0;
uint32_t speed1 = 0;
int circle_count_motor1 = 0;
void TIM3_Int_Init(void)
{
	GPIO_InitTypeDef 	        GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef	 	TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef 			TIM_ICInitStructure;
	NVIC_InitTypeDef			NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//����GPIOC��ʱ��



	GPIO_InitStruct.GPIO_Pin  	= GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd	=  GPIO_PuPd_UP;

	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);//������6ӳ��TIM8
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);//������7ӳ��TIM8



   //��ʱ������-------------------------------------------------------------
    TIM_TimeBaseInitStructure.TIM_Period = 330*4;//65536-1;   				//��װ��ֵ ������������������
	TIM_TimeBaseInitStructure.TIM_Prescaler=0x0; 				 	//Ԥ��Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 	//���ϼ���
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		//ʱ�ӷָ�

	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3

    //������ģʽ����--------------------------------------------------------------

	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//����ģʽ3

	TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0xF;  //�˲���ֵ
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    //����ж�����--------------------------------------------------------------
	// TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����TIM4����ж�
	//
	// NVIC_InitStructure.NVIC_IRQChannel					=TIM3_IRQn;
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority		=0x01;
	// NVIC_InitStructure.NVIC_IRQChannelCmd				=ENABLE;
	// NVIC_Init(&NVIC_InitStructure);

   //Reset counter-----------------------------------------------
   TIM_SetCounter(TIM3,0); //TIM4->CNT=0
   TIM_Cmd(TIM3, ENABLE);

    circle_count_motor1=0;
}



void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
    {
        Encoder_Timer_Overflow1++;


        if((TIM3->CR1>>4 & 0x01)==0)     //DIR==0  ͨ���Ĵ���TIMx_CR1����λ�ж�  0�������ת
            circle_count_motor1++;
        else if((TIM3->CR1>>4 & 0x01)==1)//DIR==1  ͨ���Ĵ���TIMx_CR1����λ�ж�  1�������ת
            circle_count_motor1--;
    }
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);

}
int16_t Encoder_Get(void)
{
	int16_t Temp;
	Temp = TIM_GetCounter(TIM3);
	// TIM_SetCounter(TIM3, 0);
	return Temp;
}