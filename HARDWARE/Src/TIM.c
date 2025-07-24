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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//开启GPIOC的时钟



	GPIO_InitStruct.GPIO_Pin  	= GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd	=  GPIO_PuPd_UP;

	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);//将引脚6映像到TIM8
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);//将引脚7映像到TIM8



   //定时器设置-------------------------------------------------------------
    TIM_TimeBaseInitStructure.TIM_Period = 330*4;//65536-1;   				//重装载值 这是两相脉冲总数量
	TIM_TimeBaseInitStructure.TIM_Prescaler=0x0; 				 	//预分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 	//向上计数
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		//时钟分割

	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3

    //编码器模式设置--------------------------------------------------------------

	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//计数模式3

	TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0xF;  //滤波器值
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    //溢出中断设置--------------------------------------------------------------
	// TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许TIM4溢出中断
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


        if((TIM3->CR1>>4 & 0x01)==0)     //DIR==0  通过寄存器TIMx_CR1第四位判断  0：电机正转
            circle_count_motor1++;
        else if((TIM3->CR1>>4 & 0x01)==1)//DIR==1  通过寄存器TIMx_CR1第四位判断  1：电机正转
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