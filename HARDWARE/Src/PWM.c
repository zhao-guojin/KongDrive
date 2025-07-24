//
// Created by AerialBird on 25-7-3.
//
#include "PWM.h"


/**
 * @brief  ���� TIM1 ����Ӧ GPIO��PA8/PA9/PA10 -> CH1/2/3��
 *         PB13/PB14/PB15 -> CH1N/2N/3N��������໥�� PWM
 * @param  arr      �Զ���װ�ؼĴ���ֵ���������� PWM ���� (ARR = ���ڼ�������)
 * @param  psc      Ԥ��Ƶֵ����������ʱ�ӷ�Ƶ (PSC+1 = ��Ƶ����)
 * @param  deadTime ����ʱ�䣬��λΪϵͳʱ�������� (0��255)
 * @retval None
 */
void PWM3Phase_TIM1_Init(uint16_t arr, uint16_t psc, uint8_t deadTime)
{
    GPIO_InitTypeDef        GPIO_InitStruct;   // GPIO ���ýṹ��
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;    // ��ʱ������ʱ�����ýṹ��
    TIM_OCInitTypeDef       TIM_OCStruct;      // ��ʱ������Ƚ� (PWM) ���ýṹ��
    TIM_BDTRInitTypeDef     TIM_BDTRStruct;    // ������ɲ��������������ýṹ��

    /* 1. ʹ������ʱ��
     *    - GPIOA/ GPIOB ʱ�ӣ����ڸ������� PA8/PA9/PA10 �� PB13/PB14/PB15
     *    - TIM1 ʱ�ӣ��߼���ʱ������������ PWM ����
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // /* 2. GPIO ��������
    //  *    ����Ӧ����ӳ�䵽 TIM1 ��ͨ������ (AF1)
    //  *    PA8  -> TIM1_CH1
    //  *    PA9  -> TIM1_CH2
    //  *    PA10 -> TIM1_CH3
    //  *    PB13 -> TIM1_CH1N (�������)
    //  *    PB14 -> TIM1_CH2N
    //  *    PB15 -> TIM1_CH3N
    //  */
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,  GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM1);

    /* 2.1 ͨ�� GPIO ����
     *    - ģʽ������ (AF)
     *    - ������ͣ����� (PP)��ȷ������������
     *    - �ٶȣ�100MHz���������
     *    - ����/�������ޣ���ƽ��ȫ�ɶ�ʱ������
     */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;         // ����ģʽ
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;        // �������
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;    // ����
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;     // ��������

    // ������ͨ������ PA8/PA9/PA10
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ���û���ͨ������ PB13/PB14/PB15
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 2. GPIO ��������
 *    ����Ӧ����ӳ�䵽 TIM1 ��ͨ������ (AF1)
 *    PA8  -> TIM1_CH1
 *    PA9  -> TIM1_CH2
 *    PA10 -> TIM1_CH3
 *    PB13 -> TIM1_CH1N (�������)
 *    PB14 -> TIM1_CH2N
 *    PB15 -> TIM1_CH3N
 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM1);

    /* 3. TIM1 ����ʱ������
     *    - Period (ARR)���Զ���װ��ֵ������ PWM �����ڳ���
     *    - Prescaler (PSC)��Ԥ��Ƶ����ʱ��ʱ�� = ����ʱ�� / (PSC + 1)
     *    - ClockDivision������ʱ�ӷ�Ƶ��һ�� DIV1
     *    - CounterMode�����Ķ���ģʽ��1�ͣ����������¼������ɼ��� PWM г���͵�������
     *    - RepetitionCounter���ظ�������������Ϊ 0����ÿ�ζ����� PWM
     */
    TIM_BaseStruct.TIM_Period            = arr;
    TIM_BaseStruct.TIM_Prescaler         = psc;
    TIM_BaseStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_CounterMode       = TIM_CounterMode_CenterAligned1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);

    /* 4. PWM ģʽ�������������
     *    - OCMode��PWM1 ģʽ (���� < CCR ����ߵ�ƽ������ >= CCR ����͵�ƽ)
     *    - OutputState��ʹ����ͨ�����
     *    - OutputNState��ʹ�ܻ���ͨ�����
     *    - Pulse����ʼռ�ձȣ�����Ϊ 50% (arr/2)�������ɶ�̬�޸� CCRx
     *    - OCPolarity��������ԣ�����Ч
     *    - OCNPolarity���������ԣ�����Ч
     *    - OCIdleState������̬�����ƽ??
     *    - OCNIdleState����������̬�����ƽ??
     */
    TIM_OCStruct.TIM_OCMode        = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState   = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OutputNState  = TIM_OutputNState_Enable;
    TIM_OCStruct.TIM_Pulse         = arr / 2;
    TIM_OCStruct.TIM_OCPolarity    = TIM_OCPolarity_High;
    TIM_OCStruct.TIM_OCNPolarity   = TIM_OCNPolarity_High;
    TIM_OCStruct.TIM_OCIdleState   = TIM_OCIdleState_Reset;
    TIM_OCStruct.TIM_OCNIdleState  = TIM_OCNIdleState_Reset;

    // �ֱ��ʼ������ͨ�����以�����
    TIM_OC1Init(TIM1, &TIM_OCStruct);
    TIM_OC2Init(TIM1, &TIM_OCStruct);
    TIM_OC3Init(TIM1, &TIM_OCStruct);

    /* 5. ����ʱ����߼���������
     *    - OSSR/OSSI��������ɲ��������ʱ����/����ͨ���ܷ��Զ���Ϊ��ȫ״̬
     *    - LOCK����������OFF ��ʾ�������Ĵ���
     *    - DeadTime������ʱ�䳤�ȣ���ֹ�����ŵ�ͨ�ص���·
     *    - Break��Ӳ��ɲ��ʹ�ܣ����ﲻʹ�� (Disable)
     *    - BreakPolarity��ɲ���ź���Ч���� (�ߵ�ƽ��Ч)
     *    - AutomaticOutput��ɲ���ָ����Զ��ָ� PWM ���
     */
    TIM_BDTRStruct.TIM_OSSRState       = TIM_OSSRState_Enable;
    TIM_BDTRStruct.TIM_OSSIState       = TIM_OSSIState_Enable;
    TIM_BDTRStruct.TIM_LOCKLevel       = TIM_LOCKLevel_OFF;
    TIM_BDTRStruct.TIM_DeadTime        = deadTime;
    TIM_BDTRStruct.TIM_Break           = TIM_Break_Disable;
    TIM_BDTRStruct.TIM_BreakPolarity   = TIM_BreakPolarity_High;
    TIM_BDTRStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRStruct);

    /* 6. ������ʱ���� PWM ���
     *    - TIM_Cmd��ʹ�� TIM1 ������
     *    - TIM_CtrlPWMOutputs��ʹ����������� (MOE)���Ը߼���ʱ���������
     */
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// ����PWMռ�ձ�
void SetPWM_Duty(uint8_t channel, uint16_t duty)
{
    switch(channel)
    {
        case 1:

            TIM1->CCR1 = duty;
            break;
        case 2:

            TIM1->CCR2 = duty;
            break;
        case 3:

            TIM1->CCR3 = duty;
            break;
        default:
            break;
    }
}