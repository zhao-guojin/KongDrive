//
// Created by AerialBird on 25-7-3.
//
#include "PWM.h"


/**
 * @brief  配置 TIM1 及对应 GPIO（PA8/PA9/PA10 -> CH1/2/3，
 *         PB13/PB14/PB15 -> CH1N/2N/3N）输出三相互补 PWM
 * @param  arr      自动重装载寄存器值，用于设置 PWM 周期 (ARR = 周期计数上限)
 * @param  psc      预分频值，用于设置时钟分频 (PSC+1 = 分频倍数)
 * @param  deadTime 死区时间，单位为系统时钟周期数 (0～255)
 * @retval None
 */
void PWM3Phase_TIM1_Init(uint16_t arr, uint16_t psc, uint8_t deadTime)
{
    GPIO_InitTypeDef        GPIO_InitStruct;   // GPIO 配置结构体
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;    // 定时器基本时基配置结构体
    TIM_OCInitTypeDef       TIM_OCStruct;      // 定时器输出比较 (PWM) 配置结构体
    TIM_BDTRInitTypeDef     TIM_BDTRStruct;    // 死区与刹车及输出控制配置结构体

    /* 1. 使能外设时钟
     *    - GPIOA/ GPIOB 时钟：用于复用引脚 PA8/PA9/PA10 和 PB13/PB14/PB15
     *    - TIM1 时钟：高级定时器，用于三相 PWM 生成
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // /* 2. GPIO 复用配置
    //  *    将对应引脚映射到 TIM1 的通道功能 (AF1)
    //  *    PA8  -> TIM1_CH1
    //  *    PA9  -> TIM1_CH2
    //  *    PA10 -> TIM1_CH3
    //  *    PB13 -> TIM1_CH1N (互补输出)
    //  *    PB14 -> TIM1_CH2N
    //  *    PB15 -> TIM1_CH3N
    //  */
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,  GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
    // GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM1);

    /* 2.1 通用 GPIO 设置
     *    - 模式：复用 (AF)
     *    - 输出类型：推挽 (PP)，确保能驱动负载
     *    - 速度：100MHz，高速输出
     *    - 上拉/下拉：无，电平完全由定时器控制
     */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;         // 复用模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;        // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;    // 高速
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;     // 无上下拉

    // 配置主通道引脚 PA8/PA9/PA10
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置互补通道引脚 PB13/PB14/PB15
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 2. GPIO 复用配置
 *    将对应引脚映射到 TIM1 的通道功能 (AF1)
 *    PA8  -> TIM1_CH1
 *    PA9  -> TIM1_CH2
 *    PA10 -> TIM1_CH3
 *    PB13 -> TIM1_CH1N (互补输出)
 *    PB14 -> TIM1_CH2N
 *    PB15 -> TIM1_CH3N
 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM1);

    /* 3. TIM1 基本时基配置
     *    - Period (ARR)：自动重装载值，决定 PWM 的周期长度
     *    - Prescaler (PSC)：预分频，定时器时钟 = 核心时钟 / (PSC + 1)
     *    - ClockDivision：采样时钟分频，一般 DIV1
     *    - CounterMode：中心对齐模式，1型，计数器上下计数，可减少 PWM 谐波和电流脉动
     *    - RepetitionCounter：重复计数器，设置为 0，即每次都更新 PWM
     */
    TIM_BaseStruct.TIM_Period            = arr;
    TIM_BaseStruct.TIM_Prescaler         = psc;
    TIM_BaseStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_CounterMode       = TIM_CounterMode_CenterAligned1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_BaseStruct);

    /* 4. PWM 模式及互补输出配置
     *    - OCMode：PWM1 模式 (计数 < CCR 输出高电平，计数 >= CCR 输出低电平)
     *    - OutputState：使能主通道输出
     *    - OutputNState：使能互补通道输出
     *    - Pulse：初始占空比，设置为 50% (arr/2)，后续可动态修改 CCRx
     *    - OCPolarity：输出极性，高有效
     *    - OCNPolarity：互补极性，高有效
     *    - OCIdleState：空闲态输出低平??
     *    - OCNIdleState：互补空闲态输出低平??
     */
    TIM_OCStruct.TIM_OCMode        = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState   = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OutputNState  = TIM_OutputNState_Enable;
    TIM_OCStruct.TIM_Pulse         = arr / 2;
    TIM_OCStruct.TIM_OCPolarity    = TIM_OCPolarity_High;
    TIM_OCStruct.TIM_OCNPolarity   = TIM_OCNPolarity_High;
    TIM_OCStruct.TIM_OCIdleState   = TIM_OCIdleState_Reset;
    TIM_OCStruct.TIM_OCNIdleState  = TIM_OCNIdleState_Reset;

    // 分别初始化三个通道及其互补输出
    TIM_OC1Init(TIM1, &TIM_OCStruct);
    TIM_OC2Init(TIM1, &TIM_OCStruct);
    TIM_OC3Init(TIM1, &TIM_OCStruct);

    /* 5. 死区时间与高级功能配置
     *    - OSSR/OSSI：当发生刹车或死区时，主/互补通道能否自动设为安全状态
     *    - LOCK：锁定级别，OFF 表示不锁定寄存器
     *    - DeadTime：死区时间长度，防止上下桥导通重叠短路
     *    - Break：硬件刹车使能，这里不使用 (Disable)
     *    - BreakPolarity：刹车信号有效极性 (高电平有效)
     *    - AutomaticOutput：刹车恢复后自动恢复 PWM 输出
     */
    TIM_BDTRStruct.TIM_OSSRState       = TIM_OSSRState_Enable;
    TIM_BDTRStruct.TIM_OSSIState       = TIM_OSSIState_Enable;
    TIM_BDTRStruct.TIM_LOCKLevel       = TIM_LOCKLevel_OFF;
    TIM_BDTRStruct.TIM_DeadTime        = deadTime;
    TIM_BDTRStruct.TIM_Break           = TIM_Break_Disable;
    TIM_BDTRStruct.TIM_BreakPolarity   = TIM_BreakPolarity_High;
    TIM_BDTRStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRStruct);

    /* 6. 启动定时器与 PWM 输出
     *    - TIM_Cmd：使能 TIM1 计数器
     *    - TIM_CtrlPWMOutputs：使能主输出开关 (MOE)，对高级定时器必须调用
     */
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

// 设置PWM占空比
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