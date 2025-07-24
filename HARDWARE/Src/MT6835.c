//
// Created by AerialBird on 25-7-4.
//
#include "MT6835.h"

void MT6835_CS(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SPI_MT6835_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // 使能GPIOC时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);  // 使能SPI3时钟

    // GPIOC 10,11,12初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          // 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          // 上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);                // 初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          // 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          // 上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);                // 初始化

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);  // PC10复用为SPI3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);  // PC11复用为SPI3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);  // PC12复用为SPI3

    // 复位SPI3
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;    // 16位数据
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;  // 通常使用7或15
    SPI_Init(SPI3, &SPI_InitStructure);

    SPI_Cmd(SPI3, ENABLE);
    // MT6835_CS_CLR;
    // SPI_MT6835_ReadWriteByte(0x0000);//启动传输
    // MT6835_CS_SET;

}

//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
//fAPB2时钟一般为84Mhz：
void SPI_MT6835_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
    SPI3->CR1&=0XFFC7;//位3-5清零，用来设置波特率
    SPI3->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度
    SPI_Cmd(SPI3,ENABLE); //使能SPI1
}
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t SPI_MT6835_ReadWriteByte(uint8_t TxData)
{
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空

    SPI_I2S_SendData(SPI3, TxData); //通过外设SPIx发送一个byte数据

    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte

    return SPI_I2S_ReceiveData(SPI3); //返回通过SPIx最近接收的数据
}


uint16_t MT6835_readSpi(const MT6835_RegName_e regName)
{
    MT6835_CS_CLR
    // 2. 准备SPI读取操作
    uint16_t zerobuff = 0x1111;  // 用于接收数据的空缓冲区
    // 构建控制字：读模式 + 寄存器地址 + 空数据(0)
    uint16_t controlword = (uint16_t)MT6835_buildCtrlWord(MT6835_CtrlMode_Read_register, regName, 0);
    uint16_t recbuff = 0xbeef;  // 接收缓冲初始值（用于错误检测）
    // 3. 发送读命令
    SPI_MT6835_ReadWriteByte(controlword);
    // 4. 片选信号特殊处理（实测需要脉冲）
    // 数据手册说明不需要脉冲，但实际硬件需要
    MT6835_CS_SET
    delay_ms(1);
    MT6835_CS_CLR
    // 5. 执行SPI收发操作
    zerobuff=SPI_MT6835_ReadWriteByte(controlword);
    // 6. 释放片选信号
    MT6835_CS_SET
    // 7. 验证数据有效性
    // assert(zerobuff != 0xbeef);  // 确保接收缓冲区被更新
    // 8. 返回有效数据位
    return(zerobuff & MT6835_DATA_MASK);
}
// /* 操作片选信号 */
// /* 执行阻塞式写入 */
// void DRV8301_writeSpi(const DRV8301_RegName_e regName,const uint16_t data) {
//     // 拉低片选信号
//     MT6835_CS_CLR
//     delay_ms(1);
//
//     // 执行阻塞式写入
//     uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Write, regName, data);
//     SPI_ReadWriteByte(controlword);
//     delay_ms(1);
//
//     // 拉高片选信号
//     MT6835_CS_SET
//     delay_ms(1);
//
//     return;
// }  // DRV8301_writeSpi()函数结束


uint8_t ReadMT6835(uint16_t add)
{
    uint8_t pTxData[3] = {0, 0, 0}, pRxData[3] = {0, 0, 0};
    uint16_t timeout = 0xFFFF;

    // 设置CS高
    MT6835_CS_SET // 你需要根据实际CS引脚调整端口和引脚

    // 组合地址和读命令
    add = ((add & 0x0FFF) | 0x8000);  // 假设READCOMMAND = 0x8000
    pTxData[0] = (uint8_t)(add >> 8);
    pTxData[1] = (uint8_t)(add);
    pTxData[2] = (uint8_t)(add);      // 第三字节按原逻辑重复add低字节

    // 设置CS低
    MT6835_CS_CLR

    for (int i = 0; i < 3; i++) {
        // 等待发送缓冲区为空
        timeout = 0xFFFF;
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) {
            if (--timeout == 0) break;
        }
        SPI_I2S_SendData(SPI3, pTxData[i]);

        // 等待接收缓冲区非空
        timeout = 0xFFFF;
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) {
            if (--timeout == 0) break;
        }
        pRxData[i] = SPI_I2S_ReceiveData(SPI3);
    }

    // 设置CS高
    MT6835_CS_SET

    return pRxData[2];
}