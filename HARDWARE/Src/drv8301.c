#include "drv8301.h"
#include "delay.h"
#include "assert.h"
void Mx_nCS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}



void SPI_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // 使能GPIOC时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);  // 使能SPI3时钟

    // GPIOC 10,11,12初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
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
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 15;  // 通常使用7或15
    SPI_Init(SPI3, &SPI_InitStructure);

    SPI_Cmd(SPI3, ENABLE);

    SPI_ReadWriteByte(0x0000);//启动传输
}

//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
//fAPB2时钟一般为84Mhz：
void SPI_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
    SPI3->CR1&=0XFFC7;//位3-5清零，用来设置波特率
    SPI3->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度
    SPI_Cmd(SPI3,ENABLE); //使能SPI1
}
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u16 SPI_ReadWriteByte(u16 TxData)
{
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空

    SPI_I2S_SendData(SPI3, TxData); //通过外设SPIx发送一个byte数据

    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte

    return SPI_I2S_ReceiveData(SPI3); //返回通过SPIx最近接收的数据
}


uint16_t DRV8301_readSpi(const DRV8301_RegName_e regName)
{
    M1_nCS_CLR
    // 2. 准备SPI读取操作
    uint16_t zerobuff = 0x1111;  // 用于接收数据的空缓冲区
    // 构建控制字：读模式 + 寄存器地址 + 空数据(0)
    uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Read, regName, 0);
    uint16_t recbuff = 0xbeef;  // 接收缓冲初始值（用于错误检测）
    // 3. 发送读命令
    SPI_ReadWriteByte(controlword);
    // 4. 片选信号特殊处理（实测需要脉冲）
    // 数据手册说明不需要脉冲，但实际硬件需要
    // M1_nCS_SET
    // delay_ms(1);
    // M1_nCS_CLR
    // 5. 执行SPI收发操作
    zerobuff=SPI_ReadWriteByte(0);
    // 6. 释放片选信号
    M1_nCS_SET
    // 7. 验证数据有效性
    // assert(zerobuff != 0xbeef);  // 确保接收缓冲区被更新
    // 8. 返回有效数据位
    return(zerobuff & DRV8301_DATA_MASK);
}
/* 操作片选信号 */
/* 执行阻塞式写入 */
void DRV8301_writeSpi(const DRV8301_RegName_e regName,const uint16_t data) {
    // 拉低片选信号
    M1_nCS_CLR
    delay_ms(1);

    // 执行阻塞式写入
    uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Write, regName, data);
    SPI_ReadWriteByte(controlword);
    delay_ms(1);

    // 拉高片选信号
    M1_nCS_SET
    delay_ms(1);

    return;
}  // DRV8301_writeSpi()函数结束


void DRV8301_writeData( DRV_SPI_8301_Vars_t *Spi_8301_Vars) {
    DRV8301_RegName_e  drvRegName;
    uint16_t drvDataNew;

    if (Spi_8301_Vars->SndCmd) {
        // 更新控制寄存器1
        drvRegName = DRV8301_RegName_Control_1;
        drvDataNew = Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT |  \
                     Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET   |  \
                     Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE     |  \
                     Spi_8301_Vars->Ctrl_Reg_1.OC_MODE      |  \
                     Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET;
        DRV8301_writeSpi(drvRegName,drvDataNew);

        // 更新控制寄存器2
        drvRegName = DRV8301_RegName_Control_2;
        drvDataNew = Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET      |  \
                     Spi_8301_Vars->Ctrl_Reg_2.GAIN          |  \
                     Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2  |  \
                     Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF;
        DRV8301_writeSpi(drvRegName,drvDataNew);

        Spi_8301_Vars->SndCmd = false;
    }

    return;
}  // DRV8301_writeData()函数结束


void DRV8301_readData(DRV_SPI_8301_Vars_t *Spi_8301_Vars) {
    DRV8301_RegName_e  drvRegName;
    uint16_t drvDataNew;

    if (Spi_8301_Vars->RcvCmd) {
        // 更新状态寄存器1
        drvRegName = DRV8301_RegName_Status_1;
        drvDataNew = DRV8301_readSpi(drvRegName);
        Spi_8301_Vars->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
        Spi_8301_Vars->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
        Spi_8301_Vars->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
        Spi_8301_Vars->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
        Spi_8301_Vars->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
        Spi_8301_Vars->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);

        // 更新状态寄存器2
        drvRegName = DRV8301_RegName_Status_2;
        drvDataNew = DRV8301_readSpi(drvRegName);
        Spi_8301_Vars->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
        Spi_8301_Vars->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);

        // 更新控制寄存器1
        drvRegName = DRV8301_RegName_Control_1;
        drvDataNew = DRV8301_readSpi(drvRegName);
        Spi_8301_Vars->Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
        Spi_8301_Vars->Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
        Spi_8301_Vars->Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
        Spi_8301_Vars->Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
        Spi_8301_Vars->Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);

        // 更新控制寄存器2
        drvRegName = DRV8301_RegName_Control_2;
        drvDataNew = DRV8301_readSpi(drvRegName);
        Spi_8301_Vars->Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
        Spi_8301_Vars->Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
        Spi_8301_Vars->Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
        Spi_8301_Vars->Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);

        Spi_8301_Vars->RcvCmd = false;
    }

    return;
}  // DRV8301_readData()函数结束
