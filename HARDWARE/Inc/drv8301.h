
#ifndef _DRV8301_H_
#define _DRV8301_H_

#include "stdbool.h"
#include "stdint.h"
#include "sys.h"

// **************************************************************************
// 预定义宏

//! \brief 定义地址掩码
//!
#define DRV8301_ADDR_MASK               (0x7800)

//! \brief 定义数据掩码
//!
#define DRV8301_DATA_MASK               (0x07FF)

//! \brief 定义读/写掩码
//!
#define DRV8301_RW_MASK                 (0x8000)

//! \brief 定义故障类型掩码
//!
#define DRV8301_FAULT_TYPE_MASK         (0x07FF)

//! \brief 定义状态寄存器1中FETLC_OC(C相低侧MOSFET过流)标志位位置
//!
#define DRV8301_STATUS1_FETLC_OC_BITS   (1 << 0)

//! \brief 定义状态寄存器1中FETHC_OC(C相高侧MOSFET过流)标志位位置
//!
#define DRV8301_STATUS1_FETHC_OC_BITS   (1 << 1)

//! \brief 定义状态寄存器1中FETLB_OC(B相低侧MOSFET过流)标志位位置
//!
#define DRV8301_STATUS1_FETLB_OC_BITS   (1 << 2)

//! \brief 定义状态寄存器1中FETHB_OC(B相高侧MOSFET过流)标志位位置
//!
#define DRV8301_STATUS1_FETHB_OC_BITS   (1 << 3)

//! \brief 定义状态寄存器1中FETLA_OC(A相低侧MOSFET过流)标志位位置
//!
#define DRV8301_STATUS1_FETLA_OC_BITS   (1 << 4)

//! \brief 定义状态寄存器1中FETHA_OC(A相高侧MOSFET过流)标志位位置
//!
#define DRV8301_STATUS1_FETHA_OC_BITS   (1 << 5)

//! \brief 定义状态寄存器1中OTW(过温警告)标志位位置
//!
#define DRV8301_STATUS1_OTW_BITS        (1 << 6)

//! \brief 定义状态寄存器1中OTSD(过温关断)标志位位置
//!
#define DRV8301_STATUS1_OTSD_BITS       (1 << 7)

//! \brief 定义状态寄存器1中PVDD_UV(电源电压欠压)标志位位置
//!
#define DRV8301_STATUS1_PVDD_UV_BITS    (1 << 8)

//! \brief 定义状态寄存器1中GVDD_UV(门极驱动电压欠压)标志位位置
//!
#define DRV8301_STATUS1_GVDD_UV_BITS    (1 << 9)

//! \brief 定义状态寄存器1中FAULT(故障)标志位位置
//!
#define DRV8301_STATUS1_FAULT_BITS      (1 << 10)

//! \brief 定义状态寄存器2中Device ID(设备ID)位位置
//!
#define DRV8301_STATUS2_ID_BITS        (15 << 0)

//! \brief 定义状态寄存器2中GVDD_OV(门极驱动电压过压)标志位位置
//!
#define DRV8301_STATUS2_GVDD_OV_BITS    (1 << 7)

//! \brief 定义控制寄存器1中GATE_CURRENT(门极驱动电流)配置位位置
//!
#define DRV8301_CTRL1_GATE_CURRENT_BITS  (3 << 0)

//! \brief 定义控制寄存器1中GATE_RESET(复位状态)配置位位置
//!
#define DRV8301_CTRL1_GATE_RESET_BITS    (1 << 2)

//! \brief 定义控制寄存器1中PWM_MODE(PWM模式)配置位位置
//!
#define DRV8301_CTRL1_PWM_MODE_BITS      (1 << 3)

//! \brief 定义控制寄存器1中OC_MODE(过流保护模式)配置位位置
//!
#define DRV8301_CTRL1_OC_MODE_BITS       (3 << 4)

//! \brief 定义控制寄存器1中OC_ADJ_SET(过流检测阈值)配置位位置
//!
#define DRV8301_CTRL1_OC_ADJ_SET_BITS   (31 << 6)

//! \brief 定义控制寄存器2中OCTW_SET(过流/过温行为)配置位位置
//!
#define DRV8301_CTRL2_OCTW_SET_BITS      (3 << 0)

//! \brief 定义控制寄存器2中GAIN(电流检测增益)配置位位置
//!
#define DRV8301_CTRL2_GAIN_BITS          (3 << 2)

//! \brief 定义控制寄存器2中DC_CAL_1(直流校准通道1)配置位位置
//!
#define DRV8301_CTRL2_DC_CAL_1_BITS      (1 << 4)

//! \brief 定义控制寄存器2中DC_CAL_2(直流校准通道2)配置位位置
//!
#define DRV8301_CTRL2_DC_CAL_2_BITS      (1 << 5)

//! \brief 定义控制寄存器2中OC_TOFF(过流关断延迟)配置位位置
//!
#define DRV8301_CTRL2_OC_TOFF_BITS       (1 << 6)

// **************************************************************************
// 类型定义枚举

//! \brief 读写模式枚举
//!
typedef enum
{
  DRV8301_CtrlMode_Read = 1 << 15,   //!< 读模式
  DRV8301_CtrlMode_Write = 0 << 15    //!< 写模式
} DRV8301_CtrlMode_e;


//! \brief 直流校准模式枚举
//!
typedef enum
{
  DRV8301_DcCalMode_Ch1_Load   = (0 << 4),   //!< 分流放大器1通过输入引脚连接负载
  DRV8301_DcCalMode_Ch1_NoLoad = (1 << 4),   //!< 分流放大器1断开负载，输入引脚短路
  DRV8301_DcCalMode_Ch2_Load   = (0 << 5),   //!< 分流放大器2通过输入引脚连接负载
  DRV8301_DcCalMode_Ch2_NoLoad = (1 << 5)    //!< 分流放大器2断开负载，输入引脚短路
} DRV8301_DcCalMode_e;


//! \brief 故障类型枚举
//!
typedef enum
{
  DRV8301_FaultType_NoFault  = (0 << 0),  //!< 无故障
  DRV8301_FaultType_FETLC_OC = (1 << 0),  //!< C相低侧MOSFET过流故障
  DRV8301_FaultType_FETHC_OC = (1 << 1),  //!< C相高侧MOSFET过流故障
  DRV8301_FaultType_FETLB_OC = (1 << 2),  //!< B相低侧MOSFET过流故障
  DRV8301_FaultType_FETHB_OC = (1 << 3),  //!< B相高侧MOSFET过流故障
  DRV8301_FaultType_FETLA_OC = (1 << 4),  //!< A相低侧MOSFET过流故障
  DRV8301_FaultType_FETHA_OC = (1 << 5),  //!< A相高侧MOSFET过流故障
  DRV8301_FaultType_OTW      = (1 << 6),  //!< 过温预警故障
  DRV8301_FaultType_OTSD     = (1 << 7),  //!< 过温关断故障
  DRV8301_FaultType_PVDD_UV  = (1 << 8),  //!< 主电源欠压故障
  DRV8301_FaultType_GVDD_UV  = (1 << 9),  //!< 门极驱动欠压故障
  DRV8301_FaultType_GVDD_OV  = (1 << 10)  //!< 门极驱动过压故障
} DRV8301_FaultType_e;


//! \brief 过流保护模式枚举
//!
typedef enum
{
  DRV8301_OcMode_CurrentLimit  = 0 << 4,   //!< 检测到过流时电流限幅
  DRV8301_OcMode_LatchShutDown = 1 << 4,   //!< 检测到过流时锁存关断
  DRV8301_OcMode_ReportOnly    = 2 << 4,   //!< 检测到过流时仅报告
  DRV8301_OcMode_Disabled      = 3 << 4    //!< 禁用过流保护
} DRV8301_OcMode_e;


//! \brief 过流关断时间模式枚举
//!
typedef enum
{
  DRV8301_OcOffTimeMode_Normal  = 0 << 6,   //!< 正常逐周期保护
  DRV8301_OcOffTimeMode_Ctrl    = 1 << 6    //!< 过流期间关断时间可控
} DRV8301_OcOffTimeMode_e;


//! \brief 过流/过温警告模式枚举
//!
typedef enum
{
  DRV8301_OcTwMode_Both    = 0 << 0,   //!< 在/OCTW引脚同时报告过温和过流
  DRV8301_OcTwMode_OT_Only = 1 << 0,   //!< 在/OCTW引脚仅报告过温
  DRV8301_OcTwMode_OC_Only = 2 << 0    //!< 在/OCTW引脚仅报告过流
} DRV8301_OcTwMode_e;


//! \brief 峰值电流等级枚举
//!
typedef enum
{
  DRV8301_PeakCurrent_1p70_A  = 0 << 0,   //!< 驱动器峰值电流1.70安
  DRV8301_PeakCurrent_0p70_A  = 1 << 0,   //!< 驱动器峰值电流0.70安
  DRV8301_PeakCurrent_0p25_A  = 2 << 0    //!< 驱动器峰值电流0.25安
} DRV8301_PeakCurrent_e;


//! \brief PWM输入模式枚举
//!
typedef enum
{
  DRV8301_PwmMode_Six_Inputs   = 0 << 3,   //!< 6路独立PWM输入
  DRV8301_PwmMode_Three_Inputs = 1 << 3    //!< 3路独立PWM输入
} DRV8301_PwmMode_e;


//! \brief 寄存器名称枚举
//!
typedef enum
{
  DRV8301_RegName_Status_1  = 0 << 11,   //!< 状态寄存器1
  DRV8301_RegName_Status_2  = 1 << 11,   //!< 状态寄存器2
  DRV8301_RegName_Control_1 = 2 << 11,  //!< 控制寄存器1
  DRV8301_RegName_Control_2 = 3 << 11   //!< 控制寄存器2
} DRV8301_RegName_e;


//! \brief 复位状态枚举
//!
typedef enum
{
  DRV8301_Reset_Normal = 0 << 2,   //!< 正常工作模式
  DRV8301_Reset_All = 1 << 2       //!< 全寄存器复位
} DRV8301_Reset_e;


//! \brief 电流检测放大器增益枚举
//!
typedef enum
{
  DRV8301_ShuntAmpGain_10VpV = 0 << 2,   //!< 10伏/伏增益
  DRV8301_ShuntAmpGain_20VpV = 1 << 2,   //!< 20伏/伏增益
  DRV8301_ShuntAmpGain_40VpV = 2 << 2,   //!< 40伏/伏增益
  DRV8301_ShuntAmpGain_80VpV = 3 << 2    //!< 80伏/伏增益
} DRV8301_ShuntAmpGain_e;


//! \brief 分流放大器通道编号枚举
//!
typedef enum
{
  DRV8301_ShuntAmpNumber_1 = 1,      //!< 分流放大器通道1
  DRV8301_ShuntAmpNumber_2 = 2       //!< 分流放大器通道2
} DRV8301_ShuntAmpNumber_e;

//! \brief 过流检测阈值(Vds电压)配置枚举
//!
typedef enum
{
  DRV8301_VdsLevel_0p060_V =  0 << 6,      //!< Vds阈值 = 0.060 伏
  DRV8301_VdsLevel_0p068_V =  1 << 6,      //!< Vds阈值 = 0.068 伏
  DRV8301_VdsLevel_0p076_V =  2 << 6,      //!< Vds阈值 = 0.076 伏
  DRV8301_VdsLevel_0p086_V =  3 << 6,      //!< Vds阈值 = 0.086 伏
  DRV8301_VdsLevel_0p097_V =  4 << 6,      //!< Vds阈值 = 0.097 伏
  DRV8301_VdsLevel_0p109_V =  5 << 6,      //!< Vds阈值 = 0.109 伏
  DRV8301_VdsLevel_0p123_V =  6 << 6,      //!< Vds阈值 = 0.123 伏
  DRV8301_VdsLevel_0p138_V =  7 << 6,      //!< Vds阈值 = 0.138 伏
  DRV8301_VdsLevel_0p155_V =  8 << 6,      //!< Vds阈值 = 0.155 伏
  DRV8301_VdsLevel_0p175_V =  9 << 6,      //!< Vds阈值 = 0.175 伏
  DRV8301_VdsLevel_0p197_V = 10 << 6,      //!< Vds阈值 = 0.197 伏
  DRV8301_VdsLevel_0p222_V = 11 << 6,      //!< Vds阈值 = 0.222 伏
  DRV8301_VdsLevel_0p250_V = 12 << 6,      //!< Vds阈值 = 0.250 伏
  DRV8301_VdsLevel_0p282_V = 13 << 6,      //!< Vds阈值 = 0.282 伏
  DRV8301_VdsLevel_0p317_V = 14 << 6,      //!< Vds阈值 = 0.317 伏
  DRV8301_VdsLevel_0p358_V = 15 << 6,      //!< Vds阈值 = 0.358 伏
  DRV8301_VdsLevel_0p403_V = 16 << 6,      //!< Vds阈值 = 0.403 伏
  DRV8301_VdsLevel_0p454_V = 17 << 6,      //!< Vds阈值 = 0.454 伏
  DRV8301_VdsLevel_0p511_V = 18 << 6,      //!< Vds阈值 = 0.511 伏
  DRV8301_VdsLevel_0p576_V = 19 << 6,      //!< Vds阈值 = 0.576 伏
  DRV8301_VdsLevel_0p648_V = 20 << 6,      //!< Vds阈值 = 0.648 伏
  DRV8301_VdsLevel_0p730_V = 21 << 6,      //!< Vds阈值 = 0.730 伏
  DRV8301_VdsLevel_0p822_V = 22 << 6,      //!< Vds阈值 = 0.822 伏
  DRV8301_VdsLevel_0p926_V = 23 << 6,      //!< Vds阈值 = 0.926 伏
  DRV8301_VdsLevel_1p043_V = 24 << 6,      //!< Vds阈值 = 1.403 伏
  DRV8301_VdsLevel_1p175_V = 25 << 6,      //!< Vds阈值 = 1.175 伏
  DRV8301_VdsLevel_1p324_V = 26 << 6,      //!< Vds阈值 = 1.324 伏
  DRV8301_VdsLevel_1p491_V = 27 << 6,      //!< Vds阈值 = 1.491 伏
  DRV8301_VdsLevel_1p679_V = 28 << 6,      //!< Vds阈值 = 1.679 伏
  DRV8301_VdsLevel_1p892_V = 29 << 6,      //!< Vds阈值 = 1.892 伏
  DRV8301_VdsLevel_2p131_V = 30 << 6,      //!< Vds阈值 = 2.131 伏
  DRV8301_VdsLevel_2p400_V = 31 << 6       //!< Vds阈值 = 2.400 伏
} DRV8301_VdsLevel_e;

// SPI输出数据选择枚举
typedef enum
{
  DRV8301_GETID=0  //!< 获取设备ID
} Drv8301SpiOutputDataSelect_e;

// DRV8301状态寄存器1结构定义
typedef struct _DRV_SPI_8301_Stat1_t_
{
  bool                  FAULT;      //!< 故障标志
  bool                  GVDD_UV;    //!< 门极驱动欠压标志
  bool                  PVDD_UV;    //!< 电源欠压标志
  bool                  OTSD;       //!< 过温关断标志
  bool                  OTW;        //!< 过温警告标志
  bool                  FETHA_OC;   //!< A相高侧过流标志
  bool                  FETLA_OC;   //!< A相低侧过流标志
  bool                  FETHB_OC;   //!< B相高侧过流标志
  bool                  FETLB_OC;   //!< B相低侧过流标志
  bool                  FETHC_OC;   //!< C相高侧过流标志
  bool                  FETLC_OC;   //!< C相低侧过流标志
}DRV_SPI_8301_Stat1_t_;

// DRV8301状态寄存器2结构定义
typedef struct _DRV_SPI_8301_Stat2_t_
{
  bool                  GVDD_OV;    //!< 门极驱动过压标志
  uint16_t              DeviceID;   //!< 设备ID值
}DRV_SPI_8301_Stat2_t_;

// DRV8301控制寄存器1结构定义
typedef struct _DRV_SPI_8301_CTRL1_t_
{
  DRV8301_PeakCurrent_e    DRV8301_CURRENT; //!< 门极驱动电流设置
  DRV8301_Reset_e          DRV8301_RESET;   //!< 复位控制状态
  DRV8301_PwmMode_e        PWM_MODE;        //!< PWM输入模式选择
  DRV8301_OcMode_e         OC_MODE;         //!< 过流保护模式
  DRV8301_VdsLevel_e       OC_ADJ_SET;      //!< 过流检测阈值
}DRV_SPI_8301_CTRL1_t_;

// DRV8301控制寄存器2结构定义
typedef struct _DRV_SPI_8301_CTRL2_t_
{
  DRV8301_OcTwMode_e       OCTW_SET;       //!< 过流/过温警告设置
  DRV8301_ShuntAmpGain_e   GAIN;           //!< 电流检测增益设置
  DRV8301_DcCalMode_e      DC_CAL_CH1p2;    //!< 直流校准模式
  DRV8301_OcOffTimeMode_e  OC_TOFF;        //!< 过流关断时间控制
}DRV_SPI_8301_CTRL2_t_;

// DRV8301变量结构定义
typedef struct _DRV_SPI_8301_Vars_t_
{
  DRV_SPI_8301_Stat1_t_     Stat_Reg_1;  //!< 状态寄存器1值
  DRV_SPI_8301_Stat2_t_     Stat_Reg_2;  //!< 状态寄存器2值
  DRV_SPI_8301_CTRL1_t_     Ctrl_Reg_1;  //!< 控制寄存器1值
  DRV_SPI_8301_CTRL2_t_     Ctrl_Reg_2;  //!< 控制寄存器2值
  bool                  SndCmd;         //!< 发送命令标志
  bool                  RcvCmd;         //!< 接收命令标志
}DRV_SPI_8301_Vars_t;

// //! \brief DRV8301对象结构定义
// //!
// typedef struct _DRV8301_Obj_
// {
//   SPI_Handle       spiHandle;        //!< SPI通信接口句柄
//   GPIO_Handle      EngpioHandle;     //!< 使能引脚GPIO句柄
//   GPIO_Number_e    EngpioNumber;     //!< 使能引脚编号
//   GPIO_Handle      nCSgpioHandle;    //!< 片选引脚GPIO句柄
//   GPIO_Number_e    nCSgpioNumber;    //!< 片选引脚编号
//   bool             RxTimeOut;        //!< 接收超时标志
//   bool             enableTimeOut;    //!< 使能超时标志
// } DRV8301_Obj;

//! \brief DRV8301对象句柄类型定义
//!
typedef struct _DRV8301_Obj_ *DRV8301_Handle;

//! \brief DRV8301数据字类型定义(16位)
//!
typedef  uint16_t    DRV8301_Word_t;


//! \brief Defines the DRV8301 Word type
//!
typedef  uint16_t    DRV8301_Word_t;

//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8301_Word_t DRV8301_buildCtrlWord(const DRV8301_CtrlMode_e ctrlMode,
                                                   const DRV8301_RegName_e regName,
                                                   const uint16_t data)
{
    DRV8301_Word_t ctrlWord = ctrlMode | regName | (data & DRV8301_DATA_MASK);

    return(ctrlWord);
} // end of DRV8301_buildCtrlWord() function

#define M0_nCS_SET GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);//1
#define M0_nCS_CLR GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);//0\

#define M1_nCS_SET GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_SET);//1
#define M1_nCS_CLR GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_RESET);//0


#define EN_GATE_SET GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
#define EN_GATE_CLR GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);


void Mx_nCS_Init(void);
void SPI_init(void);
void SPI_SetSpeed(u8 SpeedSet); //设置SPI1速度
u16 SPI_ReadWriteByte(u16 TxData);//SPI1总线读写一个字节
uint16_t DRV8301_readSpi(const DRV8301_RegName_e regName);
void DRV8301_writeSpi(const DRV8301_RegName_e regName,const uint16_t data);
uint16_t m0_spi_drv8303_read(uint16_t TxData);
uint16_t drv8301_spi_read(uint8_t address);
void drv8301_spi_write(uint8_t address,uint16_t data);
void DRV8301_readData(DRV_SPI_8301_Vars_t *Spi_8301_Vars) ;
void DRV8301_writeData( DRV_SPI_8301_Vars_t *Spi_8301_Vars);
#endif // end of _DRV8301_H_ definition





