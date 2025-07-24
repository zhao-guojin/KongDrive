
#ifndef _DRV8301_H_
#define _DRV8301_H_

#include "stdbool.h"
#include "stdint.h"
#include "sys.h"

// **************************************************************************
// Ԥ�����

//! \brief �����ַ����
//!
#define DRV8301_ADDR_MASK               (0x7800)

//! \brief ������������
//!
#define DRV8301_DATA_MASK               (0x07FF)

//! \brief �����/д����
//!
#define DRV8301_RW_MASK                 (0x8000)

//! \brief ���������������
//!
#define DRV8301_FAULT_TYPE_MASK         (0x07FF)

//! \brief ����״̬�Ĵ���1��FETLC_OC(C��Ͳ�MOSFET����)��־λλ��
//!
#define DRV8301_STATUS1_FETLC_OC_BITS   (1 << 0)

//! \brief ����״̬�Ĵ���1��FETHC_OC(C��߲�MOSFET����)��־λλ��
//!
#define DRV8301_STATUS1_FETHC_OC_BITS   (1 << 1)

//! \brief ����״̬�Ĵ���1��FETLB_OC(B��Ͳ�MOSFET����)��־λλ��
//!
#define DRV8301_STATUS1_FETLB_OC_BITS   (1 << 2)

//! \brief ����״̬�Ĵ���1��FETHB_OC(B��߲�MOSFET����)��־λλ��
//!
#define DRV8301_STATUS1_FETHB_OC_BITS   (1 << 3)

//! \brief ����״̬�Ĵ���1��FETLA_OC(A��Ͳ�MOSFET����)��־λλ��
//!
#define DRV8301_STATUS1_FETLA_OC_BITS   (1 << 4)

//! \brief ����״̬�Ĵ���1��FETHA_OC(A��߲�MOSFET����)��־λλ��
//!
#define DRV8301_STATUS1_FETHA_OC_BITS   (1 << 5)

//! \brief ����״̬�Ĵ���1��OTW(���¾���)��־λλ��
//!
#define DRV8301_STATUS1_OTW_BITS        (1 << 6)

//! \brief ����״̬�Ĵ���1��OTSD(���¹ض�)��־λλ��
//!
#define DRV8301_STATUS1_OTSD_BITS       (1 << 7)

//! \brief ����״̬�Ĵ���1��PVDD_UV(��Դ��ѹǷѹ)��־λλ��
//!
#define DRV8301_STATUS1_PVDD_UV_BITS    (1 << 8)

//! \brief ����״̬�Ĵ���1��GVDD_UV(�ż�������ѹǷѹ)��־λλ��
//!
#define DRV8301_STATUS1_GVDD_UV_BITS    (1 << 9)

//! \brief ����״̬�Ĵ���1��FAULT(����)��־λλ��
//!
#define DRV8301_STATUS1_FAULT_BITS      (1 << 10)

//! \brief ����״̬�Ĵ���2��Device ID(�豸ID)λλ��
//!
#define DRV8301_STATUS2_ID_BITS        (15 << 0)

//! \brief ����״̬�Ĵ���2��GVDD_OV(�ż�������ѹ��ѹ)��־λλ��
//!
#define DRV8301_STATUS2_GVDD_OV_BITS    (1 << 7)

//! \brief ������ƼĴ���1��GATE_CURRENT(�ż���������)����λλ��
//!
#define DRV8301_CTRL1_GATE_CURRENT_BITS  (3 << 0)

//! \brief ������ƼĴ���1��GATE_RESET(��λ״̬)����λλ��
//!
#define DRV8301_CTRL1_GATE_RESET_BITS    (1 << 2)

//! \brief ������ƼĴ���1��PWM_MODE(PWMģʽ)����λλ��
//!
#define DRV8301_CTRL1_PWM_MODE_BITS      (1 << 3)

//! \brief ������ƼĴ���1��OC_MODE(��������ģʽ)����λλ��
//!
#define DRV8301_CTRL1_OC_MODE_BITS       (3 << 4)

//! \brief ������ƼĴ���1��OC_ADJ_SET(���������ֵ)����λλ��
//!
#define DRV8301_CTRL1_OC_ADJ_SET_BITS   (31 << 6)

//! \brief ������ƼĴ���2��OCTW_SET(����/������Ϊ)����λλ��
//!
#define DRV8301_CTRL2_OCTW_SET_BITS      (3 << 0)

//! \brief ������ƼĴ���2��GAIN(�����������)����λλ��
//!
#define DRV8301_CTRL2_GAIN_BITS          (3 << 2)

//! \brief ������ƼĴ���2��DC_CAL_1(ֱ��У׼ͨ��1)����λλ��
//!
#define DRV8301_CTRL2_DC_CAL_1_BITS      (1 << 4)

//! \brief ������ƼĴ���2��DC_CAL_2(ֱ��У׼ͨ��2)����λλ��
//!
#define DRV8301_CTRL2_DC_CAL_2_BITS      (1 << 5)

//! \brief ������ƼĴ���2��OC_TOFF(�����ض��ӳ�)����λλ��
//!
#define DRV8301_CTRL2_OC_TOFF_BITS       (1 << 6)

// **************************************************************************
// ���Ͷ���ö��

//! \brief ��дģʽö��
//!
typedef enum
{
  DRV8301_CtrlMode_Read = 1 << 15,   //!< ��ģʽ
  DRV8301_CtrlMode_Write = 0 << 15    //!< дģʽ
} DRV8301_CtrlMode_e;


//! \brief ֱ��У׼ģʽö��
//!
typedef enum
{
  DRV8301_DcCalMode_Ch1_Load   = (0 << 4),   //!< �����Ŵ���1ͨ�������������Ӹ���
  DRV8301_DcCalMode_Ch1_NoLoad = (1 << 4),   //!< �����Ŵ���1�Ͽ����أ��������Ŷ�·
  DRV8301_DcCalMode_Ch2_Load   = (0 << 5),   //!< �����Ŵ���2ͨ�������������Ӹ���
  DRV8301_DcCalMode_Ch2_NoLoad = (1 << 5)    //!< �����Ŵ���2�Ͽ����أ��������Ŷ�·
} DRV8301_DcCalMode_e;


//! \brief ��������ö��
//!
typedef enum
{
  DRV8301_FaultType_NoFault  = (0 << 0),  //!< �޹���
  DRV8301_FaultType_FETLC_OC = (1 << 0),  //!< C��Ͳ�MOSFET��������
  DRV8301_FaultType_FETHC_OC = (1 << 1),  //!< C��߲�MOSFET��������
  DRV8301_FaultType_FETLB_OC = (1 << 2),  //!< B��Ͳ�MOSFET��������
  DRV8301_FaultType_FETHB_OC = (1 << 3),  //!< B��߲�MOSFET��������
  DRV8301_FaultType_FETLA_OC = (1 << 4),  //!< A��Ͳ�MOSFET��������
  DRV8301_FaultType_FETHA_OC = (1 << 5),  //!< A��߲�MOSFET��������
  DRV8301_FaultType_OTW      = (1 << 6),  //!< ����Ԥ������
  DRV8301_FaultType_OTSD     = (1 << 7),  //!< ���¹ضϹ���
  DRV8301_FaultType_PVDD_UV  = (1 << 8),  //!< ����ԴǷѹ����
  DRV8301_FaultType_GVDD_UV  = (1 << 9),  //!< �ż�����Ƿѹ����
  DRV8301_FaultType_GVDD_OV  = (1 << 10)  //!< �ż�������ѹ����
} DRV8301_FaultType_e;


//! \brief ��������ģʽö��
//!
typedef enum
{
  DRV8301_OcMode_CurrentLimit  = 0 << 4,   //!< ��⵽����ʱ�����޷�
  DRV8301_OcMode_LatchShutDown = 1 << 4,   //!< ��⵽����ʱ����ض�
  DRV8301_OcMode_ReportOnly    = 2 << 4,   //!< ��⵽����ʱ������
  DRV8301_OcMode_Disabled      = 3 << 4    //!< ���ù�������
} DRV8301_OcMode_e;


//! \brief �����ض�ʱ��ģʽö��
//!
typedef enum
{
  DRV8301_OcOffTimeMode_Normal  = 0 << 6,   //!< ���������ڱ���
  DRV8301_OcOffTimeMode_Ctrl    = 1 << 6    //!< �����ڼ�ض�ʱ��ɿ�
} DRV8301_OcOffTimeMode_e;


//! \brief ����/���¾���ģʽö��
//!
typedef enum
{
  DRV8301_OcTwMode_Both    = 0 << 0,   //!< ��/OCTW����ͬʱ������º͹���
  DRV8301_OcTwMode_OT_Only = 1 << 0,   //!< ��/OCTW���Ž��������
  DRV8301_OcTwMode_OC_Only = 2 << 0    //!< ��/OCTW���Ž��������
} DRV8301_OcTwMode_e;


//! \brief ��ֵ�����ȼ�ö��
//!
typedef enum
{
  DRV8301_PeakCurrent_1p70_A  = 0 << 0,   //!< ��������ֵ����1.70��
  DRV8301_PeakCurrent_0p70_A  = 1 << 0,   //!< ��������ֵ����0.70��
  DRV8301_PeakCurrent_0p25_A  = 2 << 0    //!< ��������ֵ����0.25��
} DRV8301_PeakCurrent_e;


//! \brief PWM����ģʽö��
//!
typedef enum
{
  DRV8301_PwmMode_Six_Inputs   = 0 << 3,   //!< 6·����PWM����
  DRV8301_PwmMode_Three_Inputs = 1 << 3    //!< 3·����PWM����
} DRV8301_PwmMode_e;


//! \brief �Ĵ�������ö��
//!
typedef enum
{
  DRV8301_RegName_Status_1  = 0 << 11,   //!< ״̬�Ĵ���1
  DRV8301_RegName_Status_2  = 1 << 11,   //!< ״̬�Ĵ���2
  DRV8301_RegName_Control_1 = 2 << 11,  //!< ���ƼĴ���1
  DRV8301_RegName_Control_2 = 3 << 11   //!< ���ƼĴ���2
} DRV8301_RegName_e;


//! \brief ��λ״̬ö��
//!
typedef enum
{
  DRV8301_Reset_Normal = 0 << 2,   //!< ��������ģʽ
  DRV8301_Reset_All = 1 << 2       //!< ȫ�Ĵ�����λ
} DRV8301_Reset_e;


//! \brief �������Ŵ�������ö��
//!
typedef enum
{
  DRV8301_ShuntAmpGain_10VpV = 0 << 2,   //!< 10��/������
  DRV8301_ShuntAmpGain_20VpV = 1 << 2,   //!< 20��/������
  DRV8301_ShuntAmpGain_40VpV = 2 << 2,   //!< 40��/������
  DRV8301_ShuntAmpGain_80VpV = 3 << 2    //!< 80��/������
} DRV8301_ShuntAmpGain_e;


//! \brief �����Ŵ���ͨ�����ö��
//!
typedef enum
{
  DRV8301_ShuntAmpNumber_1 = 1,      //!< �����Ŵ���ͨ��1
  DRV8301_ShuntAmpNumber_2 = 2       //!< �����Ŵ���ͨ��2
} DRV8301_ShuntAmpNumber_e;

//! \brief ���������ֵ(Vds��ѹ)����ö��
//!
typedef enum
{
  DRV8301_VdsLevel_0p060_V =  0 << 6,      //!< Vds��ֵ = 0.060 ��
  DRV8301_VdsLevel_0p068_V =  1 << 6,      //!< Vds��ֵ = 0.068 ��
  DRV8301_VdsLevel_0p076_V =  2 << 6,      //!< Vds��ֵ = 0.076 ��
  DRV8301_VdsLevel_0p086_V =  3 << 6,      //!< Vds��ֵ = 0.086 ��
  DRV8301_VdsLevel_0p097_V =  4 << 6,      //!< Vds��ֵ = 0.097 ��
  DRV8301_VdsLevel_0p109_V =  5 << 6,      //!< Vds��ֵ = 0.109 ��
  DRV8301_VdsLevel_0p123_V =  6 << 6,      //!< Vds��ֵ = 0.123 ��
  DRV8301_VdsLevel_0p138_V =  7 << 6,      //!< Vds��ֵ = 0.138 ��
  DRV8301_VdsLevel_0p155_V =  8 << 6,      //!< Vds��ֵ = 0.155 ��
  DRV8301_VdsLevel_0p175_V =  9 << 6,      //!< Vds��ֵ = 0.175 ��
  DRV8301_VdsLevel_0p197_V = 10 << 6,      //!< Vds��ֵ = 0.197 ��
  DRV8301_VdsLevel_0p222_V = 11 << 6,      //!< Vds��ֵ = 0.222 ��
  DRV8301_VdsLevel_0p250_V = 12 << 6,      //!< Vds��ֵ = 0.250 ��
  DRV8301_VdsLevel_0p282_V = 13 << 6,      //!< Vds��ֵ = 0.282 ��
  DRV8301_VdsLevel_0p317_V = 14 << 6,      //!< Vds��ֵ = 0.317 ��
  DRV8301_VdsLevel_0p358_V = 15 << 6,      //!< Vds��ֵ = 0.358 ��
  DRV8301_VdsLevel_0p403_V = 16 << 6,      //!< Vds��ֵ = 0.403 ��
  DRV8301_VdsLevel_0p454_V = 17 << 6,      //!< Vds��ֵ = 0.454 ��
  DRV8301_VdsLevel_0p511_V = 18 << 6,      //!< Vds��ֵ = 0.511 ��
  DRV8301_VdsLevel_0p576_V = 19 << 6,      //!< Vds��ֵ = 0.576 ��
  DRV8301_VdsLevel_0p648_V = 20 << 6,      //!< Vds��ֵ = 0.648 ��
  DRV8301_VdsLevel_0p730_V = 21 << 6,      //!< Vds��ֵ = 0.730 ��
  DRV8301_VdsLevel_0p822_V = 22 << 6,      //!< Vds��ֵ = 0.822 ��
  DRV8301_VdsLevel_0p926_V = 23 << 6,      //!< Vds��ֵ = 0.926 ��
  DRV8301_VdsLevel_1p043_V = 24 << 6,      //!< Vds��ֵ = 1.403 ��
  DRV8301_VdsLevel_1p175_V = 25 << 6,      //!< Vds��ֵ = 1.175 ��
  DRV8301_VdsLevel_1p324_V = 26 << 6,      //!< Vds��ֵ = 1.324 ��
  DRV8301_VdsLevel_1p491_V = 27 << 6,      //!< Vds��ֵ = 1.491 ��
  DRV8301_VdsLevel_1p679_V = 28 << 6,      //!< Vds��ֵ = 1.679 ��
  DRV8301_VdsLevel_1p892_V = 29 << 6,      //!< Vds��ֵ = 1.892 ��
  DRV8301_VdsLevel_2p131_V = 30 << 6,      //!< Vds��ֵ = 2.131 ��
  DRV8301_VdsLevel_2p400_V = 31 << 6       //!< Vds��ֵ = 2.400 ��
} DRV8301_VdsLevel_e;

// SPI�������ѡ��ö��
typedef enum
{
  DRV8301_GETID=0  //!< ��ȡ�豸ID
} Drv8301SpiOutputDataSelect_e;

// DRV8301״̬�Ĵ���1�ṹ����
typedef struct _DRV_SPI_8301_Stat1_t_
{
  bool                  FAULT;      //!< ���ϱ�־
  bool                  GVDD_UV;    //!< �ż�����Ƿѹ��־
  bool                  PVDD_UV;    //!< ��ԴǷѹ��־
  bool                  OTSD;       //!< ���¹ضϱ�־
  bool                  OTW;        //!< ���¾����־
  bool                  FETHA_OC;   //!< A��߲������־
  bool                  FETLA_OC;   //!< A��Ͳ������־
  bool                  FETHB_OC;   //!< B��߲������־
  bool                  FETLB_OC;   //!< B��Ͳ������־
  bool                  FETHC_OC;   //!< C��߲������־
  bool                  FETLC_OC;   //!< C��Ͳ������־
}DRV_SPI_8301_Stat1_t_;

// DRV8301״̬�Ĵ���2�ṹ����
typedef struct _DRV_SPI_8301_Stat2_t_
{
  bool                  GVDD_OV;    //!< �ż�������ѹ��־
  uint16_t              DeviceID;   //!< �豸IDֵ
}DRV_SPI_8301_Stat2_t_;

// DRV8301���ƼĴ���1�ṹ����
typedef struct _DRV_SPI_8301_CTRL1_t_
{
  DRV8301_PeakCurrent_e    DRV8301_CURRENT; //!< �ż�������������
  DRV8301_Reset_e          DRV8301_RESET;   //!< ��λ����״̬
  DRV8301_PwmMode_e        PWM_MODE;        //!< PWM����ģʽѡ��
  DRV8301_OcMode_e         OC_MODE;         //!< ��������ģʽ
  DRV8301_VdsLevel_e       OC_ADJ_SET;      //!< ���������ֵ
}DRV_SPI_8301_CTRL1_t_;

// DRV8301���ƼĴ���2�ṹ����
typedef struct _DRV_SPI_8301_CTRL2_t_
{
  DRV8301_OcTwMode_e       OCTW_SET;       //!< ����/���¾�������
  DRV8301_ShuntAmpGain_e   GAIN;           //!< ���������������
  DRV8301_DcCalMode_e      DC_CAL_CH1p2;    //!< ֱ��У׼ģʽ
  DRV8301_OcOffTimeMode_e  OC_TOFF;        //!< �����ض�ʱ�����
}DRV_SPI_8301_CTRL2_t_;

// DRV8301�����ṹ����
typedef struct _DRV_SPI_8301_Vars_t_
{
  DRV_SPI_8301_Stat1_t_     Stat_Reg_1;  //!< ״̬�Ĵ���1ֵ
  DRV_SPI_8301_Stat2_t_     Stat_Reg_2;  //!< ״̬�Ĵ���2ֵ
  DRV_SPI_8301_CTRL1_t_     Ctrl_Reg_1;  //!< ���ƼĴ���1ֵ
  DRV_SPI_8301_CTRL2_t_     Ctrl_Reg_2;  //!< ���ƼĴ���2ֵ
  bool                  SndCmd;         //!< ���������־
  bool                  RcvCmd;         //!< ���������־
}DRV_SPI_8301_Vars_t;

// //! \brief DRV8301����ṹ����
// //!
// typedef struct _DRV8301_Obj_
// {
//   SPI_Handle       spiHandle;        //!< SPIͨ�Žӿھ��
//   GPIO_Handle      EngpioHandle;     //!< ʹ������GPIO���
//   GPIO_Number_e    EngpioNumber;     //!< ʹ�����ű��
//   GPIO_Handle      nCSgpioHandle;    //!< Ƭѡ����GPIO���
//   GPIO_Number_e    nCSgpioNumber;    //!< Ƭѡ���ű��
//   bool             RxTimeOut;        //!< ���ճ�ʱ��־
//   bool             enableTimeOut;    //!< ʹ�ܳ�ʱ��־
// } DRV8301_Obj;

//! \brief DRV8301���������Ͷ���
//!
typedef struct _DRV8301_Obj_ *DRV8301_Handle;

//! \brief DRV8301���������Ͷ���(16λ)
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
void SPI_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�
u16 SPI_ReadWriteByte(u16 TxData);//SPI1���߶�дһ���ֽ�
uint16_t DRV8301_readSpi(const DRV8301_RegName_e regName);
void DRV8301_writeSpi(const DRV8301_RegName_e regName,const uint16_t data);
uint16_t m0_spi_drv8303_read(uint16_t TxData);
uint16_t drv8301_spi_read(uint8_t address);
void drv8301_spi_write(uint8_t address,uint16_t data);
void DRV8301_readData(DRV_SPI_8301_Vars_t *Spi_8301_Vars) ;
void DRV8301_writeData( DRV_SPI_8301_Vars_t *Spi_8301_Vars);
#endif // end of _DRV8301_H_ definition





