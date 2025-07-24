//
// Created by AerialBird on 25-7-4.
//

#ifndef MT6835_H
#define MT6835_H
#include "sys.h"
#include "delay.h"
#define MT6835_CS_SET GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_SET);//1
#define MT6835_CS_CLR GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET);//0\

typedef  uint16_t   MT6835_Word_t;
#define MT6835_DATA_MASK               (0x07FF)

typedef enum
{
    MT6835_CtrlMode_Read_register = 3 << 11,
    MT6835_CtrlMode_Write_register = 0 << 15,
    MT6835_CtrlMode_Burn_EEPROM = 1 << 14,
    MT6835_CtrlMode_Zero_point_register=1,
    MT6835_CtrlMode_Angle_register=1,
  } MT6835_CtrlMode_e;

typedef enum
{
    MT6835_RegName_Angle_register  = 3,
    MT6835_RegName_Status_2  = 1 << 11,
    MT6835_RegName_Control_1 = 2 << 11,
    MT6835_RegName_Control_2 = 3 << 11
  } MT6835_RegName_e;
static inline MT6835_Word_t MT6835_buildCtrlWord(const MT6835_CtrlMode_e ctrlMode,
                                                 const MT6835_RegName_e regName,
                                                 const uint16_t data)
{
    MT6835_Word_t ctrlWord = ctrlMode | regName| (data &MT6835_DATA_MASK);

    return(ctrlWord);
}

#define Read			0x3000
#define Write 			0x6000
#define WriteEEPROM		0xC000
#define SetZeroPoint	0x5000
#define ContinuousRead	0xA000
void MT6835_CS(void);
void SPI_MT6835_init(void);
void SPI_MT6835_SetSpeed(u8 SPI_BaudRatePrescaler);
uint16_t MT6835_readSpi(const MT6835_RegName_e regName);
uint8_t SPI_MT6835_ReadWriteByte(uint8_t TxData);
uint8_t ReadMT6835(uint16_t add);

#endif //MT6835_H
