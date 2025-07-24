#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "sys.h"

#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 135
#define LCD_H 240

#else
#define LCD_W 240
#define LCD_H 135
#endif



//-----------------LCD端口定义---------------- 

#define LCD_SCLK_Clr() GPIO_ResetBits(GPIOE,GPIO_Pin_11)//SCL=SCLK
#define LCD_SCLK_Set() GPIO_SetBits(GPIOE,GPIO_Pin_11)

#define LCD_MOSI_Clr() GPIO_ResetBits(GPIOE,GPIO_Pin_10)//SDA=MOSI
#define LCD_MOSI_Set() GPIO_SetBits(GPIOE,GPIO_Pin_10)

#define LCD_RES_Clr()  GPIO_ResetBits(GPIOE,GPIO_Pin_13)//RES
#define LCD_RES_Set()  GPIO_SetBits(GPIOE,GPIO_Pin_13)

#define LCD_DC_Clr()   GPIO_ResetBits(GPIOE,GPIO_Pin_12)//DC
#define LCD_DC_Set()   GPIO_SetBits(GPIOE,GPIO_Pin_12)
 		     
#define LCD_CS_Clr()   GPIO_ResetBits(GPIOE,GPIO_Pin_14)//CS
#define LCD_CS_Set()   GPIO_SetBits(GPIOE,GPIO_Pin_14)

#define LCD_BLK_Clr()  GPIO_ResetBits(GPIOE,GPIO_Pin_15)//BLK
#define LCD_BLK_Set()  GPIO_SetBits(GPIOE,GPIO_Pin_15)




void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




