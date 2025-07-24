#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "button.h"
#include "lcd_init.h"
#include "lcd.h"
#include "pic.h"
#include "drv8301.h"
#include "TIM.h"
#include "ADC.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc_vcp.h"
#include "foc_algorithm.h"
#include "MT6835.h"
#include "PWM.h"
USB_OTG_CORE_HANDLE USB_OTG_dev;


// extern u16 USB_USART_RX_STA;

//	LCD_ShowString(30,50,200,16,16,"EXPLORER F407");
//	LCD_ShowString(30,70,200,16,16,"USB Virtual USART TEST");
//	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
//	LCD_ShowString(30,110,200,16,16,"2019/9/3");
// 	LCD_ShowString(30,130,200,16,16,"USB Connecting...");//ÌáÊ¾USB¿ªÊ¼Á¬½Ó
// typedef union
// {
// 	float v_f;
// 	uint8_t v_u8[4];
// }typedefVofa;
// typedefVofa sendarray[4];
// float cnt=0;
int main(void)
{
	// DRV_SPI_8301_Vars_t *Spi_8301_Vars;
	// uint16_t zerobuff = 0;
	// __IO uint32_t i=0;
	// sendarray[3].v_u8[0] = 0x00;
	// sendarray[3].v_u8[1] = 0x00;
	// sendarray[3].v_u8[2] = 0x80;
	// sendarray[3].v_u8[3] = 0x7f;
	// // uint16_t adcx = 0;
	// // float voltage = 0;
	// // uint16_t recbuff = 0xbeef;
	// // delay_init(168);
	// // 串口4初始化
	uart_init(115200);
	// Adc_Init();
	// // 按键初始化
	// // Button_Init();
	// // TIM3_Int_Init();
	// USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);
	// // LCD初始化
	// // LCD_Init();
	// // LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
	// // printf("UART4 test \r\n");
	// // delay_ms(500);
	// // LCD_ShowPicture(160, 95, 40, 40, gImage_1);
	// // Mx_nCS_Init();
	// // SPI_init();
	// // EN_GATE_SET;
	// // delay_ms(1);
	// float angle_temp = 0.0f;
	// float angle_temp_temp = 0.0f;
	// TRANSF_COS_SIN_DEF cos_sin_temp;
	// VOLTAGE_DQ_DEF v_dq_temp;
	// VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp;
	// real32_T Udc_temp=25.0f;
	// real32_T Tpwm_temp=2500.0f;
	// uint16_t c=0;
	// MT6835_CS();
	// SPI_MT6835_init();
	// real32_T CNT=0.0f;
	// uint16_t CNT_temp=0.0f;
	// PWM_Transf PWM_={0};
	// uint16_t recbuf=0;
	// uint16_t tx_value = 0, rx_value = 0;
	// tx_value = 0xC000;
	while (1)
	{
		// v_dq_temp.Vd=0.0f;
		// v_dq_temp.Vq=5.0f;
		// recbuf= ReadMT6835(0x03);
		// printf("%x\r\n",recbuf);
		// printf("%f\r\n",0.3f);
		// delay_us(1);
		// SPI_MT6835_ReadWriteByte(tx_value);
		// // 4. 片选信号特殊处理（实测需要脉冲）
		// // 数据手册说明不需要脉冲，但实际硬件需要
		// MT6835_CS_SET;
		// // delay_us(1);
		// MT6835_CS_CLR;
		// // delay_us(1);
		// // 5. 执行SPI收发操作
		// recbuf=SPI_MT6835_ReadWriteByte(tx_value);
		// // 6. 释放片选信号
		// MT6835_CS_SET;
		// delay_ms(10);
	 //    printf("%x\r\n",recbuf);
		// if(KEY_UP == 1)printf("KEY_UP \r\n");
		// if(KEY_DOWN == 1)printf("KEY_DOWN \r\n");
		// if(KEY_RIGHT == 1)printf("KEY_RIGHT \r\n");
		// if(KEY_LEFT == 1)printf("KEY_LEFT \r\n");
		// if(KEY_CENTER == 1)printf("KEY_CENTER \r\n");
		// usb_printf("这是\r\n");
		// usb_printf("虚拟串口实验\r\n\r\n");
		// adcx=Get_Adc_Average(ADC_Channel_10,20);
		// voltage=(float)adcx/4095*3.3f;
		//
		// DRV8301_writeSpi(DRV8301_RegName_Control_1,0x1101);
		// printf("recbuff = %x \r\n",DRV8301_readSpi(DRV8301_RegName_Control_1));
		// c=MT6835_buildCtrlWord(MT6835_CtrlMode_Read_register,MT6835_RegName_Angle_register,0);
		// MT6835_CS_CLR
		// c=SPI_MT6835_ReadWriteByte(ContinuousRead|0x0003);
		// MT6835_CS_SET
		// printf("%x\r\n",c);
		// printf("circle_count_motor1:%.3f\r\n",voltage);
		// if (i++ == 0x010000)
		// {
		// 	angle_temp=angle_temp+0.01f;
		//
		// 	if (angle_temp>2*M_PI)
		// 	{
		// 		angle_temp=0.0f;
		// 	}
		// 	if (CNT==0)
		// 	{
		// 		CNT_temp=0;
		// 	}
		// 	if (CNT==25000)
		// 	{
		// 		CNT_temp=1;
		// 	}
		// 	if (CNT_temp==0)
		// 	{
		// 		CNT=CNT+1;
		// 	}
		// 	if (CNT_temp==1)
		// 	{
		// 		CNT=CNT-1;
		// 	}
		// 	Angle_To_Cos_Sin(angle_temp,&cos_sin_temp);
		// 	Rev_Park_Transf(v_dq_temp,cos_sin_temp,&v_alpha_beta_temp);
		// 	SVPWM_Calc(v_alpha_beta_temp,Udc_temp,Tpwm_temp);
		// 	PWM_Transfs(FOC_Output,CNT,PWM_);
		// 	sendarray[0].v_f = (float)PWM_.CH1;
		// 	sendarray[1].v_f = FOC_Output.Tcmp1;
		// 	sendarray[2].v_f = FOC_Output.Tcmp3;
		// 	USB_TX(&sendarray[0].v_u8[0],4*4);
		//
		//
		// 	i = 0;
		// }


			// usb_printf("a:%.3f\n\r",cnt);

	}

	// /* 3. 初始化三相 PWM */
	// uint16_t prescaler = 0;                             // PSC = 0
	// uint16_t period    = 16800;         // ARR ≈ 8399，实现 20 kHz
	// uint8_t  deadTime  = 100;                           // 死区 100 个时钟
	// PWM3Phase_TIM1_Init(period, prescaler, deadTime);
	//
	// /* 4. 打印启动提示 */
	// printf("PWM 驱动初始化完成：ARR=%u, PSC=%u, DeadTime=%u\r\n",
	// 	   (unsigned)period, (unsigned)prescaler, (unsigned)deadTime);
	//
	//
	// while (1)
	// {
	//
	//
	// 		/* 读取并打印 CCR1/CCR2/CCR3，即三相当前占空比 */
	// 		printf("CCR1=%4u, CCR2=%4u, CCR3=%4u\r\n",
	// 			   TIM1->CCR1, TIM1->CCR2, TIM1->CCR3);
	//
	// 		/* 也可打印 CNT 寄存器查看计数器实时值 */
	// 		// printf("CNT=%u\r\n", TIM1->CNT);
	//
	// }
}
