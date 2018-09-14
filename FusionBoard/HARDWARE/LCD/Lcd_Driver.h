
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//中景园电子
//店铺地址：http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  文 件 名   : main.c
//  版 本 号   : v2.0
//  作    者   : HuangKai
//  生成日期   : 2014-0101
//  最近修改   : 
//  功能描述   : 1.44寸LCD 4接口演示例程(STM32系列)
/******************************************************************************
//本程序适用与STM32F103C8
//              GND   电源地
//              VCC   接5V或3.3v电源
//              SCL   接PA5（SCL）
//              SDA   接PD7（SDA）
//              RES   接PB0
//              DC    接PB1
//              CS    接PA4 
//							BL		接PB10
*******************************************************************************/
// 修改历史   :
// 日    期   : 
// 作    者   : HuangKai
// 修改内容   : 创建文件
//版权所有，盗版必究。
//Copyright(C) 中景园电子2014/3/16
//All rights reserved
//******************************************************************************/

#define RED  	0xf800
#define GREEN	0x07e0
#define BLUE 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   	//灰色0 3165 00110 001011 00101
#define GRAY1   0x8410      	//灰色1      00000 000000 00000
#define GRAY2   0x4208      	//灰色2  1111111111011111




#define LCD_CTRLA   	  	GPIOA		//定义TFT数据端口
#define LCD_CTRLB   	  	GPIOB		//定义TFT数据端口
#define LCD_CTRLC   	  	GPIOC


#define LCD_SCL        	GPIO_Pins_5	  //PA5--->>TFT --SCL/SCK
#define LCD_SDA        	GPIO_Pins_7 	//PA7--->>TFT --SDA/DIN
#define LCD_RST       	GPIO_Pins_9 	//PA9--->>TFT --RST
#define LCD_RS         	GPIO_Pins_7 	//PC7--->>TFT --RS/DC
#define LCD_CS        	GPIO_Pins_10  //PB10--->>TFT --CS/CE
#define LCD_LED        	GPIO_Pins_1   //PB1--->>TFT --BL

////液晶控制口置1操作语句宏定义
#define	LCD_SCL_SET  	LCD_CTRLA->BSRE=LCD_SCL    
#define	LCD_SDA_SET  	LCD_CTRLA->BSRE=LCD_SDA   
#define	LCD_CS_SET  	LCD_CTRLB->BSRE=LCD_CS  

#define	LCD_LED_SET  	LCD_CTRLB->BSRE=LCD_LED   
#define	LCD_RS_SET  	LCD_CTRLC->BSRE=LCD_RS 
#define	LCD_RST_SET  	LCD_CTRLA->BSRE=LCD_RST 
////液晶控制口置0操作语句宏定义
#define	LCD_SCL_CLR  	LCD_CTRLA->BRE=LCD_SCL  
#define	LCD_SDA_CLR  	LCD_CTRLA->BRE=LCD_SDA 
#define	LCD_CS_CLR  	LCD_CTRLB->BRE=LCD_CS 
    
#define	LCD_LED_CLR  	LCD_CTRLB->BRE=LCD_LED 
#define	LCD_RST_CLR  	LCD_CTRLA->BRE=LCD_RST
#define	LCD_RS_CLR  	LCD_CTRLC->BRE=LCD_RS 

#define LCD_DATAOUT(x) LCD_DATA->OPTDT=x; //数据输出
#define LCD_DATAIN     LCD_DATA->IPTDT;   //数据输入

#define LCD_WR_DATA(data){\
LCD_RS_SET;\
LCD_CS_CLR;\
LCD_DATAOUT(data);\
LCD_WR_CLR;\
LCD_WR_SET;\
LCD_CS_SET;\
} 



void LCD_GPIO_Init(void);
void Lcd_WriteIndex(u8 Index);
void Lcd_WriteData(u8 Data);
void Lcd_WriteReg(u8 Index,u8 Data);
u16 Lcd_ReadReg(u8 LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(u16 Color);
void Lcd_SetXY(u16 x,u16 y);
void Gui_DrawPoint(u16 x,u16 y,u16 Data);
unsigned int Lcd_ReadPoint(u16 x,u16 y);
void Lcd_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end);
void LCD_WriteData_16Bit(u16 Data);
void Lcd_Clear_xy(u16 Color, u8 x, u8 y, u8 xx, u8 yy)  ;

