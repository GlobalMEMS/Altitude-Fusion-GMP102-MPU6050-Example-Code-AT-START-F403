
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//�о�԰����
//���̵�ַ��http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  �� �� ��   : main.c
//  �� �� ��   : v2.0
//  ��    ��   : HuangKai
//  ��������   : 2014-0101
//  ����޸�   : 
//  ��������   : 1.44��LCD 4�ӿ���ʾ����(STM32ϵ��)
/******************************************************************************
//������������STM32F103C8
//              GND   ��Դ��
//              VCC   ��5V��3.3v��Դ
//              SCL   ��PA5��SCL��
//              SDA   ��PD7��SDA��
//              RES   ��PB0
//              DC    ��PB1
//              CS    ��PA4 
//							BL		��PB10
*******************************************************************************/
// �޸���ʷ   :
// ��    ��   : 
// ��    ��   : HuangKai
// �޸�����   : �����ļ�
//��Ȩ���У�����ؾ���
//Copyright(C) �о�԰����2014/3/16
//All rights reserved
//******************************************************************************/

#define RED  	0xf800
#define GREEN	0x07e0
#define BLUE 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   	//��ɫ0 3165 00110 001011 00101
#define GRAY1   0x8410      	//��ɫ1      00000 000000 00000
#define GRAY2   0x4208      	//��ɫ2  1111111111011111




#define LCD_CTRLA   	  	GPIOA		//����TFT���ݶ˿�
#define LCD_CTRLB   	  	GPIOB		//����TFT���ݶ˿�
#define LCD_CTRLC   	  	GPIOC


#define LCD_SCL        	GPIO_Pins_5	  //PA5--->>TFT --SCL/SCK
#define LCD_SDA        	GPIO_Pins_7 	//PA7--->>TFT --SDA/DIN
#define LCD_RST       	GPIO_Pins_9 	//PA9--->>TFT --RST
#define LCD_RS         	GPIO_Pins_7 	//PC7--->>TFT --RS/DC
#define LCD_CS        	GPIO_Pins_10  //PB10--->>TFT --CS/CE
#define LCD_LED        	GPIO_Pins_1   //PB1--->>TFT --BL

////Һ�����ƿ���1�������궨��
#define	LCD_SCL_SET  	LCD_CTRLA->BSRE=LCD_SCL    
#define	LCD_SDA_SET  	LCD_CTRLA->BSRE=LCD_SDA   
#define	LCD_CS_SET  	LCD_CTRLB->BSRE=LCD_CS  

#define	LCD_LED_SET  	LCD_CTRLB->BSRE=LCD_LED   
#define	LCD_RS_SET  	LCD_CTRLC->BSRE=LCD_RS 
#define	LCD_RST_SET  	LCD_CTRLA->BSRE=LCD_RST 
////Һ�����ƿ���0�������궨��
#define	LCD_SCL_CLR  	LCD_CTRLA->BRE=LCD_SCL  
#define	LCD_SDA_CLR  	LCD_CTRLA->BRE=LCD_SDA 
#define	LCD_CS_CLR  	LCD_CTRLB->BRE=LCD_CS 
    
#define	LCD_LED_CLR  	LCD_CTRLB->BRE=LCD_LED 
#define	LCD_RST_CLR  	LCD_CTRLA->BRE=LCD_RST
#define	LCD_RS_CLR  	LCD_CTRLC->BRE=LCD_RS 

#define LCD_DATAOUT(x) LCD_DATA->OPTDT=x; //�������
#define LCD_DATAIN     LCD_DATA->IPTDT;   //��������

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

