#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

#define USERKEY   GPIO_ReadInputDataBit(GPIOA,GPIO_Pins_0)//读取按键3(WK_UP) 
#define KEY1      GPIO_ReadInputDataBit(GPIOB, GPIO_Pins_14)
#define KEY2      GPIO_ReadInputDataBit(GPIOB, GPIO_Pins_4)

#define UK_PRES     3	//UK_PRES按下
#define KEY1_PRES   1	//KEY1_PRES按下
#define KEY2_PRES   2	//KEY1_PRES按下

void KEY_Init(void);//IO初始化
u8 KEY_Scan(void);  	//按键扫描函数					    
#endif
