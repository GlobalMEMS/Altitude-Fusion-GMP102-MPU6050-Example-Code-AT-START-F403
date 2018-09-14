#include "at32f4xx.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"
 
								    
//按键初始化函数
void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitType GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA | RCC_APB2PERIPH_GPIOB,ENABLE);//使能PORTA,PORTE时钟

  GPIO_PinsRemapConfig(GPIO_Remap_SWJ_NoJNTRST, ENABLE);
  
	//初始化 WK_UP-->GPIOA.0	  下拉输入
	GPIO_InitStructure.GPIO_Pins  = GPIO_Pins_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PD; //PA0设置成输入，默认下拉	  
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.0
  
	GPIO_InitStructure.GPIO_Pins  = GPIO_Pins_14 | GPIO_Pins_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOA.0
}
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 
//4，KEY3按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>KEY3!!
u8 KEY_Scan(void)
{	  
	if(USERKEY==1 || KEY1==1)
	{
		delay_ms(10);//去抖动 
		if(USERKEY==1)return UK_PRES;
    else if(KEY1==1)return KEY1_PRES;
	} 
 	return 0;// 无按键按下
}
