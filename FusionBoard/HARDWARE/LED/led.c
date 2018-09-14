#include "led.h" 

//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
 GPIO_InitType  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOD, ENABLE);	 //使能PB,PE端口时钟
	
 GPIO_InitStructure.GPIO_Pins = GPIO_Pins_13|GPIO_Pins_14|GPIO_Pins_15;				 //LED端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOD, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
 GPIO_SetBits(GPIOD,GPIO_Pins_13|GPIO_Pins_14|GPIO_Pins_15);						 //PB.5 输出高
}
 
