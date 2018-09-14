#include "led.h" 

//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
 GPIO_InitType  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOD, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
	
 GPIO_InitStructure.GPIO_Pins = GPIO_Pins_13|GPIO_Pins_14|GPIO_Pins_15;				 //LED�˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP; 		 //�������
 GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOD, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
 GPIO_SetBits(GPIOD,GPIO_Pins_13|GPIO_Pins_14|GPIO_Pins_15);						 //PB.5 �����
}
 
