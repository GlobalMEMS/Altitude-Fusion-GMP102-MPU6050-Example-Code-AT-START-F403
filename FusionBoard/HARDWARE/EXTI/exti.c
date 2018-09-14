#include "exti.h"

void EXTIX_Init(void)
{

    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);	//ʹ�ܸ��ù���ʱ��

    //GPIOA.0	  �ж����Լ��жϳ�ʼ������ �����ش��� PA0  USER KEY
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinsSource0);

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineEnable = ENABLE;
    EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//ʹ�ܰ���WK_UP���ڵ��ⲿ�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;					//�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);
}
