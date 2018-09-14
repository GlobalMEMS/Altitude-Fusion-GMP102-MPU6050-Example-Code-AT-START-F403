#include "timer.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//��ʱ�� ��������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TRM3_Int_Init(u16 arr, u16 psc)
{
    TMR_TimerBaseInitType  TMR_TimeBaseStructure;
    NVIC_InitType NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, ENABLE); //ʱ��ʹ��

    //��ʱ��TIM3��ʼ��
    TMR_TimeBaseStructure.TMR_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TMR_TimeBaseStructure.TMR_DIV = psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TMR_TimeBaseStructure.TMR_ClockDivision = TMR_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
    TMR_TimeBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;  //TIM���ϼ���ģʽ
    TMR_TimeBaseInit(TMR3, &TMR_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
    
    TMR_INTConfig(TMR3, TMR_INT_Overflow, ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

    //�ж����ȼ�NVIC����
    NVIC_InitStructure.NVIC_IRQChannel = TMR3_GLOBAL_IRQn;  //TIM3�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���


    TMR_Cmd(TMR3, ENABLE);  //ʹ��TIMx
}













