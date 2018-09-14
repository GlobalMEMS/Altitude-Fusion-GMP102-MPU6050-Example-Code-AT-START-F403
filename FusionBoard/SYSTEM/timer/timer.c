#include "timer.h"
#include "led.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//定时器 驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TRM3_Int_Init(u16 arr, u16 psc)
{
    TMR_TimerBaseInitType  TMR_TimeBaseStructure;
    NVIC_InitType NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, ENABLE); //时钟使能

    //定时器TIM3初始化
    TMR_TimeBaseStructure.TMR_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TMR_TimeBaseStructure.TMR_DIV = psc; //设置用来作为TIMx时钟频率除数的预分频值
    TMR_TimeBaseStructure.TMR_ClockDivision = TMR_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
    TMR_TimeBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;  //TIM向上计数模式
    TMR_TimeBaseInit(TMR3, &TMR_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
    
    TMR_INTConfig(TMR3, TMR_INT_Overflow, ENABLE ); //使能指定的TIM3中断,允许更新中断

    //中断优先级NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = TMR3_GLOBAL_IRQn;  //TIM3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


    TMR_Cmd(TMR3, ENABLE);  //使能TIMx
}













