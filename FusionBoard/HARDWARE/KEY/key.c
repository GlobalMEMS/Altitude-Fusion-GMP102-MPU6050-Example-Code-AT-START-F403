#include "at32f4xx.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"
 
								    
//������ʼ������
void KEY_Init(void) //IO��ʼ��
{ 
 	GPIO_InitType GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA | RCC_APB2PERIPH_GPIOB,ENABLE);//ʹ��PORTA,PORTEʱ��

  GPIO_PinsRemapConfig(GPIO_Remap_SWJ_NoJNTRST, ENABLE);
  
	//��ʼ�� WK_UP-->GPIOA.0	  ��������
	GPIO_InitStructure.GPIO_Pins  = GPIO_Pins_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_PD; //PA0���ó����룬Ĭ������	  
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.0
  
	GPIO_InitStructure.GPIO_Pins  = GPIO_Pins_14 | GPIO_Pins_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOA.0
}
//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� 
//4��KEY3���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>KEY3!!
u8 KEY_Scan(void)
{	  
	if(USERKEY==1 || KEY1==1)
	{
		delay_ms(10);//ȥ���� 
		if(USERKEY==1)return UK_PRES;
    else if(KEY1==1)return KEY1_PRES;
	} 
 	return 0;// �ް�������
}
