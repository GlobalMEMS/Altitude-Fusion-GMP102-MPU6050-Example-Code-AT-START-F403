#include "sys.h"
#include "usart.h"
#include "GUI.h"
#include "Lcd_Driver.h"

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
  int handle;

};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
  x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
  while((USART1->STS & 0X40) == 0); //ѭ������,ֱ���������

  USART1->DT = (u8) ch;
  return ch;
}
#endif


#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA = 0;     //����״̬���

void uart_init(u32 bound)
{
  //GPIO�˿�����
  GPIO_InitType GPIO_InitStructure;
  USART_InitType USART_InitStructure;
  NVIC_InitType NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_USART1 | RCC_APB2PERIPH_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_USART2, ENABLE);

  //USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_2; //PA.9
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9

  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_3; //PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ; //��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

  //USART ��ʼ������
  USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
  USART_INTConfig(USART2, USART_INT_RDNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���1

  BLE_GPIO_Config();
}

u8 cnt = 0;
void USART2_IRQHandler(void)                	//����1�жϷ������
{
  u8 Res;
  u8 *ble_str;

  if(USART_GetITStatus(USART2, USART_INT_RDNE) != RESET)  //�����ж�
  {
    Res = USART_ReceiveData(USART2);	//��ȡ���յ�������

    while(USART_GetFlagStatus(USART1, USART_FLAG_TDE) != SET);

    if(cnt > 4)
    {
      cnt = 0;
      Lcd_Clear_xy(GRAY0, 10, 35, 120, 145);
    }
    itoa(Res, ble_str);
    Gui_DrawFont_GBK16(10, 40 + cnt * 20, BLUE, GRAY0, ble_str);		//��ʾ��������
    cnt++;
  }
}

int usart_send_buffer(u8* ch, u8 num)
{
  while(num--)
  {
    while((USART2->STS & 0X40) == 0); //ѭ������,ֱ���������
    USART2->DT = *ch;
    ch++;
  }
  return 0;
}

void BLE_GPIO_Config(void)
{
  GPIO_InitType GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA | RCC_APB2PERIPH_GPIOB, ENABLE);	//ʹ��USART1��GPIOAʱ��

  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_8;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pins = GPIO_Pins_0;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pins_8);
  GPIO_SetBits(GPIOB, GPIO_Pins_0);
}

#endif

