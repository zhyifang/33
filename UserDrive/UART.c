#include "UART.h"
//������
void UART2_Iint(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

    //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	//USART1_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;             //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	      //�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);                //��ʼ��GPIOA.2

	//USART1_RX	  GPIOA.3��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3 
	//USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//9600���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���1 
    
//  Usart1 NVIC ����
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4 ;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
}



//�������������ã�������ʱûд
//void USART2_IRQHandler(void)                	//����1�жϷ������
//{
//	    if ((USART2->SR & (u32)0x00000020) != (u16)RESET)
//    {
//        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
//        USART_WriteRxBuf (USART2->DR);
//    }
//}

/*����һ���ֽ�����*/
 void UART2_SendByte(uint8_t SendData)
{	   
   USART_SendData(USART2,SendData);
   while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	    
}  

/*���������ֽ�����16λ*/
void UART2_SendHalfWord(uint16_t data)
{
    uint8_t temp_h ,temp_l;
    temp_h = (data&0xff00)>>8;
    temp_l = data&0xff;
    UART2_SendByte(temp_h);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    
    UART2_SendByte(temp_l);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}


/*����8λ��������*/
void UART2_SendArray(uint8_t *array ,uint8_t num)
{
    uint8_t i;
    for(i=0;i<num;i++)
    {     
      UART2_SendByte(array[i]);
    }  
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

/*�����ַ���*/
void UART2_SendStr(uint8_t *str )
{  
    uint8_t i=0;
    do      
   {    
      UART2_SendByte(*(str+i)); 
      i++;
    }while(*(str+i)!='\0');
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

