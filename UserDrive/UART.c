#include "UART.h"
//调试用
void UART2_Iint(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

    //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	//USART1_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;             //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	      //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);                //初始化GPIOA.2

	//USART1_RX	  GPIOA.3初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3 
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//9600串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure); //初始化串口1
	USART_Cmd(USART2, ENABLE);                    //使能串口1 
    
//  Usart1 NVIC 配置
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4 ;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
}



//串口用来调试用，代码暂时没写
//void USART2_IRQHandler(void)                	//串口1中断服务程序
//{
//	    if ((USART2->SR & (u32)0x00000020) != (u16)RESET)
//    {
//        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
//        USART_WriteRxBuf (USART2->DR);
//    }
//}

/*发送一个字节数据*/
 void UART2_SendByte(uint8_t SendData)
{	   
   USART_SendData(USART2,SendData);
   while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	    
}  

/*发送两个字节数据16位*/
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


/*发送8位数据数组*/
void UART2_SendArray(uint8_t *array ,uint8_t num)
{
    uint8_t i;
    for(i=0;i<num;i++)
    {     
      UART2_SendByte(array[i]);
    }  
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

/*发送字符串*/
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

