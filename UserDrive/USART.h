#ifndef UART
#define UART
//void USART3_Init(u32 bound);

//u8 USART_PutChar(u8 ch);


#define SEND_DATA()          GPIOB->BSRR = GPIO_Pin_4
#define RECIVE_DATA()        GPIOB->BRR = GPIO_Pin_4




#endif
