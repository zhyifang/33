#ifndef __UART_H
#define __UART_H
#include "stm32f10x.h"
//#include "MotorControl.h"
void UART2_Iint(u32 bound);

u8 USART2_QueryChar(u8 *ch);
u8 USART2_ReadTxBuf (void);
void USART2_HandleTxd(void);
u8 USART2_PutChar(u8 ch);

void Uart2_TxCommand(unsigned char cmdbyte);
void Uart2_CheckCmdData(void);

void UART2_SendByte(uint8_t SendData);

void UART2_SendHalfWord(uint16_t data);
void UART2_SendArray(uint8_t *array ,uint8_t num);
void UART2_SendStr(uint8_t *str );


#endif


