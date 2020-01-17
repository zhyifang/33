#ifndef __FLASH_H
#define __FLASH_H
#include "stm32f10x.h"

#define STM32_FLASH_SIZE 32 	 		//��ѡSTM32��FLASH������С(��λΪK)

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE   0x08000000 	//STM32 FLASH����ʼ��ַ
#define FLASH_DRIVER_ADDR  0x08007C00   //����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)32����
#define FlASH_BATTERY_ADDR 0x08007800   //������õĵ�ַ
#define FlASH_INSTALL_ADDR 0x08007400   //

u16 STMFLASH_ReadHalfWord(u32 faddr);		  //��������  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//ָ����ַ��ʼд��ָ�����ȵ�����
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//ָ����ַ��ʼ��ȡָ����������


void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����

//����д��
void Test_Write(u32 WriteAddr,u16 WriteData);	

#endif
