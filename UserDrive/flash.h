#ifndef __FLASH_H
#define __FLASH_H
#include "stm32f10x.h"

#define STM32_FLASH_SIZE 32 	 		//所选STM32的FLASH容量大小(单位为K)

//FLASH起始地址
#define STM32_FLASH_BASE   0x08000000 	//STM32 FLASH的起始地址
#define FLASH_DRIVER_ADDR  0x08007C00   //设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)32扇区
#define FlASH_BATTERY_ADDR 0x08007800   //电池设置的地址
#define FlASH_INSTALL_ADDR 0x08007400   //

u16 STMFLASH_ReadHalfWord(u32 faddr);		  //读出半字  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//指定地址开始写入指定长度的数据
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//指定地址开始读取指定长度数据


void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//从指定地址开始读出指定长度的数据

//测试写入
void Test_Write(u32 WriteAddr,u16 WriteData);	

#endif
