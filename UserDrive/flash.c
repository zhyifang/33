#include "flash.h"
//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}


//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数   
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	    WriteAddr+=2;//地址增加2.
	}  
} 
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节
#else 
#define STM_SECTOR_SIZE	2048
#endif
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节
//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{   
 	u16 i;    
	FLASH_Unlock(); //解锁
	while(1) 
	{	
		FLASH_ErasePage((WriteAddr-STM32_FLASH_BASE)/1024*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
		for(i=0;i<NumToWrite;i++)//复制
		{
			STMFLASH_BUF[i]=pBuffer[i];	  
		}
		STMFLASH_Write_NoCheck((WriteAddr-STM32_FLASH_BASE)/1024*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,NumToWrite);//写入整个扇区  				   
		break;//写入结束了 
	};	
	FLASH_Lock();//上锁
}


//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
		ReadAddr+=2;//偏移2个字节.	
	}
}

//WriteAddr:起始地址
//WriteData:要写入的数据
void Test_Write(u32 WriteAddr,u16 WriteData)   	
{
	STMFLASH_Write(WriteAddr,&WriteData,1);//写入一个字 
}




