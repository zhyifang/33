#include "GPIO.h"

void MY_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE );
	//EN_PIN
	RCC_APB2PeriphClockCmd(DRV8301_EN_GPIO_CLK|RCC_APB2Periph_AFIO, ENABLE );

	GPIO_InitStructure.GPIO_Pin = DRV8301_EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DRV8301_EN_GPIO_PORT,&GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//由于PB3是JATG引脚
	
	//FAULT_PIN
	RCC_APB2PeriphClockCmd(	DRV8301_FAULT_GPIO_CLK, ENABLE );

	GPIO_InitStructure.GPIO_Pin = DRV8301_FAULT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DRV8301_FAULT_GPIO_PORT,&GPIO_InitStructure);

	//OC_PIN
	RCC_APB2PeriphClockCmd(	DRV8301_OC_GPIO_CLK, ENABLE );

	GPIO_InitStructure.GPIO_Pin = DRV8301_OC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DRV8301_OC_GPIO_PORT,&GPIO_InitStructure);
	
	//POWERGD_PIN
	RCC_APB2PeriphClockCmd(	DRV8301_POWERGD_GPIO_CLK, ENABLE );

	GPIO_InitStructure.GPIO_Pin = DRV8301_POWERGD_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DRV8301_POWERGD_GPIO_PORT,&GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(MDBT40_CS_GPIO_CLK, ENABLE );
	GPIO_InitStructure.GPIO_Pin = MDBT40_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MDBT40_CS_PORT, &GPIO_InitStructure);//初始化GPIOA.4 5182片选

 	GPIO_SetBits(MDBT40_CS_PORT,MDBT40_CS_PIN); //取消片选
	
	RCC_APB2PeriphClockCmd(DRV8301_CS_GPIO_CLK, ENABLE );
	
	GPIO_InitStructure.GPIO_Pin = DRV8301_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DRV8301_CS_PORT, &GPIO_InitStructure);//初始化GPIOA.4 NSS

 	GPIO_SetBits(DRV8301_CS_PORT,DRV8301_CS_PIN); //取消片选
	
	//------------------------零点校准口-----------------------//

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}
