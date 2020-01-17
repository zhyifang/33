#include "System.h"
/*************************
���ƣ�void initsysblock(void)
���ܣ����ֵײ������ʼ��
������None
���ߣ�zyf
************************/
//adc_sample  adc_data;
//variable    sysvariable;
//sys_flag    sys_flags;
//pid         speed_pid;
screw_cabration_flag   sys_flags;

void initsysblock(void){  
 
	RCC_Configuration();
//	SystemInit();
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	MY_GPIO_Init();
	
	
	SPI1_Init();
	delay_init(64);
	GateDriverConf();
	delay_ms(500);
	SPI_Cmd(SPI1, DISABLE);
	SPI1->CR1 = (SPI1->CR1&0xf7ff);//8λ֡��ʽ
	SPI_Cmd(SPI1, ENABLE);
//	TIM2_Int_Init(9,6399);         //1ms
	IWDG_Init(4,500);              //800ms
    
	
	       
	
	ADC_Configuration1();
//ADC_DMA_Configuration();
	Hall_Init();
	TIM1_Configuration1();
}
/*************************
���ƣ�void sys_enable(void)
���ܣ�ϵͳʹ��,
������None
���ߣ�zyf

************************/
void sys_enable(void){
/***************ʹ�ܶ�ʱ��1****************/
//    TIM_Cmd(TIM1,ENABLE);
//    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);   //���ж�or�����ж�,��
    TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE); 
}
/*************************
���ƣ�void sys_disable(void)
���ܣ�ϵͳ��ʹ��
������None
���ߣ�zyf

************************/
void sys_disable(void){
    TIM_Cmd(TIM1,DISABLE);
    TIM_CtrlPWMOutputs(TIM1, DISABLE); 
}

/*************************
���ƣ�void sys_variable_init(void)
���ܣ�ϵͳ������ʼ������
������None
���ߣ�zyf
************************/ 

/*************************
���ƣ�void sys_flag_init(void)
���ܣ�ϵͳ��־λ��ʼ������
������None
���ߣ�zyf

************************/ 
void sys_flag_init(void){
    sys_flags.ForewardStall_Flag=0;
    sys_flags.ReversedStall_Flag=0;
    
    sys_flags.screw_Calibration_Flag=0;
    sys_flags.screw_Calibrationleft_Flag=0;
    sys_flags.screw_Calibrationrigth_Flag=0;
    sys_flags.screw_CalibrationSuccess_Flag=0;
    
    sys_flags.wheel_Calibration_Flag=0;
    sys_flags.wheel_CalibrationSuccess_Flag=0;
    
}

