#include "System.h"
/*************************
名称：void initsysblock(void)
功能：各种底层外设初始化
参数：None
作者：zyf
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
	SPI1->CR1 = (SPI1->CR1&0xf7ff);//8位帧格式
	SPI_Cmd(SPI1, ENABLE);
//	TIM2_Int_Init(9,6399);         //1ms
	IWDG_Init(4,500);              //800ms
    
	
	       
	
	ADC_Configuration1();
//ADC_DMA_Configuration();
	Hall_Init();
	TIM1_Configuration1();
}
/*************************
名称：void sys_enable(void)
功能：系统使能,
参数：None
作者：zyf

************************/
void sys_enable(void){
/***************使能定时器1****************/
//    TIM_Cmd(TIM1,ENABLE);
//    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);   //开中断or不开中断,开
    TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE); 
}
/*************************
名称：void sys_disable(void)
功能：系统不使能
参数：None
作者：zyf

************************/
void sys_disable(void){
    TIM_Cmd(TIM1,DISABLE);
    TIM_CtrlPWMOutputs(TIM1, DISABLE); 
}

/*************************
名称：void sys_variable_init(void)
功能：系统变量初始化函数
参数：None
作者：zyf
************************/ 

/*************************
名称：void sys_flag_init(void)
功能：系统标志位初始化函数
参数：None
作者：zyf

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

