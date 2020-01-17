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
	TIM2_Int_Init(9,6399);         //1ms
	IWDG_Init(4,500);              //800ms
	ADC_Configuration1();
	Hall_Init();
	TIM1_Configuration1();
    
//  /********结构体变量初始化******************/
//    GPIO_InitTypeDef         GPIO_InitStructre;
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructre;
//    TIM_OCInitTypeDef        TIM_OCInitStructre;
//    NVIC_InitTypeDef         NVIC_InitStructre;
//    DMA_InitTypeDef          DMA_InitStructre;
//    ADC_InitTypeDef          ADC_InitStructre;

//  /********程序上电默认值GPIO设置******************/
//    GPIO_DeInit(GPIOB);
//    GPIO_DeInit(GPIOA);
//    GPIO_DeInit(GPIOC);
//    
//  /********程序上电默认值TIM1设置******************/
//    TIM_DeInit(TIM1);
//    DMA_DeInit(DMA1_Channel1);
//  /********GPIO的时钟开启******************/
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
//  /*******定时器时钟开启******************/
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
//  /*******DMA的时钟开启******************/ 
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
//  /*******ADC的时钟开启******************/ 
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
//  /*******TIM2的时钟开启******************/ 
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);    
//    
//  /********GPIO_LED的配置******************/
//    GPIO_InitStructre.GPIO_Pin   = GREEN_LED|RED_LED|BLUE_LED;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_OUT;
//    GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//     GPIO_Init(GPIO_LED, &GPIO_InitStructre);
//  /********GPIO_LCD的配置******************/
//    GPIO_InitStructre.GPIO_Pin   = LCD_LED|LCD_CS|LCD_RST|LCD_SDA|LCD_SCL;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_OUT;
//    GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOB, &GPIO_InitStructre);
//    
//    GPIO_InitStructre.GPIO_Pin   = LCD_RS;
//    GPIO_Init(GPIOA, &GPIO_InitStructre);
//   /********LCD背光熄灭*************/  
//    LCD_LED_CLR;
//    
//  /********LED_GPIO的初始化时候三个灯全度熄灭*************/
//    RED_OFF;
//    GREEN_OFF;
//    BLUE_OFF;
//    
///********PWM_GPIO的配置******************/
//    GPIO_InitStructre.GPIO_Pin   = PWM1H |PWM2H |PWM3H |PWM4H;
//    //上桥的AF复用
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_AF;
//    GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz; 
//    GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(PWMH_PORT, &GPIO_InitStructre); 
//    
//    
//    //下桥的pwm配置
//    GPIO_InitStructre.GPIO_Pin   = PWM1L;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_OUT;
//    GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz; 
//    GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(PWML_PORT_1, &GPIO_InitStructre); 
//    
//    GPIO_InitStructre.GPIO_Pin   = PWM2L|PWM3L;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_OUT;
//    GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz; 
//    GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(PWML_PORT_2_3, &GPIO_InitStructre);
///********PWM_GPIO复用PWM模块的配置******************/
//    GPIO_PinAFConfig(PWMH_PORT,PWM1H_AF,GPIO_AF_2);
//    GPIO_PinAFConfig(PWMH_PORT,PWM2H_AF,GPIO_AF_2);
//    GPIO_PinAFConfig(PWMH_PORT,PWM3H_AF,GPIO_AF_2);
//    GPIO_PinAFConfig(PWMH_PORT,PWM4H_AF,GPIO_AF_2);//

///********PWM_GPIO配置时钟******************/
//    TIM_TimeBaseStructre.TIM_Prescaler=0;
//    TIM_TimeBaseStructre.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
//    TIM_TimeBaseStructre.TIM_Period=3199;                   //15k的PWM周期频率
//    TIM_TimeBaseStructre.TIM_ClockDivision=0;
//    TIM_TimeBaseStructre.TIM_RepetitionCounter=0;
//    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructre);
///***************PWM配置******************/  
//    TIM_OCInitStructre.TIM_OCMode=TIM_OCMode_PWM1;//PWM1模式
//    /***************输出使能******************/
//    TIM_OCInitStructre.TIM_OutputState=TIM_OutputState_Enable;
//    /***************PWM互补通道禁止******************/
//    TIM_OCInitStructre.TIM_OCNPolarity=TIM_OutputNState_Disable;
//    TIM_OCInitStructre.TIM_Pulse=0;                //初始占空比设为零
//    TIM_OCInitStructre.TIM_OCPolarity=TIM_OCPolarity_High;
//    TIM_OCInitStructre.TIM_OCNPolarity=TIM_OCNPolarity_High;
//    /***************空闲输出低******************/
//    TIM_OCInitStructre.TIM_OCIdleState=TIM_OCIdleState_Reset;
//    /***************互补通道空闲输出低******************/
//    TIM_OCInitStructre.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
//    TIM_OC1Init(TIM1, &TIM_OCInitStructre);
//    TIM_OC2Init(TIM1, &TIM_OCInitStructre);
//    TIM_OC3Init(TIM1, &TIM_OCInitStructre);
//    
//    /***************PWMCC4的初始化触发AD采样时间*****************/
//    TIM_OCInitStructre.TIM_OCMode=TIM_OCMode_PWM1;//PWM1模式
//    /***************输出使能******************/
//    TIM_OCInitStructre.TIM_OutputState=TIM_OutputState_Enable;
//    /***************PWM互补通道禁止******************/
//    TIM_OCInitStructre.TIM_OCNPolarity=TIM_OutputNState_Disable;
//    TIM_OCInitStructre.TIM_Pulse=AD_TriggerTime;                //初始占空比设为零
//    TIM_OCInitStructre.TIM_OCPolarity=TIM_OCPolarity_High;
//    TIM_OCInitStructre.TIM_OCNPolarity=TIM_OCNPolarity_High;
//    /***************空闲输出低******************/
//    TIM_OCInitStructre.TIM_OCIdleState=TIM_OCIdleState_Reset;
//    TIM_OC4Init(TIM1, &TIM_OCInitStructre);
//    
//    /***************CC4通道4的中断使能******************/
//    TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);
//    
//    /***************嵌套中断管理器*****************/
//    NVIC_InitStructre.NVIC_IRQChannel=TIM1_CC_IRQn;
//    NVIC_InitStructre.NVIC_IRQChannelPriority=2;
//    NVIC_InitStructre.NVIC_IRQChannelCmd=ENABLE;
//    NVIC_Init(&NVIC_InitStructre);
//    
//    /***************先禁止定时器1****************/
//    TIM_Cmd(TIM1,DISABLE);
//    
//    /***************将PWM输出打开****************/
//    TIM_CtrlPWMOutputs(TIM1, ENABLE);   //使能PWM
//   
///********GPIO_ADC的配置 模拟口******************/
//    GPIO_InitStructre.GPIO_Pin   = Bridge_CURRENT|ELE_GUN;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_AN;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(ADC_POARTC, &GPIO_InitStructre);
//    
//    GPIO_InitStructre.GPIO_Pin   = VOTAGE;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_AN;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(ADC_POARTA, &GPIO_InitStructre);

///********DMA的配置 用于adc的采用******************/
//    //外设地址ADC，DMA开始搬运数据  
//    DMA_InitStructre.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
//    //DMA搬运数据存放在内存地址
//    DMA_InitStructre.DMA_MemoryBaseAddr = (uint32_t)&adc_data;
//    //DMA搬运数据的方向，从外往里搬运数据
//    DMA_InitStructre.DMA_DIR = DMA_DIR_PeripheralSRC;
//    //DMA搬运的通道，有三个通道
//    DMA_InitStructre.DMA_BufferSize = 3;
//    //外设的地址不自己增加
//    DMA_InitStructre.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    
//    //内存的地址自己增加
//    DMA_InitStructre.DMA_MemoryInc = DMA_MemoryInc_Enable ;
//    //外设数据大小
//    DMA_InitStructre.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//    //内存数据的大小
//    DMA_InitStructre.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord ;
//    //DMA搬运数据的循环模式
//    DMA_InitStructre.DMA_Mode =  DMA_Mode_Circular;
//    //DMA 优先级的配置
//    DMA_InitStructre.DMA_Priority = DMA_Priority_Medium;
//    //DMA由内存到内存
//    DMA_InitStructre.DMA_M2M = DMA_M2M_Disable;
//    
//    DMA_Init(DMA1_Channel1, &DMA_InitStructre);
//    
//    
///********DMA的中断配置******************/
//    NVIC_InitStructre.NVIC_IRQChannel= DMA1_Channel1_IRQn; //中断通道
//    NVIC_InitStructre.NVIC_IRQChannelPriority=1;    //中断优先级
//    NVIC_InitStructre.NVIC_IRQChannelCmd=ENABLE;    //使能
//    NVIC_Init(&NVIC_InitStructre);  
//       
//    DMA_ClearFlag(DMA1_FLAG_TC1);                     //清除中断标志位
//    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);   //开中断or不开中断,先不开
//    DMA_Cmd(DMA1_Channel1,ENABLE);                    //DMA使能，进行搬运
//    
//    
///********ADC的配置用于电压，电流和速度给定******************/   
//    //四分频消除抖动
//    ADC_JitterCmd(ADC1,ADC_JitterOff_PCLKDiv4,ENABLE);
//    RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);   //RCC四分频给ADC时钟
//    //12位的AD
//    ADC_InitStructre.ADC_Resolution = ADC_Resolution_12b;
//    ADC_InitStructre.ADC_ContinuousConvMode = DISABLE;
//    ADC_InitStructre.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling ;
//    ADC_InitStructre.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4; //tim1的cc4触发
//    ADC_InitStructre.ADC_DataAlign = ADC_DataAlign_Right;
//    ADC_InitStructre.ADC_ScanDirection = ADC_ScanDirection_Upward; //有低到高扫描
//    ADC_Init(ADC1,&ADC_InitStructre); 
// /********ADC的通道及采样时间配置******************/     
//   //电流， 旋转纽扣，电压
//    ADC_ChannelConfig(ADC1, Bridge_CURRENT_CHANNEL, ADC_SampleTime_7_5Cycles );
//    ADC_ChannelConfig(ADC1, ELE_GUN_CHANNEL, ADC_SampleTime_7_5Cycles );
//    ADC_ChannelConfig(ADC1, VOTAGE_CHANNEL, ADC_SampleTime_7_5Cycles );
//      
// /********ADC的校准******************/  
//    ADC_GetCalibrationFactor(ADC1);
//    ADC_Cmd(ADC1,ENABLE);
//    
// /********等待ADC的准备好******************/ 
//    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY  ));
// /********使能ADC的DMA传输******************/ 
//    ADC_DMACmd(ADC1, ENABLE);
// /********ADC的DMA模式的配置******************/  
//    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
//    ADC_StartOfConversion(ADC1);
//    
///********GPIO_HALL的配置******************/
//    GPIO_InitStructre.GPIO_Pin   = HALLU|HALLV|HALLW;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_IN;
//   //GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz;
//   // GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
//    GPIO_Init(HALL_POART, &GPIO_InitStructre); 
// 
///********转速的计算的配置，定时器TIM2的配置******************/  
///**************************/
//    TIM_TimeBaseStructre.TIM_Prescaler=47;//48分频，为了得到1us计数一次
//    
//    TIM_TimeBaseStructre.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
//    TIM_TimeBaseStructre.TIM_Period=0xffff;                  //重再值65535
//    TIM_TimeBaseStructre.TIM_ClockDivision=0;
//    TIM_TimeBaseStructre.TIM_RepetitionCounter=0;
//    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructre);
//    TIM2->CNT = 0;
//    TIM_Cmd(TIM2,ENABLE);     
//    


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
void sys_variable_init(void){
//    sysvariable.desire_speed=0;
//    sysvariable.measure_speed=0;
//    sysvariable.motor_current=0;
//    sysvariable.motor_now_position=0;
//    sysvariable.motor_old_position=0;
//    sysvariable.run_status=0;
//    sysvariable.votage=2709;//24V的数值
//    
//    sysvariable.pwm=MIN_DUTY;
//    
//    sys_flags.voltage_protect_flag=0;
//    sys_flags.current_protect_flag=0;
//    sys_flags.currentloop_protect_flag=0;
     
}
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
/*************************
名称：void Screw_Calibration(void)
功能：丝杆校准函数
参数：None
作者：zyf

************************/ 
void Screw_Calibration(void)
{
	if((sys_flags.screw_Calibration_Flag==1)&&(sys_flags.screw_CalibrationSuccess_Flag==0))  //进行校准中(没成功也没失败)
    {
		uiLengthDifferent = 80;//100max，相当于给一个匀速

		if(sys_flags.screw_Calibrationleft_Flag == 0)
		{
			
			ucMotorDrection = 1;//左，顺时针拉回
			if((GPIOB->IDR & GPIO_Pin_9)==0)//接触到行程开关了
			{
                sys_flags.screw_Calibrationleft_Flag = 1;  //左校准完成标志位
				keytemp = 2;
				PowerPara.Out = 0;
				PowerPara.Ui  = 0;
				SpeedPara.Out = 0;
				SpeedPara.Ui  = 0;
				uiCurrentLocationHallCnt = 0;//标记0点
				uiHallSpeedCnt = 0;          //标记0点
				
			}
			else if(sys_flags.ReversedStall_Flag ==1) //堵转了，但是没检测到霍尔信号
			{
                sys_flags.screw_Calibrationleft_Flag = 1;//左校准完成
                
                sys_flags.ReversedStall_Flag = 0;
                keytemp = 2;
				PowerPara.Out = 0;
				PowerPara.Ui  = 0;
				SpeedPara.Out = 0;
				SpeedPara.Ui  = 0;
				ucLeftShortFlag = 1;
				
				uiCurrentLocationHallCnt = 0;
				uiHallSpeedCnt = 0;
				
			}
			
		}
		if(sys_flags.screw_Calibrationleft_Flag == 1)                   //
		{
			keytemp = 1;			                      //启动，因为堵转导致停止了
			ucMotorDrection = 0;	                      //往右，电机逆时针，推动丝杆往前
			if(uiHallSpeedCnt>=(uiLeftLength+uiRigthLength))
			{
				sys_flags.screw_Calibrationrigth_Flag = 1;
//				uiLengthSum = uiHallSpeedCnt;             //记录丝杠长度
				ucMotorDrection=1;                        //左，电机顺时针，拉动丝杆往后

			}
			else if((sys_flags.screw_Calibrationrigth_Flag==0)&&(uiHallSpeedCnt<(uiLeftLength+uiRigthLength))&&(sys_flags.ForewardStall_Flag == 1))//堵转了还没达到
			{
				if(uiHallSpeedCnt<((uiLeftLength+uiRigthLength)*9/10))
				{
					sys_flags.screw_Calibrationrigth_Flag = 0;
					if(ucLeftShortFlag==0)
						ucRightShortFlag = 1;
				}
				else
				{
                    sys_flags.ForewardStall_Flag = 0;
//					uiLengthSum = uiHallSpeedCnt;//记录丝杠长度
				    sys_flags.screw_Calibrationrigth_Flag = 1;
					ucMotorDrection=1;//左
				}

			}
            
            if(sys_flags.screw_Calibrationrigth_Flag==1&&sys_flags.screw_Calibrationleft_Flag==1){
                
                sys_flags.screw_CalibrationSuccess_Flag=1;
            
            }else sys_flags.screw_CalibrationSuccess_Flag=0;
            
            
		}
        if((sys_flags.screw_CalibrationSuccess_Flag==1)||(sys_flags.screw_CalibrationSuccess_Flag==0))
		{
//			uiCalibrationSpeed = 0;
			uiHallSpeedCnt = 0;
			uiLengthDifferent = 0;
			keytemp = 2;
			sys_flags.screw_Calibrationleft_Flag = 0;
		
		}
	}
}


/*************************
名称：void wheel_Calibration(void)
功能：方向盘0点校准位置函数
参数：None
作者：zyf

************************/
void wheel_Calibration(void)
{
	if((sys_flags.wheel_Calibration_Flag==1)&&(sys_flags.wheel_CalibrationSuccess_Flag==0))
	{
		if(ucSteeringDrection==1)//1方向盘左。电机拉动丝杆往后走了
		{
			//转动角度和转动方向
			ucDistanceAngle    = ucAngle;
//			ucDistanceDirction = ucSteeringDrection;
//			//对半分
//			uiLeftLength  = uiLengthSum / 2;
//			uiRigthLength = uiLengthSum / 2;
			
			ucDriveDistanceError = uiCurrentLocation - uiLeftLength;
			uiLeftLength  = uiLeftLength + ucDriveDistanceError;
			uiRigthLength = uiRigthLength - ucDriveDistanceError;
			datatemp[0] = uiLeftLength;
			datatemp[1] = uiRigthLength;
            
			datatemp[2] = uiRigthLength^uiLeftLength;
			datatemp[3] = 0xaaaa;
			
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flash写
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash读
			
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				sys_flags.wheel_CalibrationSuccess_Flag = 1;       //方向盘0点校准成功标志位
		}
		else                     //0方向盘右。电机推动丝杆往前走了
		{
			ucDistanceAngle    = ucAngle;
//			ucDistanceDirction = ucSteeringDrection;
//			//对半分丝杠长度
//			uiLeftLength = uiLengthSum / 2;
//			uiRigthLength = uiLengthSum / 2;
			
			ucDriveDistanceError = uiLeftLength - uiCurrentLocation;
			uiLeftLength = uiLeftLength - ucDriveDistanceError;
			uiRigthLength = uiRigthLength + ucDriveDistanceError;
			
			datatemp[0] = uiLeftLength;
			datatemp[1] = uiRigthLength;
			datatemp[2] = uiRigthLength^uiLeftLength;
			datatemp[3] = 0xaaaa;
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flash写
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash读
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				sys_flags.wheel_CalibrationSuccess_Flag = 1;
		}
	}	
}

