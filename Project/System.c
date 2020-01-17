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
	TIM2_Int_Init(9,6399);         //1ms
	IWDG_Init(4,500);              //800ms
	ADC_Configuration1();
	Hall_Init();
	TIM1_Configuration1();
    
//  /********�ṹ�������ʼ��******************/
//    GPIO_InitTypeDef         GPIO_InitStructre;
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructre;
//    TIM_OCInitTypeDef        TIM_OCInitStructre;
//    NVIC_InitTypeDef         NVIC_InitStructre;
//    DMA_InitTypeDef          DMA_InitStructre;
//    ADC_InitTypeDef          ADC_InitStructre;

//  /********�����ϵ�Ĭ��ֵGPIO����******************/
//    GPIO_DeInit(GPIOB);
//    GPIO_DeInit(GPIOA);
//    GPIO_DeInit(GPIOC);
//    
//  /********�����ϵ�Ĭ��ֵTIM1����******************/
//    TIM_DeInit(TIM1);
//    DMA_DeInit(DMA1_Channel1);
//  /********GPIO��ʱ�ӿ���******************/
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
//  /*******��ʱ��ʱ�ӿ���******************/
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
//  /*******DMA��ʱ�ӿ���******************/ 
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
//  /*******ADC��ʱ�ӿ���******************/ 
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
//  /*******TIM2��ʱ�ӿ���******************/ 
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);    
//    
//  /********GPIO_LED������******************/
//    GPIO_InitStructre.GPIO_Pin   = GREEN_LED|RED_LED|BLUE_LED;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_OUT;
//    GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//     GPIO_Init(GPIO_LED, &GPIO_InitStructre);
//  /********GPIO_LCD������******************/
//    GPIO_InitStructre.GPIO_Pin   = LCD_LED|LCD_CS|LCD_RST|LCD_SDA|LCD_SCL;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_OUT;
//    GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOB, &GPIO_InitStructre);
//    
//    GPIO_InitStructre.GPIO_Pin   = LCD_RS;
//    GPIO_Init(GPIOA, &GPIO_InitStructre);
//   /********LCD����Ϩ��*************/  
//    LCD_LED_CLR;
//    
//  /********LED_GPIO�ĳ�ʼ��ʱ��������ȫ��Ϩ��*************/
//    RED_OFF;
//    GREEN_OFF;
//    BLUE_OFF;
//    
///********PWM_GPIO������******************/
//    GPIO_InitStructre.GPIO_Pin   = PWM1H |PWM2H |PWM3H |PWM4H;
//    //���ŵ�AF����
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_AF;
//    GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz; 
//    GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(PWMH_PORT, &GPIO_InitStructre); 
//    
//    
//    //���ŵ�pwm����
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
///********PWM_GPIO����PWMģ�������******************/
//    GPIO_PinAFConfig(PWMH_PORT,PWM1H_AF,GPIO_AF_2);
//    GPIO_PinAFConfig(PWMH_PORT,PWM2H_AF,GPIO_AF_2);
//    GPIO_PinAFConfig(PWMH_PORT,PWM3H_AF,GPIO_AF_2);
//    GPIO_PinAFConfig(PWMH_PORT,PWM4H_AF,GPIO_AF_2);//

///********PWM_GPIO����ʱ��******************/
//    TIM_TimeBaseStructre.TIM_Prescaler=0;
//    TIM_TimeBaseStructre.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
//    TIM_TimeBaseStructre.TIM_Period=3199;                   //15k��PWM����Ƶ��
//    TIM_TimeBaseStructre.TIM_ClockDivision=0;
//    TIM_TimeBaseStructre.TIM_RepetitionCounter=0;
//    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructre);
///***************PWM����******************/  
//    TIM_OCInitStructre.TIM_OCMode=TIM_OCMode_PWM1;//PWM1ģʽ
//    /***************���ʹ��******************/
//    TIM_OCInitStructre.TIM_OutputState=TIM_OutputState_Enable;
//    /***************PWM����ͨ����ֹ******************/
//    TIM_OCInitStructre.TIM_OCNPolarity=TIM_OutputNState_Disable;
//    TIM_OCInitStructre.TIM_Pulse=0;                //��ʼռ�ձ���Ϊ��
//    TIM_OCInitStructre.TIM_OCPolarity=TIM_OCPolarity_High;
//    TIM_OCInitStructre.TIM_OCNPolarity=TIM_OCNPolarity_High;
//    /***************���������******************/
//    TIM_OCInitStructre.TIM_OCIdleState=TIM_OCIdleState_Reset;
//    /***************����ͨ�����������******************/
//    TIM_OCInitStructre.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
//    TIM_OC1Init(TIM1, &TIM_OCInitStructre);
//    TIM_OC2Init(TIM1, &TIM_OCInitStructre);
//    TIM_OC3Init(TIM1, &TIM_OCInitStructre);
//    
//    /***************PWMCC4�ĳ�ʼ������AD����ʱ��*****************/
//    TIM_OCInitStructre.TIM_OCMode=TIM_OCMode_PWM1;//PWM1ģʽ
//    /***************���ʹ��******************/
//    TIM_OCInitStructre.TIM_OutputState=TIM_OutputState_Enable;
//    /***************PWM����ͨ����ֹ******************/
//    TIM_OCInitStructre.TIM_OCNPolarity=TIM_OutputNState_Disable;
//    TIM_OCInitStructre.TIM_Pulse=AD_TriggerTime;                //��ʼռ�ձ���Ϊ��
//    TIM_OCInitStructre.TIM_OCPolarity=TIM_OCPolarity_High;
//    TIM_OCInitStructre.TIM_OCNPolarity=TIM_OCNPolarity_High;
//    /***************���������******************/
//    TIM_OCInitStructre.TIM_OCIdleState=TIM_OCIdleState_Reset;
//    TIM_OC4Init(TIM1, &TIM_OCInitStructre);
//    
//    /***************CC4ͨ��4���ж�ʹ��******************/
//    TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);
//    
//    /***************Ƕ���жϹ�����*****************/
//    NVIC_InitStructre.NVIC_IRQChannel=TIM1_CC_IRQn;
//    NVIC_InitStructre.NVIC_IRQChannelPriority=2;
//    NVIC_InitStructre.NVIC_IRQChannelCmd=ENABLE;
//    NVIC_Init(&NVIC_InitStructre);
//    
//    /***************�Ƚ�ֹ��ʱ��1****************/
//    TIM_Cmd(TIM1,DISABLE);
//    
//    /***************��PWM�����****************/
//    TIM_CtrlPWMOutputs(TIM1, ENABLE);   //ʹ��PWM
//   
///********GPIO_ADC������ ģ���******************/
//    GPIO_InitStructre.GPIO_Pin   = Bridge_CURRENT|ELE_GUN;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_AN;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(ADC_POARTC, &GPIO_InitStructre);
//    
//    GPIO_InitStructre.GPIO_Pin   = VOTAGE;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_AN;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//    GPIO_Init(ADC_POARTA, &GPIO_InitStructre);

///********DMA������ ����adc�Ĳ���******************/
//    //�����ַADC��DMA��ʼ��������  
//    DMA_InitStructre.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
//    //DMA�������ݴ�����ڴ��ַ
//    DMA_InitStructre.DMA_MemoryBaseAddr = (uint32_t)&adc_data;
//    //DMA�������ݵķ��򣬴��������������
//    DMA_InitStructre.DMA_DIR = DMA_DIR_PeripheralSRC;
//    //DMA���˵�ͨ����������ͨ��
//    DMA_InitStructre.DMA_BufferSize = 3;
//    //����ĵ�ַ���Լ�����
//    DMA_InitStructre.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    
//    //�ڴ�ĵ�ַ�Լ�����
//    DMA_InitStructre.DMA_MemoryInc = DMA_MemoryInc_Enable ;
//    //�������ݴ�С
//    DMA_InitStructre.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//    //�ڴ����ݵĴ�С
//    DMA_InitStructre.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord ;
//    //DMA�������ݵ�ѭ��ģʽ
//    DMA_InitStructre.DMA_Mode =  DMA_Mode_Circular;
//    //DMA ���ȼ�������
//    DMA_InitStructre.DMA_Priority = DMA_Priority_Medium;
//    //DMA���ڴ浽�ڴ�
//    DMA_InitStructre.DMA_M2M = DMA_M2M_Disable;
//    
//    DMA_Init(DMA1_Channel1, &DMA_InitStructre);
//    
//    
///********DMA���ж�����******************/
//    NVIC_InitStructre.NVIC_IRQChannel= DMA1_Channel1_IRQn; //�ж�ͨ��
//    NVIC_InitStructre.NVIC_IRQChannelPriority=1;    //�ж����ȼ�
//    NVIC_InitStructre.NVIC_IRQChannelCmd=ENABLE;    //ʹ��
//    NVIC_Init(&NVIC_InitStructre);  
//       
//    DMA_ClearFlag(DMA1_FLAG_TC1);                     //����жϱ�־λ
//    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);   //���ж�or�����ж�,�Ȳ���
//    DMA_Cmd(DMA1_Channel1,ENABLE);                    //DMAʹ�ܣ����а���
//    
//    
///********ADC���������ڵ�ѹ���������ٶȸ���******************/   
//    //�ķ�Ƶ��������
//    ADC_JitterCmd(ADC1,ADC_JitterOff_PCLKDiv4,ENABLE);
//    RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);   //RCC�ķ�Ƶ��ADCʱ��
//    //12λ��AD
//    ADC_InitStructre.ADC_Resolution = ADC_Resolution_12b;
//    ADC_InitStructre.ADC_ContinuousConvMode = DISABLE;
//    ADC_InitStructre.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling ;
//    ADC_InitStructre.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4; //tim1��cc4����
//    ADC_InitStructre.ADC_DataAlign = ADC_DataAlign_Right;
//    ADC_InitStructre.ADC_ScanDirection = ADC_ScanDirection_Upward; //�е͵���ɨ��
//    ADC_Init(ADC1,&ADC_InitStructre); 
// /********ADC��ͨ��������ʱ������******************/     
//   //������ ��תŦ�ۣ���ѹ
//    ADC_ChannelConfig(ADC1, Bridge_CURRENT_CHANNEL, ADC_SampleTime_7_5Cycles );
//    ADC_ChannelConfig(ADC1, ELE_GUN_CHANNEL, ADC_SampleTime_7_5Cycles );
//    ADC_ChannelConfig(ADC1, VOTAGE_CHANNEL, ADC_SampleTime_7_5Cycles );
//      
// /********ADC��У׼******************/  
//    ADC_GetCalibrationFactor(ADC1);
//    ADC_Cmd(ADC1,ENABLE);
//    
// /********�ȴ�ADC��׼����******************/ 
//    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY  ));
// /********ʹ��ADC��DMA����******************/ 
//    ADC_DMACmd(ADC1, ENABLE);
// /********ADC��DMAģʽ������******************/  
//    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
//    ADC_StartOfConversion(ADC1);
//    
///********GPIO_HALL������******************/
//    GPIO_InitStructre.GPIO_Pin   = HALLU|HALLV|HALLW;
//    GPIO_InitStructre.GPIO_Mode  = GPIO_Mode_IN;
//   //GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz;
//   // GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructre.GPIO_PuPd  = GPIO_PuPd_NOPULL; 
//    GPIO_Init(HALL_POART, &GPIO_InitStructre); 
// 
///********ת�ٵļ�������ã���ʱ��TIM2������******************/  
///**************************/
//    TIM_TimeBaseStructre.TIM_Prescaler=47;//48��Ƶ��Ϊ�˵õ�1us����һ��
//    
//    TIM_TimeBaseStructre.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
//    TIM_TimeBaseStructre.TIM_Period=0xffff;                  //����ֵ65535
//    TIM_TimeBaseStructre.TIM_ClockDivision=0;
//    TIM_TimeBaseStructre.TIM_RepetitionCounter=0;
//    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructre);
//    TIM2->CNT = 0;
//    TIM_Cmd(TIM2,ENABLE);     
//    


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
void sys_variable_init(void){
//    sysvariable.desire_speed=0;
//    sysvariable.measure_speed=0;
//    sysvariable.motor_current=0;
//    sysvariable.motor_now_position=0;
//    sysvariable.motor_old_position=0;
//    sysvariable.run_status=0;
//    sysvariable.votage=2709;//24V����ֵ
//    
//    sysvariable.pwm=MIN_DUTY;
//    
//    sys_flags.voltage_protect_flag=0;
//    sys_flags.current_protect_flag=0;
//    sys_flags.currentloop_protect_flag=0;
     
}
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
/*************************
���ƣ�void Screw_Calibration(void)
���ܣ�˿��У׼����
������None
���ߣ�zyf

************************/ 
void Screw_Calibration(void)
{
	if((sys_flags.screw_Calibration_Flag==1)&&(sys_flags.screw_CalibrationSuccess_Flag==0))  //����У׼��(û�ɹ�Ҳûʧ��)
    {
		uiLengthDifferent = 80;//100max���൱�ڸ�һ������

		if(sys_flags.screw_Calibrationleft_Flag == 0)
		{
			
			ucMotorDrection = 1;//��˳ʱ������
			if((GPIOB->IDR & GPIO_Pin_9)==0)//�Ӵ����г̿�����
			{
                sys_flags.screw_Calibrationleft_Flag = 1;  //��У׼��ɱ�־λ
				keytemp = 2;
				PowerPara.Out = 0;
				PowerPara.Ui  = 0;
				SpeedPara.Out = 0;
				SpeedPara.Ui  = 0;
				uiCurrentLocationHallCnt = 0;//���0��
				uiHallSpeedCnt = 0;          //���0��
				
			}
			else if(sys_flags.ReversedStall_Flag ==1) //��ת�ˣ�����û��⵽�����ź�
			{
                sys_flags.screw_Calibrationleft_Flag = 1;//��У׼���
                
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
			keytemp = 1;			                      //��������Ϊ��ת����ֹͣ��
			ucMotorDrection = 0;	                      //���ң������ʱ�룬�ƶ�˿����ǰ
			if(uiHallSpeedCnt>=(uiLeftLength+uiRigthLength))
			{
				sys_flags.screw_Calibrationrigth_Flag = 1;
//				uiLengthSum = uiHallSpeedCnt;             //��¼˿�ܳ���
				ucMotorDrection=1;                        //�󣬵��˳ʱ�룬����˿������

			}
			else if((sys_flags.screw_Calibrationrigth_Flag==0)&&(uiHallSpeedCnt<(uiLeftLength+uiRigthLength))&&(sys_flags.ForewardStall_Flag == 1))//��ת�˻�û�ﵽ
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
//					uiLengthSum = uiHallSpeedCnt;//��¼˿�ܳ���
				    sys_flags.screw_Calibrationrigth_Flag = 1;
					ucMotorDrection=1;//��
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
���ƣ�void wheel_Calibration(void)
���ܣ�������0��У׼λ�ú���
������None
���ߣ�zyf

************************/
void wheel_Calibration(void)
{
	if((sys_flags.wheel_Calibration_Flag==1)&&(sys_flags.wheel_CalibrationSuccess_Flag==0))
	{
		if(ucSteeringDrection==1)//1�������󡣵������˿����������
		{
			//ת���ǶȺ�ת������
			ucDistanceAngle    = ucAngle;
//			ucDistanceDirction = ucSteeringDrection;
//			//�԰��
//			uiLeftLength  = uiLengthSum / 2;
//			uiRigthLength = uiLengthSum / 2;
			
			ucDriveDistanceError = uiCurrentLocation - uiLeftLength;
			uiLeftLength  = uiLeftLength + ucDriveDistanceError;
			uiRigthLength = uiRigthLength - ucDriveDistanceError;
			datatemp[0] = uiLeftLength;
			datatemp[1] = uiRigthLength;
            
			datatemp[2] = uiRigthLength^uiLeftLength;
			datatemp[3] = 0xaaaa;
			
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flashд
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash��
			
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				sys_flags.wheel_CalibrationSuccess_Flag = 1;       //������0��У׼�ɹ���־λ
		}
		else                     //0�������ҡ�����ƶ�˿����ǰ����
		{
			ucDistanceAngle    = ucAngle;
//			ucDistanceDirction = ucSteeringDrection;
//			//�԰��˿�ܳ���
//			uiLeftLength = uiLengthSum / 2;
//			uiRigthLength = uiLengthSum / 2;
			
			ucDriveDistanceError = uiLeftLength - uiCurrentLocation;
			uiLeftLength = uiLeftLength - ucDriveDistanceError;
			uiRigthLength = uiRigthLength + ucDriveDistanceError;
			
			datatemp[0] = uiLeftLength;
			datatemp[1] = uiRigthLength;
			datatemp[2] = uiRigthLength^uiLeftLength;
			datatemp[3] = 0xaaaa;
			STMFLASH_Write(FLASH_DRIVER_ADDR,(u16*)datatemp,4);      //flashд
			STMFLASH_Read(FLASH_DRIVER_ADDR,(u16*)Left_RightLengh,4);//flash��
			if((Left_RightLengh[3]==datatemp[3])&&(Left_RightLengh[2]==datatemp[2]))
				sys_flags.wheel_CalibrationSuccess_Flag = 1;
		}
	}	
}

