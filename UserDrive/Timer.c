#include "Timer.h"
void TIM1_Configuration1(void)
{	
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM1_OCInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	/* TIM1 Registers reset */
	/* Enable TIM1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);//ʹ��APB2ʱ��
	TIM_DeInit(TIM1);
	
	/*PA8,PA9,PA10 Ϊ�°��ű�*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*PB13,PB14,PB15 Ϊ�ϰ��ű�*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_TimeBaseStructInit(&TIM1_TimeBaseStructure);
	/* Time Base configuration */
    
    TIM1_TimeBaseStructure.TIM_Period    = PWM_PERIOD-1;      //2000
	TIM1_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;//���¼���ģʽ
	
	TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;  //0
	TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);

	TIM1->CR2=0X0020;//�����¼���Ϊ����Դ

	TIM_OCStructInit(&TIM1_OCInitStructure);
	
	
//	/* Channel 1, 2,3 in PWM mode */
	TIM1_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;          //PWMģʽ1λ������ֵС��ccRx�Ƚ�ֵʱ��Ϊ��Ч
	TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; 
	TIM1_OCInitStructure.TIM_OutputNState= TIM_OutputNState_Enable;	
	TIM1_OCInitStructure.TIM_Pulse = 0x00;                           //dummy value 500
	TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;      //����Ϊ����Ч

	TIM_OC1Init(TIM1, &TIM1_OCInitStructure); 
	TIM_OC2Init(TIM1, &TIM1_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM1_OCInitStructure);  
    
    
    
	/* Enables the TIM1 Preload on CC1 Register */
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	/* Enables the TIM1 Preload on CC2 Register */
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	/* Enables the TIM1 Preload on CC3 Register */
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);


	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;             //TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

	TIM_ARRPreloadConfig(TIM1, ENABLE);                           //ʹ��TIM��ARR�ϵ�Ԥװ�ؼĴ���
    
	TIM_CtrlPWMOutputs(TIM1, DISABLE);                            //ʧ��PWM���
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_Cmd(TIM1, ENABLE);
}



void RCC_Configuration(void)
{
    //ErrorStatus HSEStartUpStatus;
    u32 tmpreg = 0;
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();
    /* Enable HSI */
    RCC_HSICmd(ENABLE);  //ʹ�õ��ڲ�ʱ��
    /* HCLK = SYSCLK */
    RCC->CFGR &=0xFFFFFF0F;//CFGR_HPRE_Reset_Mask;//
    /* PCLK2 = HCLK */
    RCC->CFGR &=0xFFFFC7FF;//CFGR_PPRE2_Reset_Mask;//
    /* PCLK1 = HCLK/2 */
    tmpreg = RCC->CFGR;//
    tmpreg &= 0xFFFFF8FF;//CFGR_PPRE1_Reset_Mask;
    tmpreg |= 0x00000400;//RCC_HCLK_Div2;
    RCC->CFGR = tmpreg;//APB1 2��Ƶ���
    /* Flash 2 wait state */
    FLASH->ACR &=0x00000038;//ACR_LATENCY_Mask;//
    FLASH->ACR |= 0x00000002;//FLASH_Latency_2;//
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH->ACR &= 0xFFFFFFEF;//ACR_PRFTBE_Mask;//
    FLASH->ACR |=0x00000010;//FLASH_PrefetchBuffer_Enable;//
    /* PLLCLK = 8MHz/2 * 16 = 64 MHz */
    tmpreg = RCC->CFGR;//
    tmpreg &= 0xFFC0FFFF;//CFGR_PLL_Mask;
    tmpreg |= 0x00388000;//RCC_PLLMul_16
    RCC->CFGR = tmpreg;
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    //asm("nop");//151126-1
    }
    /* Select PLL as system clock source */
    tmpreg = RCC->CFGR;//
    tmpreg &= 0xFFFFFFFC;//CFGR_SW_Mask;
    tmpreg |= 0x00000002;//RCC_SYSCLKSource_PLLCLK;
    RCC->CFGR = tmpreg;
    while(((u8)(RCC->CFGR & 0x0000000C)) != 0x08)//
    {
    //asm("nop");//151126-1
    }
    
    
}

void TIM2_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM2��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM2�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx					 
}

