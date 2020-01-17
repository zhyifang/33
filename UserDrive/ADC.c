#include "ADC.h"
#define ADC1_DR_Address    ((u32)0x4001244C)
extern short ADC_ConvertedValue[];//DMA时注意用short型  int的话 要调整DMA
void ADC_Configuration1(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB, ENABLE);

    //GPIO的ADC配置/PB0 ADCIN8-TEMP  PB1 ADCIN9-VBUS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PA1 ADCIN1-IBUS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   // 独立工作模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;         // 扫描方式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // 连续转换不是能
    
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;// 外部触发禁止
	
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	ADC_InitStructure.ADC_NbrOfChannel = 2;//规则通道数2，暂时没用到
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_ExternalTrigInjectedConvConfig(ADC1,ADC_ExternalTrigInjecConv_T1_TRGO);//注入组触发源为TIM1的触发
	ADC_InjectedSequencerLengthConfig(ADC1,3);//注入组长度

	/* ADC1 regular channel14 configuration */ 

	ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);//注入组采样序列1,IBUS
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_1Cycles5);//注入组采样序列2,VBUS
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 3, ADC_SampleTime_1Cycles5);//注入组采样序列3,TEMP

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE);

}

//结构体定义
adc_sample  ADC_Data;
void ADC_DMA_Configuration(void){
    GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef  ADC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructre;
	/* Enable ADC1 and  clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
/********GPIO_ADC的配置 模拟口******************/
    //PB0 ADCIN8-TEMP  PB1 ADCIN9-VBUS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PA1 ADCIN1-IBUS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
/********DMA的配置 用于adc的采用******************/
    //外设地址ADC，DMA开始搬运数据  
    DMA_InitStructre.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->JDR1;//ADC1->DR
    //DMA搬运数据存放在内存地址
    DMA_InitStructre.DMA_MemoryBaseAddr = (uint32_t)&ADC_Data;
    //DMA搬运数据的方向，从外往里搬运数据
    DMA_InitStructre.DMA_DIR = DMA_DIR_PeripheralSRC;
    //DMA搬运的通道，有三个通道
    DMA_InitStructre.DMA_BufferSize = 1;
    //外设的地址不自己增加
    DMA_InitStructre.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    //内存的地址自己增加
    DMA_InitStructre.DMA_MemoryInc = DMA_MemoryInc_Enable ;//DMA_MemoryInc_Disable    DMA_MemoryInc_Enable
    //外设数据大小
    DMA_InitStructre.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    //内存数据的大小
    DMA_InitStructre.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord ;
    //DMA搬运数据的循环模式
    DMA_InitStructre.DMA_Mode =  DMA_Mode_Circular;
    //DMA 优先级的配置
    DMA_InitStructre.DMA_Priority = DMA_Priority_Medium;
    //DMA由内存到内存
    DMA_InitStructre.DMA_M2M = DMA_M2M_Disable;
    
    DMA_Init(DMA1_Channel1, &DMA_InitStructre);
    DMA_Cmd(DMA1_Channel1,ENABLE);                     //DMA使能，进行搬运
    
/********ADC的配置用于电压，电流和温度给定配置******************/      
    /* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   // 独立工作模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;         // 扫描方式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // 连续转换不是能  
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;// 外部触发禁止
	
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 2;              //规则组，通道数2，暂时没用到
	ADC_Init(ADC1, &ADC_InitStructure);
    //以上是规则组触发
    
    
    //以下是注入组AD
	ADC_ExternalTrigInjectedConvConfig(ADC1,ADC_ExternalTrigInjecConv_T1_TRGO);//注入组触发源为TIM1的触发

	ADC_InjectedSequencerLengthConfig(ADC1,3);//注入组长度
	/* ADC1 regular channel14 configuration */ 

    ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);//注入组采样序列1,IBUS
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_1Cycles5);//注入组采样序列2,VBUS
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 3, ADC_SampleTime_1Cycles5);//注入组采样序列3,TEMP
	/* Enable ADC1 */
    
    ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE);

}








const int temRef[] = 
{
467	,
490	,
513	,
537	,
562	,
587	,
614	,
641	,
669	,
697	,
727	,
757	,
787	,
819	,
851	,
884	,
917	,
951	,
986	,
1021	,
1057	,
1094	,
1131	,
1168	,
1206	,
1244	,
1283	,
1322	,
1361	,
1401	,
1441	,
1481	,
1521	,
1562	,
1602	,
1643	,
1684	,
1724	,
1765	,
1806	,
1846	,
1887	,
1927	,
1967	,
2007	,
2047	,
2086	,
2126	,
2164	,
2203	,
2241	,
2279	,
2316	,
2353	,
2389	,
2425	,
2461	,
2496	,
2531	,
2565	,
2598	,
2631	,
2664	,
2696	,
2727	,
2758	,
2788	,
2818	,
2847	,
2876	,
2904	,
2931	,
2958	,
2985	,
3010	,
3036	,
3061	,
3085	,
3109	,
3132	,
3154	,
3177	,
3198	,
3219	,
3240	,
3260	,
3280	,
3299	,
3318	,
3336	,
3354	,
3372	,
3389	,
3406	,
3422	,
3438	,
3453	,
3468	,
3483	,
3497	,
3511	,
3525	,
3538	,
3551	,
3564	,
3576	,
3588	,
3600	,
3611	,
3622	,
3633	,
3643	,
3654	,
3664	,
3673	,
3683	,
3692	,
3701	,
3710	,
3719	,
3727	,
3735	,
3743	,
3751	,
3758	,
3766	,
};

/**
 * @brief 二分查找
 * @note 如果数组中有被查找的数字，则返回对应的元素的序号
 *       如果数组中不存在被查找的数字，则返回与他相邻的偏小的元素的序号
 * @retval int型
 */
int16_t UtilBiSearchInt(const int sortedIntArr[], int find, uint32_t maxN)
{
	int16_t low, mid, upper;
	low = 0;
	upper = maxN - 1;
	while (low <= upper)
	{
		mid = (low + upper) >> 1;//相当于除以2
		if (sortedIntArr[mid] < find)
			low = mid + 1;
		else if (sortedIntArr[mid] > find)
			upper = mid - 1;
		else
			return mid;
	}
	// 可以看到退出上循环后 upper 一定小于 low(因为只要小于等于low就加)
	return upper;
}

//计算温度

int16_t Caculate_Tem(u16 temp_val)
{
	return UtilBiSearchInt(temRef,temp_val,128);
	 
}

u16 Caculate_CurrentZero(char times)
{
	u16 CurrentSum = 0;
	char i;
	for(i=0;i<times;i++)
	{
        while(!(ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC))){};      
		CurrentSum+=ADC1->JDR1;
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
	}
	return (CurrentSum/times);
}

u16 Caculate_Current(u16 ADC_Value)
{
//	static u16 AD;
//    AD=((165-(330*ADC_Value)/4095)*5)>>1; 
//    return (AD);
//放大10倍
   return ((((ADC_Value*25)>>4)*33)>>8);  //原创程序放大10倍  
}


u16 Caculate_Vbus10Times(u16 ADC_Value)
{
    return (((ADC_Value)*(653>>6))>>6);
}
