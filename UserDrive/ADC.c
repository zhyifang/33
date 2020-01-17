#include "ADC.h"
#define ADC1_DR_Address    ((u32)0x4001244C)
extern short ADC_ConvertedValue[];//DMAʱע����short��  int�Ļ� Ҫ����DMA
void ADC_Configuration1(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB, ENABLE);

    //GPIO��ADC����/PB0 ADCIN8-TEMP  PB1 ADCIN9-VBUS
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
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   // ��������ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;         // ɨ�跽ʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // ����ת��������
    
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;// �ⲿ������ֹ
	
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	ADC_InitStructure.ADC_NbrOfChannel = 2;//����ͨ����2����ʱû�õ�
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_ExternalTrigInjectedConvConfig(ADC1,ADC_ExternalTrigInjecConv_T1_TRGO);//ע���鴥��ԴΪTIM1�Ĵ���
	ADC_InjectedSequencerLengthConfig(ADC1,3);//ע���鳤��

	/* ADC1 regular channel14 configuration */ 

	ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);//ע�����������1,IBUS
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_1Cycles5);//ע�����������2,VBUS
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 3, ADC_SampleTime_1Cycles5);//ע�����������3,TEMP

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

//�ṹ�嶨��
adc_sample  ADC_Data;
void ADC_DMA_Configuration(void){
    GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef  ADC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructre;
	/* Enable ADC1 and  clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
/********GPIO_ADC������ ģ���******************/
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
/********DMA������ ����adc�Ĳ���******************/
    //�����ַADC��DMA��ʼ��������  
    DMA_InitStructre.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->JDR1;//ADC1->DR
    //DMA�������ݴ�����ڴ��ַ
    DMA_InitStructre.DMA_MemoryBaseAddr = (uint32_t)&ADC_Data;
    //DMA�������ݵķ��򣬴��������������
    DMA_InitStructre.DMA_DIR = DMA_DIR_PeripheralSRC;
    //DMA���˵�ͨ����������ͨ��
    DMA_InitStructre.DMA_BufferSize = 1;
    //����ĵ�ַ���Լ�����
    DMA_InitStructre.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    //�ڴ�ĵ�ַ�Լ�����
    DMA_InitStructre.DMA_MemoryInc = DMA_MemoryInc_Enable ;//DMA_MemoryInc_Disable    DMA_MemoryInc_Enable
    //�������ݴ�С
    DMA_InitStructre.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    //�ڴ����ݵĴ�С
    DMA_InitStructre.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord ;
    //DMA�������ݵ�ѭ��ģʽ
    DMA_InitStructre.DMA_Mode =  DMA_Mode_Circular;
    //DMA ���ȼ�������
    DMA_InitStructre.DMA_Priority = DMA_Priority_Medium;
    //DMA���ڴ浽�ڴ�
    DMA_InitStructre.DMA_M2M = DMA_M2M_Disable;
    
    DMA_Init(DMA1_Channel1, &DMA_InitStructre);
    DMA_Cmd(DMA1_Channel1,ENABLE);                     //DMAʹ�ܣ����а���
    
/********ADC���������ڵ�ѹ���������¶ȸ�������******************/      
    /* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   // ��������ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;         // ɨ�跽ʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // ����ת��������  
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;// �ⲿ������ֹ
	
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 2;              //�����飬ͨ����2����ʱû�õ�
	ADC_Init(ADC1, &ADC_InitStructure);
    //�����ǹ����鴥��
    
    
    //������ע����AD
	ADC_ExternalTrigInjectedConvConfig(ADC1,ADC_ExternalTrigInjecConv_T1_TRGO);//ע���鴥��ԴΪTIM1�Ĵ���

	ADC_InjectedSequencerLengthConfig(ADC1,3);//ע���鳤��
	/* ADC1 regular channel14 configuration */ 

    ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);//ע�����������1,IBUS
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_1Cycles5);//ע�����������2,VBUS
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 3, ADC_SampleTime_1Cycles5);//ע�����������3,TEMP
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
 * @brief ���ֲ���
 * @note ����������б����ҵ����֣��򷵻ض�Ӧ��Ԫ�ص����
 *       ��������в����ڱ����ҵ����֣��򷵻��������ڵ�ƫС��Ԫ�ص����
 * @retval int��
 */
int16_t UtilBiSearchInt(const int sortedIntArr[], int find, uint32_t maxN)
{
	int16_t low, mid, upper;
	low = 0;
	upper = maxN - 1;
	while (low <= upper)
	{
		mid = (low + upper) >> 1;//�൱�ڳ���2
		if (sortedIntArr[mid] < find)
			low = mid + 1;
		else if (sortedIntArr[mid] > find)
			upper = mid - 1;
		else
			return mid;
	}
	// ���Կ����˳���ѭ���� upper һ��С�� low(��ΪֻҪС�ڵ���low�ͼ�)
	return upper;
}

//�����¶�

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
//�Ŵ�10��
   return ((((ADC_Value*25)>>4)*33)>>8);  //ԭ������Ŵ�10��  
}


u16 Caculate_Vbus10Times(u16 ADC_Value)
{
    return (((ADC_Value)*(653>>6))>>6);
}
