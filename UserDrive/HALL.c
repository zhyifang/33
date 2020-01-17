#include "HALL.h"
#include "MotorControl.h"
u16 My_PWM = 400;    //����max 2000
extern u16 uiHall;
void Hall_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	/* ����Hall�ӿ�IO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*�����ź����ж�����*/
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);
 
	EXTI_InitStructure.EXTI_Line = EXTI_Line6|EXTI_Line7|EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;    //�������½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
u8 ucHallTemp=0;

void Hall_SW(void) //��������
{
	if(ucMotorDrection==1)           //�������ת�Ŀ���
	{
		ucHallTemp=(u8)(uiHall^0x07);//���ȡ������
	}
	else
	{
		ucHallTemp=uiHall;
	}
	//test
	switch(ucHallTemp)
	{
		
//		case 1:
		case 5:     //original 5
//                TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);//PWM2�� 3�����
//                TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_1 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_1, TIM_CCx_Enable);//ʹ��
        
				TIM1->CCR1 = My_PWM;           //AB  ���UV��ͨ
				TIM1->CCR2 = 0; 
				TIM1->CCR3 = 0;
                
				GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_10);
				GPIO_SetBits(GPIOA, GPIO_Pin_9);

				break;
//		case 3:
		case 4:  //original 1
//                TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);//PWM2�� 3�����
//                TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_1 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_1, TIM_CCx_Enable);//ʹ��
				TIM1->CCR1 = My_PWM;            //AC  ���UW��ͨ
				TIM1->CCR2 = 0;
				TIM1->CCR3 = 0;
                
				GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9);
				GPIO_SetBits(GPIOA, GPIO_Pin_10);


				break;
//		case 2:
		case 6:  //original 5
		/* Next step: Step 4 Configuration ---------------------------- */
//                TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);//PWM2�� 3�����
//                TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_2 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_2, TIM_CCx_Enable);      //ʹ��
				TIM1->CCR1 = 0;          //BC     ���VW��ͨ
				TIM1->CCR2 = My_PWM;
				TIM1->CCR3 = 0;
                
				GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9);
				GPIO_SetBits(GPIOA, GPIO_Pin_10);

				break;

//		case 6: 
		case 2:    //original 2
		/* Next step: Step 5 Configuration ---------------------------- */
//		        TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);//PWM2�� 3�����
//                TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_2 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_2, TIM_CCx_Enable);      //ʹ��
				TIM1->CCR1 = 0;        //BA        ���VU��ͨ
				TIM1->CCR2 = My_PWM;
				TIM1->CCR3 = 0;
                
			    GPIO_ResetBits(GPIOA, GPIO_Pin_9 | GPIO_Pin_10);
			    GPIO_SetBits(GPIOA, GPIO_Pin_8);

        
				break;

//		case 4:
		case 3:   //original 6
		/* Next step: Step 6 Configuration ---------------------------- */
//			    TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);//PWM2�� 3�����
//                TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_3 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_3, TIM_CCx_Enable);//ʹ��
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0;//CA                ���WU��ͨ
				TIM1->CCR3 = My_PWM;
                
			    GPIO_ResetBits(GPIOA, GPIO_Pin_9 | GPIO_Pin_10);
			    GPIO_SetBits(GPIOA, GPIO_Pin_8);

				break;

//		case 5: 
		case 1:     //original 4
		/* Next step: Step 1 Configuration ---------------------------- */
//			    TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);//PWM2�� 3�����
//                TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_3 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_3, TIM_CCx_Enable);//ʹ��
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0; //CB                ���WV��ͨ
				TIM1->CCR3 = My_PWM;
                
			    GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_10);
			    GPIO_SetBits(GPIOA, GPIO_Pin_9);

				break;
		default:
				break;
	}
}
void Read_Hall_number(void){
    
    uiHall=GPIO_ReadInputData(GPIOB);        //��ȡ����ֵ
    uiHall=uiHall&0x01c0;
    uiHall=uiHall>>6; 
}
    




