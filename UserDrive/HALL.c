#include "HALL.h"
#include "MotorControl.h"
u16 My_PWM = 400;    //设置max 2000
extern u16 uiHall;
void Hall_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	/* 配置Hall接口IO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*霍尔信号线中断配置*/
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);
 
	EXTI_InitStructure.EXTI_Line = EXTI_Line6|EXTI_Line7|EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;    //上升沿下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
u8 ucHallTemp=0;

void Hall_SW(void) //六步换向
{
	if(ucMotorDrection==1)           //电机正反转的控制
	{
		ucHallTemp=(u8)(uiHall^0x07);//异或取反方向
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
//                TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);//PWM2和 3不输出
//                TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_1 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_1, TIM_CCx_Enable);//使能
        
				TIM1->CCR1 = My_PWM;           //AB  电机UV线通
				TIM1->CCR2 = 0; 
				TIM1->CCR3 = 0;
                
				GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_10);
				GPIO_SetBits(GPIOA, GPIO_Pin_9);

				break;
//		case 3:
		case 4:  //original 1
//                TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);//PWM2和 3不输出
//                TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_1 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_1, TIM_CCx_Enable);//使能
				TIM1->CCR1 = My_PWM;            //AC  电机UW线通
				TIM1->CCR2 = 0;
				TIM1->CCR3 = 0;
                
				GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9);
				GPIO_SetBits(GPIOA, GPIO_Pin_10);


				break;
//		case 2:
		case 6:  //original 5
		/* Next step: Step 4 Configuration ---------------------------- */
//                TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);//PWM2和 3不输出
//                TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_2 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_2, TIM_CCx_Enable);      //使能
				TIM1->CCR1 = 0;          //BC     电机VW线通
				TIM1->CCR2 = My_PWM;
				TIM1->CCR3 = 0;
                
				GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_9);
				GPIO_SetBits(GPIOA, GPIO_Pin_10);

				break;

//		case 6: 
		case 2:    //original 2
		/* Next step: Step 5 Configuration ---------------------------- */
//		        TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);//PWM2和 3不输出
//                TIM_ForcedOC3Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_2 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_2, TIM_CCx_Enable);      //使能
				TIM1->CCR1 = 0;        //BA        电机VU线通
				TIM1->CCR2 = My_PWM;
				TIM1->CCR3 = 0;
                
			    GPIO_ResetBits(GPIOA, GPIO_Pin_9 | GPIO_Pin_10);
			    GPIO_SetBits(GPIOA, GPIO_Pin_8);

        
				break;

//		case 4:
		case 3:   //original 6
		/* Next step: Step 6 Configuration ---------------------------- */
//			    TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);//PWM2和 3不输出
//                TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_3 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_3, TIM_CCx_Enable);//使能
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0;//CA                电机WU线通
				TIM1->CCR3 = My_PWM;
                
			    GPIO_ResetBits(GPIOA, GPIO_Pin_9 | GPIO_Pin_10);
			    GPIO_SetBits(GPIOA, GPIO_Pin_8);

				break;

//		case 5: 
		case 1:     //original 4
		/* Next step: Step 1 Configuration ---------------------------- */
//			    TIM_ForcedOC1Config(TIM1, TIM_ForcedAction_InActive);//PWM2和 3不输出
//                TIM_ForcedOC2Config(TIM1, TIM_ForcedAction_InActive);//
//                TIM_SelectOCxM(TIM1,TIM_Channel_3 , TIM_OCMode_PWM1);
//                TIM_CCxCmd(TIM1,TIM_Channel_3, TIM_CCx_Enable);//使能
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0; //CB                电机WV线通
				TIM1->CCR3 = My_PWM;
                
			    GPIO_ResetBits(GPIOA, GPIO_Pin_8 | GPIO_Pin_10);
			    GPIO_SetBits(GPIOA, GPIO_Pin_9);

				break;
		default:
				break;
	}
}
void Read_Hall_number(void){
    
    uiHall=GPIO_ReadInputData(GPIOB);        //读取引脚值
    uiHall=uiHall&0x01c0;
    uiHall=uiHall>>6; 
}
    




