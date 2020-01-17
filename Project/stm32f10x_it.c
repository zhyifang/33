#include "stm32f10x_it.h" 
#include "stdlib.h"

u8 uc1msFlag = 0;  //1msִ������
u16 uiHall   = 0;  //��¼����������ֵ

//1ms�жϣ����ڳ���ִ��
void TIM2_IRQHandler(void)   //TIM2�ж�
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM2�����жϷ������
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);     //���TIMx�����жϱ�־ 
		uc1msFlag = 1;
	}

}
        
//�жϴ���Դ�����ش��������ɼ��ж�
void EXTI9_5_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line6)!= RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
	if(EXTI_GetITStatus(EXTI_Line7)!= RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	if(EXTI_GetITStatus(EXTI_Line8)!= RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
	uiHall=GPIO_ReadInputData(GPIOB); //��ȡ����ֵ
	uiHall=uiHall&0x01c0;             //�ɼ�PB6,PB7,PB8
	uiHall=uiHall>>6;  
	if((uiHall&0x07)==ucForwardHall[HallTemp])
	{
		uiCurrentLocationHallCnt++;
		MototStatus &= ~MOTOR_Status_SENSORFault;
	}
	else if((uiHall&0x07)==ucReverseHall[HallTemp])
	{
		uiCurrentLocationHallCnt--;
		MototStatus &= ~MOTOR_Status_SENSORFault;
	}
	/************������Ϣ����*****************/
	else if(((uiHall&0x07)!=ucReverseHall[HallTemp])||((uiHall&0x07)!=ucForwardHall[HallTemp]))
	{
		//GPIOC->ODR ^= GPIO_Pin_13;
		MototStatus |= MOTOR_Status_SENSORFault;
	}
	HallTemp=(u8)(uiHall&0x07);

    HALLerror_protect();      //hall���󱣻�

	uiHallSpeedCnt++;  //У׼����˿�˵�λ��
	if((ucCalibrationSuccessFlag==1)||(ucCalibrationFailedFlag==1))
	{
		uiHallSpeedCnt = 0;
	}
    
	uiHallSpeedBackCnt++;    //hall�仯�����������������ٶ�
	Hall_SW();

}

/*****************************************************************/
/*************tim1�жϣ��ж�ʱ��62.5us,16kPWM��*******************/
/*********��Ƶ64M��PWM��ģʽΪUP and down2000,4000,***************/
/*****************************************************************/
u16 uiCurrentAD_Last = 0;
u16 uiVoltageAD_Last = 0;
u16 uiMosTemAD_Last  = 0;
u16 uiHallA_Time=0;
u16 uiHallB_Time=0;
u16 uiHallC_Time=0;
void TIM1_UP_IRQHandler(void)
{
	static u8 ucHallLast=0;
	u8 ucHallErr=0;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		if((TIM1->RCR==0)&&((TIM1->CNT)>100))
		{
			TIM1->RCR=1;// ���Ȳ��������¼�
		}
		if(uiHallSpeedTimeCnt<10000)  //û�õ�
		{
			uiHallSpeedTimeCnt++;
		}   
		uiVoltageAD = ADC1->JDR2; //��ѹADֵ
		uiMosTemAD  = ADC1->JDR3; //�¶�ADֵ
        uiCurrentAD = ADC1->JDR1; //����ADֵ
		if(uiCurrentAD<uiCurrentZero)
		{
			uiCurrentAD = uiCurrentZero - ADC1->JDR1;    
		}
		else
		{
			uiCurrentAD = 0;
		}
        //uiCurrentAD = 41.25-ADC1->JDR1/4095*82.5f;����˲��е�
		uiVoltageADFbk = uiVoltageAD_Last*3/4+uiVoltageAD/4;
		uiCurrentFbk   = uiCurrentAD_Last*3/4+uiCurrentAD/4; 
        uiMosTemADFbk  = uiMosTemAD_Last*3/4+uiMosTemAD/4;
        
		UserCurrent10Times = Caculate_Current(uiCurrentFbk);
		UserVolatge10Times = Caculate_Vbus10Times(uiVoltageADFbk);
        
		uiCurrentAD_Last = uiCurrentFbk;
		uiVoltageAD_Last = uiVoltageADFbk;
		uiMosTemAD_Last  = uiMosTemADFbk;
        
        current_protect();                              //��������
        voltage_protect();                              //��ѹ����
		if(uiHallA_Time<65000)
		{
			uiHallA_Time++;
		}
		if(uiHallB_Time<65000)
		{
			uiHallB_Time++;
		}
		if(uiHallC_Time<65000)
		{
			uiHallC_Time++;
		}
		if(keytemp ==2)
		{
			uiHallA_Time=0;
			uiHallB_Time=0;
			uiHallC_Time=0;
		}
		if(ucHallLast!=uiHall)
		{
			ucHallErr=ucHallLast^uiHall;
			if(ucHallErr&0x01)
			{
				uiHallA_Time=0;
				uiHallB_Time=0;
				uiHallC_Time=0;
			}
			if(ucHallErr&0x02)
			{
				uiHallA_Time=0;
				uiHallB_Time=0;
				uiHallC_Time=0;
			}
			if(ucHallErr&0x04)
			{
				uiHallA_Time=0;
				uiHallB_Time=0;
				uiHallC_Time=0;
			}
			ucHallLast=uiHall;  //��һ�ε�HALLֵ
            //��ߵ�speedsum��û����
			SpeedData.NewData = uiHallSpeedTimeCnt;
			SpeedSum(&SpeedData);	//42		       
			uiHallSpeedTimeCnt=0;   //����
		}
        //����Ķ�ת�ж���
		if((uiHallA_Time>30000)||(uiHallB_Time>30000)||(uiHallC_Time>30000))//62.5us*30000=1.8��
		{

			if(keytemp==1)
			{

				if(ucMotorDrection==0)
				{
					ucForewardStallFlag=1;
				}
				else
				{
					ucReversedStallFlag=1;
				}
                
				if((ucCalibrationSuccessFlag==1)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
				{
					MototStatus |= MOTOR_Status_Stall;
                    motor_stalling_protect();      //�����ת����                
				}
                uiHallA_Time=0;
				uiHallB_Time=0;
				uiHallC_Time=0;
			}

		}
        
		uiCurrentLocation=uiCurrentLocationHallCnt;   //��������hallֵ����ǰλ��hall����
		CalculateDistance();                          //���㷽����ת�ǣ�˿���˶����롣
        if(keytemp==1)
		{
           current_loop();                                 //����������
           power_loop();                                   //���ʻ�����
		}
	}
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
