#include "stm32f10x_it.h" 
#include "stdlib.h"

u8 uc1msFlag = 0;  //1ms执行周期
u16 uiHall   = 0;  //记录霍尔传感器值

//1ms中断，用于程序执行
void TIM2_IRQHandler(void)   //TIM2中断
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);     //清除TIMx更新中断标志 
		uc1msFlag = 1;
	}

}
        
//中断触发源上下沿触发霍尔采集中断
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
	uiHall=GPIO_ReadInputData(GPIOB); //读取引脚值
	uiHall=uiHall&0x01c0;             //采集PB6,PB7,PB8
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
	/************霍尔信息错误*****************/
	else if(((uiHall&0x07)!=ucReverseHall[HallTemp])||((uiHall&0x07)!=ucForwardHall[HallTemp]))
	{
		//GPIOC->ODR ^= GPIO_Pin_13;
		MototStatus |= MOTOR_Status_SENSORFault;
	}
	HallTemp=(u8)(uiHall&0x07);

    HALLerror_protect();      //hall错误保护

	uiHallSpeedCnt++;  //校准左右丝杆的位置
	if((ucCalibrationSuccessFlag==1)||(ucCalibrationFailedFlag==1))
	{
		uiHallSpeedCnt = 0;
	}
    
	uiHallSpeedBackCnt++;    //hall变化，用来计算电机反馈速度
	Hall_SW();

}

/*****************************************************************/
/*************tim1中断，中断时间62.5us,16kPWM，*******************/
/*********主频64M，PWM的模式为UP and down2000,4000,***************/
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
			TIM1->RCR=1;// 波谷产生更新事件
		}
		if(uiHallSpeedTimeCnt<10000)  //没用到
		{
			uiHallSpeedTimeCnt++;
		}   
		uiVoltageAD = ADC1->JDR2; //电压AD值
		uiMosTemAD  = ADC1->JDR3; //温度AD值
        uiCurrentAD = ADC1->JDR1; //电流AD值
		if(uiCurrentAD<uiCurrentZero)
		{
			uiCurrentAD = uiCurrentZero - ADC1->JDR1;    
		}
		else
		{
			uiCurrentAD = 0;
		}
        //uiCurrentAD = 41.25-ADC1->JDR1/4095*82.5f;这边滤波有点
		uiVoltageADFbk = uiVoltageAD_Last*3/4+uiVoltageAD/4;
		uiCurrentFbk   = uiCurrentAD_Last*3/4+uiCurrentAD/4; 
        uiMosTemADFbk  = uiMosTemAD_Last*3/4+uiMosTemAD/4;
        
		UserCurrent10Times = Caculate_Current(uiCurrentFbk);
		UserVolatge10Times = Caculate_Vbus10Times(uiVoltageADFbk);
        
		uiCurrentAD_Last = uiCurrentFbk;
		uiVoltageAD_Last = uiVoltageADFbk;
		uiMosTemAD_Last  = uiMosTemADFbk;
        
        current_protect();                              //电流保护
        voltage_protect();                              //电压保护
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
			ucHallLast=uiHall;  //上一次的HALL值
            //这边的speedsum的没有用
			SpeedData.NewData = uiHallSpeedTimeCnt;
			SpeedSum(&SpeedData);	//42		       
			uiHallSpeedTimeCnt=0;   //清零
		}
        //电机的堵转判定，
		if((uiHallA_Time>30000)||(uiHallB_Time>30000)||(uiHallC_Time>30000))//62.5us*30000=1.8秒
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
                    motor_stalling_protect();      //电机堵转保护                
				}
                uiHallA_Time=0;
				uiHallB_Time=0;
				uiHallC_Time=0;
			}

		}
        
		uiCurrentLocation=uiCurrentLocationHallCnt;   //用来计算hall值，当前位置hall计数
		CalculateDistance();                          //计算方向盘转角，丝杠运动距离。
        if(keytemp==1)
		{
           current_loop();                                 //电流环限流
           power_loop();                                   //功率环限流
		}
	}
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
