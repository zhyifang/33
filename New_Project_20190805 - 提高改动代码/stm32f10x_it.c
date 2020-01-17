#include "stm32f10x_it.h" 
#include "stdlib.h"

u8 uc1msFlag = 0;    //1ms执行周期
u16 uiHall   = 0;    //记录霍尔传感器值
u8 uc1msCount = 0;   //1ms计数值
////1ms中断，用于程序执行
//void TIM2_IRQHandler(void)   //TIM2中断
//{
//	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2更新中断发生与否
//	{
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);     //清除TIMx更新中断标志 
//		uc1msFlag = 1;
//	}
//}
        
//中断触发源上下沿触发霍尔采集中断
u16 uiHallA_Time=0;
u16 uiHallB_Time=0;
u16 uiHallC_Time=0;
u8  ucHallLast=0;
//u8 uctempLast=0;
void EXTI9_5_IRQHandler(void)
{
	
	u8 ucHallErr=0;
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
		uiCurrentLocationHallCnt++;   //方向盘跟踪的函数位置用
		MototStatus &= ~MOTOR_Status_SENSORFault;
	}
	else if((uiHall&0x07)==ucReverseHall[HallTemp])
	{
		uiCurrentLocationHallCnt--;
		MototStatus &= ~MOTOR_Status_SENSORFault;
	}
	else if(((uiHall&0x07)==0)||((uiHall&0x07)==7))  	//************霍尔信息错误*****************//
	{
  	    MototStatus |= MOTOR_Status_SENSORFault;
		HALLerror_protect();        //hall错误保护
	}

	
	HallTemp=(u8)(uiHall&0x07);
	

	uiHallSpeedCnt++;           //校准左右丝杆的位置用
	if((ucCalibrationSuccessFlag==1)||(ucCalibrationFailedFlag==1))
	{
		uiHallSpeedCnt = 0;
	}
    
	uiHallSpeedBackCnt++;       //hall变化，用来计算电机反馈速度用
	Hall_SW();
    
  
	ucHallErr=ucHallLast^uiHall;    //异或的反就是同或
    if(ucHallErr&0x01)
	{
		uiHallA_Time++;

	}
    else{
            
        uiHallA_Time=0;
            
    }
	if(ucHallErr&0x02)
	{

		uiHallB_Time++;

	}
    else{
 
        uiHallB_Time=0;
    }
	if(ucHallErr&0x04)
	{

		uiHallC_Time++;
	}
    else{
            
        uiHallC_Time=0;
            
    }
	ucHallLast=uiHall;  //上一次的HALL值
            
            
   if((uiHallA_Time>200)||(uiHallB_Time>200)||(uiHallC_Time>200))
   {
           
        MototStatus |= MOTOR_Status_Stall;
        motor_stalling_protect();          //电机堵转保护
           
        uiHallA_Time=0;
		uiHallB_Time=0;
        uiHallC_Time=0;
   }
}
/******************************************************************************************/
/*************更新中断，Cnt为零伤害触发中断tim1中断，中断时间62.5us,16kPWM，*****************/
/*********主频64M，PWM的模式为UP and down2000,4000,****************************************/
/******************************************************************************************/
u16 uiCurrentAD_Last = 0;
u16 uiVoltageAD_Last = 0;
u16 uiMosTemAD_Last  = 0;

u16 uiStall_time=0;
//u16 uiVotage[5]={80,180,280,380,480};
//u16 uiCurrent[5]={50,40,30,25,20,};
//u8 index_VC=0;
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); 
        /************霍尔信息错误*****************/
        if(((uiHall&0x07)==7))  	
	   {
  	    MototStatus |= MOTOR_Status_SENSORFault;
		HALLerror_protect();        //hall错误保护
       }
  
        uc1msCount++;
        if(uc1msCount==16){
            
            uc1msCount=0;
            uc1msFlag = 1;
        }
        
		if((TIM1->RCR==0)&&((TIM1->CNT)>100))
		{
			TIM1->RCR=1;          // 波谷产生更新事件
		}
        
        uiCurrentAD = ADC1->JDR1; //电流AD   
		uiVoltageAD = ADC1->JDR2; //电压AD值
		uiMosTemAD  = ADC1->JDR3; //温度AD值

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
        UserPower = UserVolatge10Times * UserCurrent10Times / 100;
        
		uiCurrentAD_Last = uiCurrentFbk;
		uiVoltageAD_Last = uiVoltageADFbk;
		uiMosTemAD_Last  = uiMosTemADFbk;
    	if((GPIOB->IDR & GPIO_Pin_9)==0)//接触到行程开关了
		{	                
           uiCurrentLocationHallCnt = 0;//标记0点
		   uiHallSpeedCnt = 0;          //hall计数，丝杆位置记录              
		}  		
        current_protect();              //电流保护
        voltage_protect();              //电压保护
         
        
        //电机的堵转判定  
//         if((keytemp==1)&&(SpeedPara.Fbk==0))
//         {
//             uiStall_time=uiStall_time+1;
//              if(uiStall_time>20000){
//                  uiStall_time=0;
//                 if(ucMotorDrection==0)
//			     {
//				     ucForewardStallFlag=1;
//			     }
//			     else
//			     {
//				     ucReversedStallFlag=1;
//                 }
//                 if((SetOK==1)&&((ucForewardStallFlag==1)))
//		         {
//		            MototStatus |= MOTOR_Status_Stall;
//                    motor_stalling_protect();          //电机堵转保护                
//		         }               
//                 if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
//		         {
//		            MototStatus |= MOTOR_Status_Stall;
//                    motor_stalling_protect();          //电机堵转保护                
//		         }
//             }
//                  
//           else uiStall_time=0;

//         }
      
//电机的堵转判定  
		if(UserVolatge10Times >480){
		
		  if((keytemp==1)&&(SpeedPara.Fbk<=300)&&(UserCurrent10Times >20)){
             
              uiStall_time=uiStall_time+1;
              if(uiStall_time>20000){
                  uiStall_time=0;
                 if(ucMotorDrection==0)
			     {
				     ucForewardStallFlag=1;
			     }
			     else
			     {
				     ucReversedStallFlag=1;
                 }
                 if((SetOK==1)&&((ucForewardStallFlag==1)))
		         {
		            MototStatus |= MOTOR_Status_Stall;
                    motor_stalling_protect();          //电机堵转保护                
		         }               
                 if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
		         {
		            MototStatus |= MOTOR_Status_Stall;
                    motor_stalling_protect();          //电机堵转保护                
		         }
             }
                  
           }else uiStall_time=0;
		}
		else if(UserVolatge10Times >380)
		{
		  if((keytemp==1)&&(SpeedPara.Fbk<=300)&&(UserCurrent10Times >25)){
             
             uiStall_time=uiStall_time+1;
             if(uiStall_time>20000){
             uiStall_time=0;
             if(ucMotorDrection==0)
		     {
				     ucForewardStallFlag=1;
			  }
			 else
			 {
				     ucReversedStallFlag=1;
              }
				
              if((SetOK==1)&&((ucForewardStallFlag==1)))
		      {
		         MototStatus |= MOTOR_Status_Stall;
                 motor_stalling_protect();          //电机堵转保护                
		       }               
              if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
		      {
		         MototStatus |= MOTOR_Status_Stall;
                 motor_stalling_protect();          //电机堵转保护                
		       }
             }
                  
          }else uiStall_time=0;
		
		}
		else if(UserVolatge10Times >280)
		{
		  if((keytemp==1)&&(SpeedPara.Fbk<=300)&&(UserCurrent10Times >30)){
             
           uiStall_time=uiStall_time+1;
           if(uiStall_time>20000){
           uiStall_time=0;
           if(ucMotorDrection==0)
			{
			   ucForewardStallFlag=1;
			}
			else
			{
				ucReversedStallFlag=1;
            }
				
            if((SetOK==1)&&((ucForewardStallFlag==1)))
		    {
		         MototStatus |= MOTOR_Status_Stall;
                 motor_stalling_protect();          //电机堵转保护                
		    }               
             if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
		    {
		         MototStatus |= MOTOR_Status_Stall;
                 motor_stalling_protect();          //电机堵转保护                
		    }
          }
                  
         }else uiStall_time=0;
		
		}
		else if(UserVolatge10Times >180)
		{
		 if((keytemp==1)&&(SpeedPara.Fbk<=300)&&(UserCurrent10Times >40)){
             
           uiStall_time=uiStall_time+1;
           if(uiStall_time>20000){
           uiStall_time=0;
           if(ucMotorDrection==0)
			{
				     ucForewardStallFlag=1;
			 }
		   else
			{
				     ucReversedStallFlag=1;
           }
				
           if((SetOK==1)&&((ucForewardStallFlag==1)))
		       {
		         MototStatus |= MOTOR_Status_Stall;
                 motor_stalling_protect();          //电机堵转保护                
		       }               
           if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
		       {
		         MototStatus |= MOTOR_Status_Stall;
                 motor_stalling_protect();          //电机堵转保护                
		       }
           }
                  
          }else uiStall_time=0;
		
		}
		else if(UserVolatge10Times >80)
		{
		 if((keytemp==1)&&(SpeedPara.Fbk<=300)&&(UserCurrent10Times >50)){
             
           uiStall_time=uiStall_time+1;
           if(uiStall_time>20000){

               uiStall_time=0;     
               if(ucMotorDrection==0)    
               {
                   ucForewardStallFlag=1;
		
               }
               else
               {
                   ucReversedStallFlag=1;
       
               }
               if((SetOK==1)&&((ucForewardStallFlag==1)))
               {
                   MototStatus |= MOTOR_Status_Stall;
                   motor_stalling_protect();          //电机堵转保护                
               }               
               if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
               { 
                   MototStatus |= MOTOR_Status_Stall;
                   motor_stalling_protect();          //电机堵转保护                
               }
       
           }
 
         }else uiStall_time=0;

        }
       
        
        
        if(open_power_flag_finish==1){	
            open_power_flag_finish=0;
            uiCurrentLocationHallCnt= Position_All;
        }
        CalculateDistance();                          //方向盘转角跟踪函数，丝杠运动距离。
    }
}




/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
