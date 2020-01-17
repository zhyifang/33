#include "stm32f10x_it.h" 
#include "stdlib.h"

u8 uc1msFlag = 0;    //1msִ������
u16 uiHall   = 0;    //��¼����������ֵ
u8 uc1msCount = 0;   //1ms����ֵ
////1ms�жϣ����ڳ���ִ��
//void TIM2_IRQHandler(void)   //TIM2�ж�
//{
//	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //���TIM2�����жϷ������
//	{
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);     //���TIMx�����жϱ�־ 
//		uc1msFlag = 1;
//	}
//}
        
//�жϴ���Դ�����ش��������ɼ��ж�
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
	uiHall=GPIO_ReadInputData(GPIOB); //��ȡ����ֵ
	uiHall=uiHall&0x01c0;             //�ɼ�PB6,PB7,PB8
	uiHall=uiHall>>6;  
	
	
	if((uiHall&0x07)==ucForwardHall[HallTemp])
	{
		uiCurrentLocationHallCnt++;   //�����̸��ٵĺ���λ����
		MototStatus &= ~MOTOR_Status_SENSORFault;
	}
	else if((uiHall&0x07)==ucReverseHall[HallTemp])
	{
		uiCurrentLocationHallCnt--;
		MototStatus &= ~MOTOR_Status_SENSORFault;
	}
	else if(((uiHall&0x07)==0)||((uiHall&0x07)==7))  	//************������Ϣ����*****************//
	{
  	    MototStatus |= MOTOR_Status_SENSORFault;
		HALLerror_protect();        //hall���󱣻�
	}

	
	HallTemp=(u8)(uiHall&0x07);
	

	uiHallSpeedCnt++;           //У׼����˿�˵�λ����
	if((ucCalibrationSuccessFlag==1)||(ucCalibrationFailedFlag==1))
	{
		uiHallSpeedCnt = 0;
	}
    
	uiHallSpeedBackCnt++;       //hall�仯�����������������ٶ���
	Hall_SW();
    
  
	ucHallErr=ucHallLast^uiHall;    //���ķ�����ͬ��
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
	ucHallLast=uiHall;  //��һ�ε�HALLֵ
            
            
   if((uiHallA_Time>200)||(uiHallB_Time>200)||(uiHallC_Time>200))
   {
           
        MototStatus |= MOTOR_Status_Stall;
        motor_stalling_protect();          //�����ת����
           
        uiHallA_Time=0;
		uiHallB_Time=0;
        uiHallC_Time=0;
   }
}
/******************************************************************************************/
/*************�����жϣ�CntΪ���˺������ж�tim1�жϣ��ж�ʱ��62.5us,16kPWM��*****************/
/*********��Ƶ64M��PWM��ģʽΪUP and down2000,4000,****************************************/
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
        /************������Ϣ����*****************/
        if(((uiHall&0x07)==7))  	
	   {
  	    MototStatus |= MOTOR_Status_SENSORFault;
		HALLerror_protect();        //hall���󱣻�
       }
  
        uc1msCount++;
        if(uc1msCount==16){
            
            uc1msCount=0;
            uc1msFlag = 1;
        }
        
		if((TIM1->RCR==0)&&((TIM1->CNT)>100))
		{
			TIM1->RCR=1;          // ���Ȳ��������¼�
		}
        
        uiCurrentAD = ADC1->JDR1; //����AD   
		uiVoltageAD = ADC1->JDR2; //��ѹADֵ
		uiMosTemAD  = ADC1->JDR3; //�¶�ADֵ

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
        UserPower = UserVolatge10Times * UserCurrent10Times / 100;
        
		uiCurrentAD_Last = uiCurrentFbk;
		uiVoltageAD_Last = uiVoltageADFbk;
		uiMosTemAD_Last  = uiMosTemADFbk;
    	if((GPIOB->IDR & GPIO_Pin_9)==0)//�Ӵ����г̿�����
		{	                
           uiCurrentLocationHallCnt = 0;//���0��
		   uiHallSpeedCnt = 0;          //hall������˿��λ�ü�¼              
		}  		
        current_protect();              //��������
        voltage_protect();              //��ѹ����
         
        
        //����Ķ�ת�ж�  
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
//                    motor_stalling_protect();          //�����ת����                
//		         }               
//                 if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
//		         {
//		            MototStatus |= MOTOR_Status_Stall;
//                    motor_stalling_protect();          //�����ת����                
//		         }
//             }
//                  
//           else uiStall_time=0;

//         }
      
//����Ķ�ת�ж�  
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
                    motor_stalling_protect();          //�����ת����                
		         }               
                 if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
		         {
		            MototStatus |= MOTOR_Status_Stall;
                    motor_stalling_protect();          //�����ת����                
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
                 motor_stalling_protect();          //�����ת����                
		       }               
              if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
		      {
		         MototStatus |= MOTOR_Status_Stall;
                 motor_stalling_protect();          //�����ת����                
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
                 motor_stalling_protect();          //�����ת����                
		    }               
             if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
		    {
		         MototStatus |= MOTOR_Status_Stall;
                 motor_stalling_protect();          //�����ת����                
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
                 motor_stalling_protect();          //�����ת����                
		       }               
           if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
		       {
		         MototStatus |= MOTOR_Status_Stall;
                 motor_stalling_protect();          //�����ת����                
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
                   motor_stalling_protect();          //�����ת����                
               }               
               if((SetOK==0)&&((ucForewardStallFlag==1)||(ucReversedStallFlag==1)))
               { 
                   MototStatus |= MOTOR_Status_Stall;
                   motor_stalling_protect();          //�����ת����                
               }
       
           }
 
         }else uiStall_time=0;

        }
       
        
        
        if(open_power_flag_finish==1){	
            open_power_flag_finish=0;
            uiCurrentLocationHallCnt= Position_All;
        }
        CalculateDistance();                          //������ת�Ǹ��ٺ�����˿���˶����롣
    }
}




/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
