#include "Protect.h"

/*******************************************
���ƣ�void voltage_protect(void)
���ܣ���ѹ��������
������None
���ߣ�zyf

*******************************************/ 
void voltage_protect(void){
     static u8 voltageprotimes;
   /*****��ѹ��ѹ����ǷѹҪһ��ʱ�䣬�ͽ��е�ѹ����******/
     if((MototStatus&0x02)==MOTOR_Status_MotOV||(MototStatus&0x04)==MOTOR_Status_MotUV){
        voltageprotimes++;
        if(voltageprotimes>100){
            voltageprotimes=0;
            keytemp=2;
            motor_stop();     
        } 

    }else voltageprotimes=0;
      
}
 
/*******************************************
���ƣ�void current_protect(void)
���ܣ�������������
������None
���ߣ�zyf
*******************************************/ 
void current_protect(void){
    static u8 currentprotimes;
    /*****��������ת��С��10����Ҫһ��ʱ�䣬�ͽ��е�������******/
    if((MototStatus&0x08)==MOTOR_Status_MotOC){
        currentprotimes++;
        if(currentprotimes>100){
            currentprotimes=0;
            keytemp=2;
            motor_stop();     
       } 

    }
    else currentprotimes=0;
}
/*******************************************
���ƣ�void motor_stalling_protect(void)
���ܣ���ת��������
������None
���ߣ�zyf
*******************************************/ 
void motor_stalling_protect(void){
    static u8 stallingprotimes;
    if((MototStatus&0x10)==MOTOR_Status_Stall){
        stallingprotimes++;
        if(stallingprotimes>=1){
            stallingprotimes=0;
            keytemp=2;
            motor_stop();     
       } 

    }
    else stallingprotimes=0;

}

/*******************************************
���ƣ�void HALLerror_protect(void)
���ܣ�hall���󱣻�����
������None
���ߣ�zyf
*******************************************/ 
void HALLerror_protect(void){
    
    static u8 HALL_error_protimes;
    if((MototStatus&0x80)==MOTOR_Status_SENSORFault){
        HALL_error_protimes++;
        if(HALL_error_protimes>=100){
            HALL_error_protimes=0;
            keytemp=2;
            motor_stop();     
       } 

    }
    else HALL_error_protimes=0;

}

/*******************************************
���ƣ�void overtemperature_protect(void)
���ܣ����±�������
������None
���ߣ�zyf
*******************************************/ 
void overtemperature_protect(void){
    static u8 temperatureprotimes;
        if((MototStatus&0x20)==MOTOR_Status_MotOT){
        temperatureprotimes++;
        if(temperatureprotimes>100){
            temperatureprotimes=0;
            keytemp=2;
            motor_stop();     
       } 

    }
    else temperatureprotimes=0;
}



/*******************************************
���ƣ�void current_loop(void)
���ܣ������������޵�����ֵ
������None
���ߣ�zyf

*******************************************/ 
void current_loop(void){
     
    if(keytemp==1){
        if(UserCurrent10Times>350){
            
//          sys_flags.currentloop_protect_flag =1;
            My_PWM =My_PWM-10;

     
        }
        else{
//         sys_flags.currentloop_protect_flag =0;            
        }   
  }   
}
/*******************************************
���ƣ�void power_loop(void)
���ܣ����ʻ������޹���
������None
���ߣ�zyf

*******************************************/ 
void power_loop(void){
      if(keytemp==1){
        if(UserPower>uiLimitPower){
            
//          sys_flags.currentloop_protect_flag =1;
            My_PWM =My_PWM-10;

//     
        }
        else{
//         sys_flags.currentloop_protect_flag =0;            
        }   
  }   
}
/*************************
���ƣ�void motor_stop(void)
���ܣ����ֹͣ�ĺ���
������None
���ߣ�zyf

************************/
void motor_stop(void){
   /*****�����ź�����******/   
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
    /*****ռ�ձ�Ϊ��******/ 
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    SpeedParaInit();
	SpeedSumInit();
//	PowerParaInit();
//	CurrentParaInit();
    /*****��������ռ�ձ�******/
    //sysvariable.pwm = MIN_DUTY; //��������ռ�ձ�
    /*****ϵͳ״̬Ϊ����͵��״̬��������******/
//    sysvariable.run_status = Standby;
//    sys_flags.voltage_protect_flag = 0;
//    sys_flags.current_protect_flag = 0;     
}
