#include "Protect.h"

/*******************************************
名称：void voltage_protect(void)
功能：电压保护函数
参数：None
作者：zyf

*******************************************/ 
void voltage_protect(void){
     static u8 voltageprotimes;
   /*****过压电压或者欠压要一定时间，就进行电压保护******/
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
名称：void current_protect(void)
功能：电流保护函数
参数：None
作者：zyf
*******************************************/ 
void current_protect(void){
    static u8 currentprotimes;
    /*****电流或者转速小于10，需要一定时间，就进行电流保护******/
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
名称：void motor_stalling_protect(void)
功能：堵转保护函数
参数：None
作者：zyf
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
名称：void HALLerror_protect(void)
功能：hall错误保护函数
参数：None
作者：zyf
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
名称：void overtemperature_protect(void)
功能：过温保护函数
参数：None
作者：zyf
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
名称：void current_loop(void)
功能：电流环函数限电流幅值
参数：None
作者：zyf

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
名称：void power_loop(void)
功能：功率环函数限功率
参数：None
作者：zyf

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
名称：void motor_stop(void)
功能：电机停止的函数
参数：None
作者：zyf

************************/
void motor_stop(void){
   /*****关下桥和上桥******/   
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);
    /*****占空比为零******/ 
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    SpeedParaInit();
	SpeedSumInit();
//	PowerParaInit();
//	CurrentParaInit();
    /*****设置启动占空比******/
    //sysvariable.pwm = MIN_DUTY; //设置启动占空比
    /*****系统状态为清零和电机状态处于闲置******/
//    sysvariable.run_status = Standby;
//    sys_flags.voltage_protect_flag = 0;
//    sys_flags.current_protect_flag = 0;     
}
