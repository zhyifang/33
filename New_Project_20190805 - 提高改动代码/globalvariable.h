#ifndef _GLOBALVARIABLE_H__
#define _GLOBALVARIABLE_H__

#include "stm32f10x.h"
#include "Drv8301.h"
#include "Delay.h"
#include "GPIO.h"
#include "Timer.h"
#include "IWDG.h"
#include "ADC.h"
#include "HALL.h"
#include "MotorControl.h"
#include "SPI.h"
#include "USART.h"
#include "flash.h"
#include "5182.h"
//
#include "Protect.h"
#include "System.h"




typedef struct{
    unsigned voltage_protect_flag :1;    //过压标志位
    unsigned current_protect_flag :1;    //过流标志位
    unsigned currentloop_protect_flag :1;//电流环标志位 
    
    unsigned screw_Calibration_Flag:1;         //丝杆校准标志位
    unsigned screw_CalibrationSuccess_Flag:1;  //丝杆校准成功标志位
    unsigned screw_Calibrationleft_Flag:1;     //丝杆左端校准成功标志位
    unsigned screw_Calibrationrigth_Flag:1;    //丝杆右端校准成功标志位
    
    unsigned wheel_Calibration_Flag:1;         //方向盘校准标志位
    unsigned wheel_CalibrationSuccess_Flag:1;  //方向盘校准成功标志位 
    
    unsigned ForewardStall_Flag:1;       //前堵转
    unsigned ReversedStall_Flag:1;       //后堵转
}screw_cabration_flag;


typedef struct{
    int32_t pre_error;//
    float kp ;        //
    float ki ;        //
    float kd ;        //
 
}pid;
typedef struct{
    uint16_t votage;              //电压
    uint16_t motor_current;       //电流
    uint16_t desire_speed;        //速度给定
    uint16_t measure_speed;       // 测量转速
    uint16_t motor_now_position;  //电机HALL的位置
    uint16_t motor_old_position;  //前一次的电机HALL位置
    uint16_t pwm;                 //调节PWM输出变量
    
    uint8_t  run_status;        //电机运行标志位
    

}variable;

extern  screw_cabration_flag   sys_flags;



#endif


