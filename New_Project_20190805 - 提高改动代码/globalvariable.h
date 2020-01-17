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
    unsigned voltage_protect_flag :1;    //��ѹ��־λ
    unsigned current_protect_flag :1;    //������־λ
    unsigned currentloop_protect_flag :1;//��������־λ 
    
    unsigned screw_Calibration_Flag:1;         //˿��У׼��־λ
    unsigned screw_CalibrationSuccess_Flag:1;  //˿��У׼�ɹ���־λ
    unsigned screw_Calibrationleft_Flag:1;     //˿�����У׼�ɹ���־λ
    unsigned screw_Calibrationrigth_Flag:1;    //˿���Ҷ�У׼�ɹ���־λ
    
    unsigned wheel_Calibration_Flag:1;         //������У׼��־λ
    unsigned wheel_CalibrationSuccess_Flag:1;  //������У׼�ɹ���־λ 
    
    unsigned ForewardStall_Flag:1;       //ǰ��ת
    unsigned ReversedStall_Flag:1;       //���ת
}screw_cabration_flag;


typedef struct{
    int32_t pre_error;//
    float kp ;        //
    float ki ;        //
    float kd ;        //
 
}pid;
typedef struct{
    uint16_t votage;              //��ѹ
    uint16_t motor_current;       //����
    uint16_t desire_speed;        //�ٶȸ���
    uint16_t measure_speed;       // ����ת��
    uint16_t motor_now_position;  //���HALL��λ��
    uint16_t motor_old_position;  //ǰһ�εĵ��HALLλ��
    uint16_t pwm;                 //����PWM�������
    
    uint8_t  run_status;        //������б�־λ
    

}variable;

extern  screw_cabration_flag   sys_flags;



#endif


