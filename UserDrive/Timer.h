#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"

/****	 Pattern type is center aligned  ****/
#define CKTIM	((u32)64000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */
#define PWM_PRSC ((u8)0)
/****	Power devices switching frequency  ****/
#define PWM_FREQ ((u16) 32000) // in Hz  (N.b.: pattern type is center aligned)
/* Resolution: 1Hz */                            
#define PWM_PERIOD ((u16) (CKTIM / (u32)(PWM_FREQ *(PWM_PRSC+1)))) //2000
	
/****	ADC IRQ-HANDLER frequency, related to PWM  ****/
#define REP_RATE (0)  	// (N.b): Internal current loop is performed every

/****    Deadtime Value   ****/
#define DEADTIME_NS	((u16) 800)  //in nsec; range is [0...3500]
////////////////////////////// Deadtime Value /////////////////////////////////
#define DEADTIME  (u16)((unsigned long long)CKTIM/2*(unsigned long long)DEADTIME_NS/1000000000uL) 

void RCC_Configuration(void);
void TIM1_Configuration1(void);
void TIM2_Int_Init(u16 arr,u16 psc);
#endif


