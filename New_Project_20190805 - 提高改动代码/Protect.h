#ifndef _PROTECT_H__
#define _PROTECT_H__

#include "globalvariable.h"

void voltage_protect(void);
void current_protect(void);
void motor_stalling_protect(void);
void HALLerror_protect(void);
void overtemperature_protect(void);
    
void current_loop(void);
void power_loop(void);

void motor_stop(void);

#endif

