#ifndef _SYSTEM_H__
#define _SYSTEM_H__

#include "globalvariable.h"

void initsysblock(void);
void sys_variable_init(void);

void sys_enable(void);
void sys_disable(void);
void sys_flag_init(void);

void Screw_Calibration(void);
void wheel_Calibration(void);


#endif

