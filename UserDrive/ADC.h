#ifndef __ADC_H
#define __ADC_H
#include "stm32f10x.h"
void ADC_Configuration1(void);
void DMA_Configuration1(void);
void ADC_DMA_Configuration(void);
/*****结构体的定义**AD采样电压电流三个参数******/ 
typedef struct{
    
//    uint16_t current;
    uint16_t votage;
    uint16_t temp;
    
}adc_sample;
extern  adc_sample  ADC_Data;

int16_t Caculate_Tem(u16 temp_val);
int16_t UtilBiSearchInt(const int sortedIntArr[], int find, uint32_t maxN);
u16 Caculate_CurrentZero(char times);
u16 Caculate_Current(u16 ADC_Value);
u16 Caculate_Vbus10Times(u16 ADC_Value);
#endif
