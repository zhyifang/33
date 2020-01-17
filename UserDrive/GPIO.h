#ifndef __GPIO_H
#define __GPIO_H
#include "stm32f10x.h"

//Control pin
#define DRV8301_EN_PIN											GPIO_Pin_3              
#define DRV8301_EN_GPIO_PORT									GPIOB
#define DRV8301_EN_GPIO_CLK										RCC_APB2Periph_GPIOB

#define DRV8301_FAULT_PIN										GPIO_Pin_12         
#define DRV8301_FAULT_GPIO_PORT									GPIOB
#define DRV8301_FAULT_GPIO_CLK									RCC_APB2Periph_GPIOB

#define DRV8301_OC_PIN											GPIO_Pin_11              
#define DRV8301_OC_GPIO_PORT									GPIOB
#define DRV8301_OC_GPIO_CLK										RCC_APB2Periph_GPIOB

#define DRV8301_POWERGD_PIN										GPIO_Pin_10             
#define DRV8301_POWERGD_GPIO_PORT								GPIOB
#define DRV8301_POWERGD_GPIO_CLK								RCC_APB2Periph_GPIOB

#define MDBT40_CS_PORT											GPIOA
#define MDBT40_CS_PIN											GPIO_Pin_4 
#define MDBT40_CS_GPIO_CLK										RCC_APB2Periph_GPIOA

#define DRV8301_CS_PORT											GPIOA
#define	DRV8301_CS_PIN											GPIO_Pin_11
#define	DRV8301_CS_GPIO_CLK										RCC_APB2Periph_GPIOA

#define DRV8301_FAULT_STATUS									GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)//¶ÁÈ¡FAULT×´Ì¬
#define DRV8301_OC_STATUS										GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)//¶ÁÈ¡OC×´Ì¬

void MY_GPIO_Init(void);
#endif

