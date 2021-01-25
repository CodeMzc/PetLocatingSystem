#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"
#include "Led.h"

void Timer2_Init_Config(void);
void TIM7_Int_Init(u16 arr,u16 psc);
void TIM6_Int_Init(u16 arr,u16 psc);
#endif
