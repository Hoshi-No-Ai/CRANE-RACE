#include "stm32f4xx.h"
#ifndef __TIM_H__
#define __TIM_H__

/*TIM2��һλ����ʱ��*/          // 1us
#define TIM2_BASE_TIME 0.000001 //��λs

void TIM2_Configuration(void);
void TIM3_PWM_Configuration(u16 arr, u16 psc);
void TIM5_PWM_Configuration(u16 arr, u16 psc);
void TIM7_Configuration(u16 arr, u16 psc);

#endif
