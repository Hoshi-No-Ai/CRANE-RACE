#ifndef __TIM_H__
#define __TIM_H__

/*TIM2��һλ����ʱ��*/  //1us

extern void TIM2_Configuration(u32 arr,u16 psc);
extern void TIM3_Int_Init(u16 arr,u16 psc);
extern void TIM5_Int_Init(u16 arr,u16 psc);

#endif
