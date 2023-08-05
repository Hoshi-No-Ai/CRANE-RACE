#ifndef RCC_H
#define RCC_H


//系统时钟延迟

void RCC_Configuration(void);
void SysTickDelayms(u16 dly_ms);
void SysTickDelayus(u16 dly_us);
void Delay(vu32 nCount);            //DT35专用延时函数

#endif



