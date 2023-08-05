#ifndef _TIMER_H
#define _TIMER_H

void Tim2_Configuration(void);			//TIM2初始化
void Tim3_Configuration(void);
void Tim4_Configuration(void);
void ServoCtrl(unsigned char ServoNum,unsigned int ServoPulse);	//设置捕获寄存器，改变占空比

#endif

