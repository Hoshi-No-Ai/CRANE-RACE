#include "STM32Lib\\stm32f10x.h"
#include "main.h"


RCC_ClocksTypeDef    RCC_ClockFreq;

void RCC_Configuration(void)
{
	SystemInit();//Դ��system_stm32f10x.c�ļ�,ֻ��Ҫ���ô˺���,������RCC������.�����뿴2_RCC

	/**************************************************
	��ȡRCC����Ϣ,������
	��ο�RCC_ClocksTypeDef�ṹ�������,��ʱ��������ɺ�,
	���������ֵ��ֱ�ӷ�ӳ�������������ֵ�����Ƶ��
	***************************************************/
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	/* ������ÿ�ʹ�ⲿ����ͣ���ʱ��,����һ��NMI�ж�,����Ҫ�õĿ����ε�*/
	//RCC_ClockSecuritySystemCmd(ENABLE);


	//SYSTICK��Ƶ--1ms��ϵͳʱ���ж�
	if (SysTick_Config(SystemFrequency / 1000000))
  	{ 
  	  	/* Capture error */ 
    	while (1);
  	}

}


/********************************************
**������:SysTickDelayms
**����:ʹ��ϵͳʱ�ӵ�Ӳ�ӳ�
**ע������:һ���,��Ҫ���ж��е��ñ�����,����������������.�������������ȫ���ж�,��Ҫʹ�ô˺���
********************************************/

void SysTickDelayms(u16 dly_ms)
{
			ucTimDlyCntms=dly_ms;
			while(ucTimDlyCntms);
}

void SysTickDelayus(u16 dly_us)
{
			ucTimDlyCntus=dly_us;
			while(ucTimDlyCntus);
}
void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}



