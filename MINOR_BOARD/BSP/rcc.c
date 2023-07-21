#include "rcc.h"

/*��ʼ��ϵͳʱ�Ӽ�ʹ������ʱ��*/
void RCC_Configuration(void)
{
	//SYSTICK��Ƶ--1ms��ϵͳʱ���ж�
	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
    /* Capture error */ 
		while (1);
	}
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);

	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
		
}
