#include "rcc.h"

/*初始化系统时钟及使能外设时钟*/
void RCC_Configuration(void)
{
	//SYSTICK分频--1ms的系统时钟中断
	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
    /* Capture error */ 
		while (1);
	}
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);

	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
		
}
