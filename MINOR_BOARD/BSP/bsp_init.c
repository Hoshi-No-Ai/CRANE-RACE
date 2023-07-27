#include "bsp_init.h"

void bsp_init(void)
{
	SystemInit();
	RCC_Configuration();//初始化系统时钟
	CAN_Configuration();
	NVIC_Configuration();
	TIM2_Configuration(1250-1,8400-1);
	USART1_Configuration();
	USART6_Configuration();
	USART2_Configuration();
	USART3_Configuration();
	gpio_init();
//	Openclose_motor.
	
//	TIM3_Int_Init(125-1,8400-1);//10us
}



