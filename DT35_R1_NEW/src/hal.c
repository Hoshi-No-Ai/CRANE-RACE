/***************************************************
**HAL.c
**主要用于芯片硬件的内部外围和外部外围的初始化，两大INIT函数
**在MAIN中调用，使MAIN函数中尽量与硬件库无关
***************************************************/
#include "STM32Lib\\stm32f10x.h"
#include "main.h"



void  ChipHalInit(void)
{
	//初始化时钟源
	RCC_Configuration();
	
	//初始化GPIO
	GPIO_Configuration();
	
	//初始化中断源
	NVIC_Configuration();
	
	//初始化定时器
	Tim2_Configuration();
	Tim3_Configuration();
	Tim4_Configuration();
	
	//初始化CAN
	CAN_Configuration();
	CAN_Interrupt();
	
	GPIO_JTAG_Configuration();
	
	//初始化DT35
//	ADS1256_GPIO_init();                    //初始化ADS1256 GPIO管脚
	SPI2_Init();
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);	//设置为9M时钟,高速模式
	Delay(0x1ffFF);
	GPIO_SetBits(GPIO_RCC_ADS1256Reset_PORT, GPIO_RCC_ADS1256Reset);  
//	ADS1256_Init();
	
//	ValveAllStart();
	SysTickDelayms(1000);
	ValveAllClose();
}

/*********************************
**函数名:ChipOutHalInit()
**功能:片外硬件初始化
*********************************/
void  ChipOutHalInit(void)
{
	
}
