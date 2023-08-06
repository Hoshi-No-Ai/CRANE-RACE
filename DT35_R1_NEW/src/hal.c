/***************************************************
**HAL.c
**��Ҫ����оƬӲ�����ڲ���Χ���ⲿ��Χ�ĳ�ʼ��������INIT����
**��MAIN�е��ã�ʹMAIN�����о�����Ӳ�����޹�
***************************************************/
#include "STM32Lib\\stm32f10x.h"
#include "main.h"



void  ChipHalInit(void)
{
	//��ʼ��ʱ��Դ
	RCC_Configuration();
	
	//��ʼ��GPIO
	GPIO_Configuration();
	
	//��ʼ���ж�Դ
	NVIC_Configuration();
	
	//��ʼ����ʱ��
	Tim2_Configuration();
	Tim3_Configuration();
	Tim4_Configuration();
	
	//��ʼ��CAN
	CAN_Configuration();
	CAN_Interrupt();
	
	GPIO_JTAG_Configuration();
	
	//��ʼ��DT35
//	ADS1256_GPIO_init();                    //��ʼ��ADS1256 GPIO�ܽ�
	SPI2_Init();
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);	//����Ϊ9Mʱ��,����ģʽ
	Delay(0x1ffFF);
	GPIO_SetBits(GPIO_RCC_ADS1256Reset_PORT, GPIO_RCC_ADS1256Reset);  
//	ADS1256_Init();
	
//	ValveAllStart();
	SysTickDelayms(1000);
	ValveAllClose();
}

/*********************************
**������:ChipOutHalInit()
**����:Ƭ��Ӳ����ʼ��
*********************************/
void  ChipOutHalInit(void)
{
	
}
