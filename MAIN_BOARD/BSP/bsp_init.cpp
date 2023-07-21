#include "bsp_init.h"

void bsp_init(void)
{
	SystemInit();
	RCC_Configuration();//初始化系统时钟
	ARM_LED_Configuration();//初始化ARM运行指示灯
	FSMC_Configuration();//初始化FSMC总线
	CAN_Configuration();
	NVIC_Configuration();
BUZZER_Init();
	TIM3_PWM_Configuration(19999,83);
	TIM5_PWM_Configuration(19999,83);

	TIM2_Configuration();
//	TIM7_Configuration(9, 167);
//	TIM2_Init();
//	Tim2IOInit();
//	TIM4_Init();
//	Tim4IOInit();

//	TIM2_Configuration(999,83);
//	SPI2_Configuration();

	USART1_Configuration();
	USART2_Configuration();
//	UART4_Configuration();
	USART3_Configuration();
	USART6_Configuration();
//	NRF24L01_Init();
//	NRF24L01_RX_Mode();
//	while(NRF24L01_Check())
//	{
//		
//	}

//	 while(imu_init())            //等待初始化完毕
//    {
//        delay_ms(1);
//    }
//	WriteReg(FILT_CTRL, 0x06); 

	//easyflash_init();
}
