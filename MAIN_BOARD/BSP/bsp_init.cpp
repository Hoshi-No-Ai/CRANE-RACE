#include "bsp_init.h"

void bsp_init(void)
{
	SystemInit();
	RCC_Configuration();//��ʼ��ϵͳʱ��
	ARM_LED_Configuration();//��ʼ��ARM����ָʾ��
	FSMC_Configuration();//��ʼ��FSMC����
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

//	 while(imu_init())            //�ȴ���ʼ�����
//    {
//        delay_ms(1);
//    }
//	WriteReg(FILT_CTRL, 0x06); 

	//easyflash_init();
}
