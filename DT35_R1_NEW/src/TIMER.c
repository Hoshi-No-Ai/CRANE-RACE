#include "STM32Lib\\stm32f10x.h"
#include "main.h"

void Tim2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;

	GPIO_InitTypeDef         GPIO_InitStructure;

	/* PA0,1,2,3����Ϊ���ܽ�(PWM) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //���츴�ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	
	TIM_DeInit(TIM2);

	/*TIM2ʱ������*/
	TIM_TimeBaseStructure.TIM_Prescaler = 719;					//Ԥ��Ƶ(ʱ�ӷ�Ƶ)72M/720=100K
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���
	TIM_TimeBaseStructure.TIM_Period = 1999 ;						//װ��ֵ 100k/300=333hz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	/* Channel 1��2��3��4  Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//����ͨ����Ч
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;//����ͨ����Ч
	TIM_OCInitStructure.TIM_Pulse = 150; 							//ռ��ʱ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//�������
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;     //�����˵ļ���  
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;		//ѡ�����״̬�µ÷ǹ���״̬
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	//ѡ�񻥲�����״̬�µ÷ǹ���״̬


	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC1Init(TIM2,&TIM_OCInitStructure); 						//ͨ��1

	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC2Init(TIM2,&TIM_OCInitStructure); 						//ͨ��2

	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC3Init(TIM2,&TIM_OCInitStructure); 						//ͨ��3

	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC4Init(TIM2,&TIM_OCInitStructure); 						//ͨ��4

	/* TIM2 counter enable */
	TIM_Cmd(TIM2,ENABLE);
	
	/* TIM2 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM2,ENABLE);
}
void Tim3_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;

		GPIO_InitTypeDef         GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);
	

//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
//	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

	/* PA0,1,2,3����Ϊ���ܽ�(PWM) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //���츴�ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //���츴�ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, DISABLE);

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	
	TIM_DeInit(TIM3);

	/*TIM3ʱ������*/
	TIM_TimeBaseStructure.TIM_Prescaler = 71;					//Ԥ��Ƶ(ʱ�ӷ�Ƶ)72M/720=100K
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���
	TIM_TimeBaseStructure.TIM_Period = 19999;						//װ��ֵ 100k/2000=50hz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

	/* Channel 1��2��3��4  Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//PWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//����ͨ����Ч
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;//����ͨ����Ч
	TIM_OCInitStructure.TIM_Pulse = 150; 							//ռ��ʱ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//�������
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;     //�����˵ļ���  
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;		//ѡ�����״̬�µ÷ǹ���״̬
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	//ѡ�񻥲�����״̬�µ÷ǹ���״̬


	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC1Init(TIM3,&TIM_OCInitStructure); 						//ͨ��1

	TIM_OCInitStructure.TIM_Pulse = 90;
	TIM_OC2Init(TIM3,&TIM_OCInitStructure); 						//ͨ��2

	TIM_OCInitStructure.TIM_Pulse = 110;
	TIM_OC3Init(TIM3,&TIM_OCInitStructure); 						//ͨ��3

	TIM_OCInitStructure.TIM_Pulse = 150  ;
	TIM_OC4Init(TIM3,&TIM_OCInitStructure); 						//ͨ��4

	/* TIM3 counter enable */
	TIM_Cmd(TIM3,ENABLE);
	
	/* TIM3 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
}

void Tim4_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��
	
	TIM_DeInit(TIM4);

	TIM_TimeBaseStructure.TIM_Period = 10; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =7199; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIMx����
							 
}

//���ò���Ĵ������ı�ռ�ձȡ������ServoPulse��Χ��50-250��
void ServoCtrl(unsigned char Num,unsigned int Pulse)
{	
	u8 HighPulseTemp;
	u8 LowPulseTemp;
	if(Num <= 3)
	{
		if( Pulse>250 )
		{
			LowPulseTemp = 250 ; 
		}
		else if( Pulse<50 )
		{
			LowPulseTemp = 50 ;
		}
		else
		{
			LowPulseTemp = Pulse;
		}
	}
		
	else if((Num >= 4) && (Num <= 7))
	{
		if( Pulse>250 )
		{
			HighPulseTemp = 250 ; 
		}
		else if( Pulse<50 )
		{
			HighPulseTemp = 50 ;
		}
		else
		{
			HighPulseTemp = Pulse;
		}
	}
	
	switch(Num)
	{
		case 0:
			TIM2->CCR1 = LowPulseTemp;
		break;
		case 1:
			TIM2->CCR2 = LowPulseTemp;
		break;
		case 2:
			TIM2->CCR3 = LowPulseTemp;
		break;
		case 3:
			TIM2->CCR4 = LowPulseTemp;
		break;
		case 4:
			TIM3->CCR1 = HighPulseTemp;
		break;
		case 5:
			TIM3->CCR2 = HighPulseTemp;
		break;
		case 6:
			TIM3->CCR3 = HighPulseTemp;
		break;
		case 7:
			TIM3->CCR4 = HighPulseTemp;
		break;
		default:
		break;
	}
}
