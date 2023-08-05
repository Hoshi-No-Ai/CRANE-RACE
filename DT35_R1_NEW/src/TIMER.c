#include "STM32Lib\\stm32f10x.h"
#include "main.h"

void Tim2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;

	GPIO_InitTypeDef         GPIO_InitStructure;

	/* PA0,1,2,3设置为功能脚(PWM) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //推挽复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	
	TIM_DeInit(TIM2);

	/*TIM2时钟配置*/
	TIM_TimeBaseStructure.TIM_Prescaler = 719;					//预分频(时钟分频)72M/720=100K
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseStructure.TIM_Period = 1999 ;						//装载值 100k/300=333hz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	/* Channel 1，2，3，4  Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//正向通道有效
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;//反向通道无效
	TIM_OCInitStructure.TIM_Pulse = 150; 							//占空时间
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//输出极性
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;     //互补端的极性  
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;		//选择空闲状态下得非工作状态
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	//选择互补空闲状态下得非工作状态


	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC1Init(TIM2,&TIM_OCInitStructure); 						//通道1

	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC2Init(TIM2,&TIM_OCInitStructure); 						//通道2

	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC3Init(TIM2,&TIM_OCInitStructure); 						//通道3

	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC4Init(TIM2,&TIM_OCInitStructure); 						//通道4

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

	/* PA0,1,2,3设置为功能脚(PWM) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //推挽复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		  //推挽复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, DISABLE);

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	
	TIM_DeInit(TIM3);

	/*TIM3时钟配置*/
	TIM_TimeBaseStructure.TIM_Prescaler = 71;					//预分频(时钟分频)72M/720=100K
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseStructure.TIM_Period = 19999;						//装载值 100k/2000=50hz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

	/* Channel 1，2，3，4  Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 				//PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//正向通道有效
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;//反向通道无效
	TIM_OCInitStructure.TIM_Pulse = 150; 							//占空时间
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 		//输出极性
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;     //互补端的极性  
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;		//选择空闲状态下得非工作状态
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	//选择互补空闲状态下得非工作状态


	TIM_OCInitStructure.TIM_Pulse = 150;
	TIM_OC1Init(TIM3,&TIM_OCInitStructure); 						//通道1

	TIM_OCInitStructure.TIM_Pulse = 90;
	TIM_OC2Init(TIM3,&TIM_OCInitStructure); 						//通道2

	TIM_OCInitStructure.TIM_Pulse = 110;
	TIM_OC3Init(TIM3,&TIM_OCInitStructure); 						//通道3

	TIM_OCInitStructure.TIM_Pulse = 150  ;
	TIM_OC4Init(TIM3,&TIM_OCInitStructure); 						//通道4

	/* TIM3 counter enable */
	TIM_Cmd(TIM3,ENABLE);
	
	/* TIM3 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
}

void Tim4_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	TIM_DeInit(TIM4);

	TIM_TimeBaseStructure.TIM_Period = 10; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =7199; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM4, ENABLE);  //使能TIMx外设
							 
}

//设置捕获寄存器，改变占空比。舵机的ServoPulse范围在50-250。
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
