#include "tim.h"

// 1us进入一次中断
void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF; // 自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler = 83;	   // 预分频系数
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIM enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void TIM3_PWM_Configuration(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruc;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能AHB1时钟

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStruc.TIM_Period = arr;						// 设置自动重装载寄存器周期值
	TIM_TimeBaseStruc.TIM_Prescaler = psc;					// 设置时钟频率除数的预分频值
	TIM_TimeBaseStruc.TIM_ClockDivision = TIM_CKD_DIV1;		// 设置时钟分割
	TIM_TimeBaseStruc.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruc);				// 初始化TIM3

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;			  // 选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	  // 输出极性:TIM输出比较极性低

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	// 使能TIM3在CCR1上的预装载寄存器
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE); // ARPE使能

	TIM_Cmd(TIM3, ENABLE);
}

void TIM5_PWM_Configuration(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruc;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能AHB1时钟

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3; // GPIOC2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		   // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	   // 速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		   // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		   // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	TIM_TimeBaseStruc.TIM_Period = arr;						// 设置自动重装载寄存器周期值
	TIM_TimeBaseStruc.TIM_Prescaler = psc;					// 设置时钟频率除数的预分频值
	TIM_TimeBaseStruc.TIM_ClockDivision = TIM_CKD_DIV1;		// 设置时钟分割
	TIM_TimeBaseStruc.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruc);				// 初始化TIM5

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;			  // 选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	  // 输出极性:TIM输出比较极性低

	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);

	// 使能TIM5在CCR1上的预装载寄存器
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM5, ENABLE); // ARPE使能

	TIM_Cmd(TIM5, ENABLE);
}

void TIM7_Configuration(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruc;
	NVIC_InitTypeDef NVIC_InitStruc;

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // 使能PORTF时钟

	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOB);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		   // GPIOC2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	   // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE); // 使能PORTF时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; // GPIOC2
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; // GPIOC2
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseStruc.TIM_Period = arr;						// 设置自动重装载寄存器周期值
	TIM_TimeBaseStruc.TIM_Prescaler = psc;					// 设置时钟频率除数的预分频值
	TIM_TimeBaseStruc.TIM_ClockDivision = TIM_CKD_DIV1;		// 设置时钟分割
	TIM_TimeBaseStruc.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStruc);				// 初始化TIM2

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	// 设置中断优先级分组
	NVIC_InitStruc.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStruc.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruc.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruc.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStruc);

	TIM_Cmd(TIM7, ENABLE);
}
