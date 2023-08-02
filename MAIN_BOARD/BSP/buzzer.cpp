#include "buzzer.h"

void BUZZER_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); // 使能GPIOG时钟

  // 初始化蜂鸣器对应引脚GPIOG7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;     // 下拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);             // 初始化GPIO

  GPIO_ResetBits(GPIOG, GPIO_Pin_7); // 蜂鸣器对应引脚GPIOG7拉低，

  GPIO_InitTypeDef GPIO_InitStructure1;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); // 使能GPIOG时钟

  // 初始化蜂鸣器对应引脚GPIOG7
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;     // 下拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);             // 初始化GPIO

  GPIO_ResetBits(GPIOF, GPIO_Pin_8); // 蜂鸣器对应引脚GPIOG7拉低，
}
