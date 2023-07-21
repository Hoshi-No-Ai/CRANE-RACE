#ifndef __LED_H__
#define __LED_H__

#include "stm32f4xx.h"

//ARM_LEDÉÁË¸
#define ARM_LED_FLASH() GPIOF->ODR ^= GPIO_Pin_7

void ARM_LED_Configuration(void);

#endif
