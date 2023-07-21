#ifndef __BASIC_FUNTION__
#define __BASIC_FUNTION__

#include "basic_type.h"
#include "stm32f4xx.h"
#include "stdint.h"

union float2char
{
	uint8_t char_num[4];
	fp32 float_num;	
};

union s16tochar
{
	uint8_t char_num[2];
	s16 s16_num;	
};

extern void delay_ms(int t);
extern void delay_us(u32 t);

#endif