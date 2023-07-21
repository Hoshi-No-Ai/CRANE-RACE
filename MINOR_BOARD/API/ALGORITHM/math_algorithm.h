/*---------------------------------------------------------------------
版权声明：HITCRT(哈工大竞技机器人队)
文件名：HITCRT_Coff.h
最近修改日期：2022.03.28
版本：1.0
---------------------------------------------------------------------*/
#ifndef __MATH_ALGORITHM_H__
#define __MATH_ALGORITHM_H__

#include "stm32f4xx.h"
#include "basic_type.h"
#include "math.h"

#define PI 3.1415926535
typedef struct
{
	fp32 x;
	fp32 y;
}stPoint;


extern int32_t Clip(int32_t siValue, int32_t siMin, int32_t siMax);
extern uint8_t ClipChar(uint8_t ucValue, uint8_t ucMin, uint8_t ucMax);
extern uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern fp32 ClipFloat(fp32 fpValue, fp32 fpMin, fp32 fpMax);
extern float Sign_Judge(float fp_Judge_Number);

#endif
