/*******************************************************************
版权声明：HITCRT(哈工大竞技机器人队)
文件名：Common_Algorithm.c
最近修改日期：2022.03.28
版本：2022版
作者：李禄平 王一宇
**********************************************************************/

#include "math_algorithm.h"

/*******************************************************************
函数名称：Clip()
函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
输入：    siValue:实际值
		  siMin:下限值
		  siMax:上限值
输出：    siValue：削波后的值
备注：	  适用于整型变量的消波
********************************************************************/
int32_t Clip(int32_t siValue, int32_t siMin, int32_t siMax)
{
    if(siValue < siMin)
    {
        return siMin;
    }
    else if(siValue > siMax)
    {
        return siMax;
    }
    else
    {
        return siValue;
    }
}

/*******************************************************************
函数名称：ClipChar()
函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
输入：    ucValue:实际值
		  ucMin:下限值
		  ucMax:上限值
输出：    ucValue：削波后的值
备注：	  适用于无符号字符型削波
********************************************************************/
uint8_t ClipChar(uint8_t ucValue, uint8_t ucMin, uint8_t ucMax)
{
    if(ucValue < ucMin)
    {
        return ucMin;
    }
    else if(ucValue > ucMax)
    {
        return ucMax;
    }
    else
    {
        return ucValue;
    }
}

/*******************************************************************
函数名称：ClipFloat()
函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
输入：  ucValue:实际值
				ucMin:下限值
				ucMax:上限值
输出：  ucValue：削波后的值
备注：	适用于浮点类型削波
********************************************************************/

fp32 ClipFloat(fp32 fpValue, fp32 fpMin, fp32 fpMax)
{
    if(fpValue < fpMin)
    {
        return fpMin;
    }
    else if(fpValue > fpMax)
    {
        return fpMax;
    }
    else
    {
        return fpValue;
    }
}

float Sign_Judge(float fp_Judge_Number)
{
    return fp_Judge_Number>=0 ? 1:-1;
}

/*******************************************************************
函数名称：float_to_uint()
函数功能：将浮点类型变量转化为整型
输入：	x——int:输入量
				x_min:下限值
				x_max:上限值
				bits： 输入量的位数
输出：  转化成的浮点
备注：  适用于HT原装电机的反馈解析
********************************************************************/
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/*******************************************************************
函数名称：uint_to_float()
函数功能：将整型变量转化为浮点类型
输入：	x——int:输入量
				x_min:下限值
				x_max:上限值
				bits： 输入量的位数
输出：  转化成的浮点
备注：  适用于HT原装电机的反馈解析
********************************************************************/
  float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
