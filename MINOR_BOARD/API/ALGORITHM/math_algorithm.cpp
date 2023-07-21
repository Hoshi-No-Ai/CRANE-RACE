/*******************************************************************
��Ȩ������HITCRT(�����󾺼������˶�)
�ļ�����Common_Algorithm.c
����޸����ڣ�2022.03.28
�汾��2022��
���ߣ���»ƽ ��һ��
**********************************************************************/

#include "math_algorithm.h"

/*******************************************************************
�������ƣ�Clip()
�������ܣ�����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
���룺    siValue:ʵ��ֵ
		  siMin:����ֵ
		  siMax:����ֵ
�����    siValue���������ֵ
��ע��	  ���������ͱ���������
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
�������ƣ�ClipChar()
�������ܣ�����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
���룺    ucValue:ʵ��ֵ
		  ucMin:����ֵ
		  ucMax:����ֵ
�����    ucValue���������ֵ
��ע��	  �������޷����ַ�������
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
�������ƣ�ClipFloat()
�������ܣ�����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
���룺  ucValue:ʵ��ֵ
				ucMin:����ֵ
				ucMax:����ֵ
�����  ucValue���������ֵ
��ע��	�����ڸ�����������
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
�������ƣ�float_to_uint()
�������ܣ����������ͱ���ת��Ϊ����
���룺	x����int:������
				x_min:����ֵ
				x_max:����ֵ
				bits�� ��������λ��
�����  ת���ɵĸ���
��ע��  ������HTԭװ����ķ�������
********************************************************************/
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/*******************************************************************
�������ƣ�uint_to_float()
�������ܣ������ͱ���ת��Ϊ��������
���룺	x����int:������
				x_min:����ֵ
				x_max:����ֵ
				bits�� ��������λ��
�����  ת���ɵĸ���
��ע��  ������HTԭװ����ķ�������
********************************************************************/
  float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
