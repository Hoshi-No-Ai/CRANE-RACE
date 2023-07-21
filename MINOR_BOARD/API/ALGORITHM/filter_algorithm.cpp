#include "filter_algorithm.h"
#include "math_algorithm.h"
#include "math.h"

ST_LPF lpf_PID = {0, 0, 0, 250, 0.001};

/*******************************************************************
�������ƣ�LpFilter(ST_LPF* lpf)
�������ܣ��˲�����������ȥ�仯̫���������ͨ�˲���
���룺    ST_LPF* lpf
          �˲�����������ṹ��
�����    lpf->out���˲����ֵ
��ע��	  ������PID��������е�Kd��
********************************************************************/
void LpFilter(ST_LPF* lpf)
{                                                                   
	fp32 fir_a = 1/(1 + lpf->off_freq * lpf->samp_tim); 
  lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;
	lpf->preout = lpf->out;	
}

/*******************************************************************
�������ƣ�TD_Function()
�������ܣ�΢�ָ��������ɽ���Ծ�ź�ת��Ϊ����Ľ�ƽ�����ź�
���룺    ��Ծ�ź���
�����    x1��x2
��ע��	  �����ڼ�С�������
********************************************************************/
void cTD::TD_Function()
{
	float d,d0,y,a0,a=0;
    x = x1 - aim;
    d = r * h;
    d0 = h * d;
    y = x + h * x2;
    a0 = sqrt(d*d + 8*r*fabs(y));

    if(fabs(y)>d0)
        a = x2+(a0-d)*Sign_Judge(y)/2;
    else
        a = x2 + y/h;

    if(fabs(a)>d)
        y=-1*r*Sign_Judge(a);
    else
        y=-1*r*a/d;

    x1 +=  T*x2;
    x2 +=  T*y;
}
