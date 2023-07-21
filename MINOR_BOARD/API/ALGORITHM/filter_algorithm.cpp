#include "filter_algorithm.h"
#include "math_algorithm.h"
#include "math.h"

ST_LPF lpf_PID = {0, 0, 0, 250, 0.001};

/*******************************************************************
函数名称：LpFilter(ST_LPF* lpf)
函数功能：滤波函数，可滤去变化太大的量，低通滤波器
输入：    ST_LPF* lpf
          滤波函数的输入结构体
输出：    lpf->out：滤波后的值
备注：	  适用于PID计算输出中的Kd项
********************************************************************/
void LpFilter(ST_LPF* lpf)
{                                                                   
	fp32 fir_a = 1/(1 + lpf->off_freq * lpf->samp_tim); 
  lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;
	lpf->preout = lpf->out;	
}

/*******************************************************************
函数名称：TD_Function()
函数功能：微分跟踪器，可将阶跃信号转化为渐变的较平滑的信号
输入：    阶跃信号量
输出：    x1，x2
备注：	  适用于减小电机超调
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
