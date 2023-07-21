#ifndef __FILTER_ALGORITHM_H__
#define __FILTER_ALGORITHM_H__

#include "basic_type.h"

//可以用RC低通滤波电路推导原理
//推导出传递函数1/(1+RCS),RC=时间常数，故1/RC为截止频率
//再对传递函数进行z变换，即可推导出离散的一阶滤波公式
//其中m_off_freq为截止频率，m_samp_tim为采样周期
class C_LPF {
   public:
    fp32 m_preout;
    fp32 m_out;
    fp32 m_in;
    fp32 m_off_freq;  //权重
    fp32 m_samp_tim;  //采样步长

    C_LPF(){};
    ~C_LPF(){};
    C_LPF(fp32 off_freq, fp32 samp_tim) : m_off_freq(off_freq), m_samp_tim(samp_tim){};
    void LpFilter(void);
};

extern C_LPF lpf_PID;

#endif
