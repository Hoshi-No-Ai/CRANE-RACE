#ifndef __FILTER_ALGORITHM_H__
#define __FILTER_ALGORITHM_H__

#include "basic_type.h"

//������RC��ͨ�˲���·�Ƶ�ԭ��
//�Ƶ������ݺ���1/(1+RCS),RC=ʱ�䳣������1/RCΪ��ֹƵ��
//�ٶԴ��ݺ�������z�任�������Ƶ�����ɢ��һ���˲���ʽ
//����m_off_freqΪ��ֹƵ�ʣ�m_samp_timΪ��������
class C_LPF {
   public:
    fp32 m_preout;
    fp32 m_out;
    fp32 m_in;
    fp32 m_off_freq;  //Ȩ��
    fp32 m_samp_tim;  //��������

    C_LPF(){};
    ~C_LPF(){};
    C_LPF(fp32 off_freq, fp32 samp_tim) : m_off_freq(off_freq), m_samp_tim(samp_tim){};
    void LpFilter(void);
};

extern C_LPF lpf_PID;

#endif
