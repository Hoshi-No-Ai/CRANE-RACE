#include "filter_algorithm.h"

C_LPF lpf_PID(250, 0.001);

//（1-fir_a）为滤波系数
//滤波系数越小，滤波结果越平稳，但是灵敏度越低；
//滤波系数越大，灵敏度越高，但是滤波结果越不稳定
void C_LPF::LpFilter(void) {
    fp32 fir_a = 1 / (1 + m_off_freq * m_samp_tim);
    m_out = fir_a * m_preout + (1 - fir_a) * m_in;
    m_preout = m_out;
}
