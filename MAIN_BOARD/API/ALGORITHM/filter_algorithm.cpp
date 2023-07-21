#include "filter_algorithm.h"

C_LPF lpf_PID(250, 0.001);

//��1-fir_a��Ϊ�˲�ϵ��
//�˲�ϵ��ԽС���˲����Խƽ�ȣ�����������Խ�ͣ�
//�˲�ϵ��Խ��������Խ�ߣ������˲����Խ���ȶ�
void C_LPF::LpFilter(void) {
    fp32 fir_a = 1 / (1 + m_off_freq * m_samp_tim);
    m_out = fir_a * m_preout + (1 - fir_a) * m_in;
    m_preout = m_out;
}
