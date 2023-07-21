#ifndef __SMC_ALGORITHM_H__
#define __SMC_ALGORITHM_H__

#include "stm32f4xx.h"
#include "math_algorithm.h"
#include "basic_type.h"
#include "math.h"
#include "filter_algorithm.h"
#include "math_algorithm.h"




class cSMC
{
	public:
	float aim;//Ŀ��ֵ
	float FB;//��ǰֵ
	float x1;//Ŀ��ֵ��ȥ��ǰֵ
	float x2;//x1��΢��  �ٶ�
	float x3;//x2��΢��  ���ٶ�
	float prex1;//��һ�ε�x1
  float prex2;//��һ�ε�x2
	/*��ģ��ز���*/
	float epsilon;
	float s;
	float q;
	float c;
	

	float fpUi;//�Ե������ֵ���л���
	float realfpU;//�������������ֵ
	float fpUMax;
	float fpUMin;
	float real_current;//�������
	
	
	cTD td_smc;
	ST_LPF lpf_smc;
	
	cSMC(){}
	cSMC(
		float EP,float Q, float C, //��Ĥ��ز���
		float UMax,float UMin,//�޷�����
		float off,float tim
	
			)
		{ 
			this ->aim = 0;
			this ->FB = 0;
			this ->x1 = 0;
			this ->x2 = 0;
			this ->x3 = 0;
			this ->prex1 = 0;
			this ->prex2 = 0;
			this ->s = 0;
			this ->fpUi = 0;
			this ->realfpU = 0;
			this ->real_current = 0; 
			this ->lpf_smc.preout = 0;
      this ->lpf_smc.out = 0;
      this ->lpf_smc.in = 0;			
			this->td_smc.aim =0;
			
			this ->epsilon = EP;
			this ->fpUMax = UMax;
			this ->fpUMin = UMin;
			this ->q = Q;
			this ->c = C;
			this ->lpf_smc.off_freq = off;
			this ->lpf_smc.samp_tim = tim;
		}
	
	void CalSMC(void);
	float SGN(float a);
};


#endif
