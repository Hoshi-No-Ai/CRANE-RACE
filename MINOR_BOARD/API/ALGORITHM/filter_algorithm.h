#ifndef __FILTER_ALGORITHM_H__
#define __FILTER_ALGORITHM_H__

#include "basic_type.h"

typedef struct
{
	fp32 preout;
	fp32 out;
	fp32 in;
	fp32 off_freq;//权重
	fp32 samp_tim;//采样步长
}ST_LPF;

extern void LpFilter(ST_LPF* lpf);

class cTD
{
	public:
	  float x1 = 0;
	  float x2 = 0;
	  float x = 0;
	  float r = 1000;
	  float h = 0.001;
	  float T = 0.001;
	  float aim;

	cTD(){}
	cTD(float _r, float _h)
	{
		this->r = _r;
		this->h = _h;
	}

	void TD_Function(void);
};

extern ST_LPF lpf_PID;
#endif
