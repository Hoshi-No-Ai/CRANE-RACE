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
	float aim;//目标值
	float FB;//当前值
	float x1;//目标值减去当前值
	float x2;//x1的微分  速度
	float x3;//x2的微分  加速度
	float prex1;//上一次的x1
  float prex2;//上一次的x2
	/*滑模相关参数*/
	float epsilon;
	float s;
	float q;
	float c;
	

	float fpUi;//对单次输出值进行积分
	float realfpU;//削波函数后输出值
	float fpUMax;
	float fpUMin;
	float real_current;//输出电流
	
	
	cTD td_smc;
	ST_LPF lpf_smc;
	
	cSMC(){}
	cSMC(
		float EP,float Q, float C, //滑膜相关参数
		float UMax,float UMin,//限幅参数
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
