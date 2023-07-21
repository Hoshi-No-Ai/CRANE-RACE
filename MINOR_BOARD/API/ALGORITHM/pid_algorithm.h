#ifndef __PID_ALGORITHM_H__
#define __PID_ALGORITHM_H__

#define SQUARE(x)    ((x) * (x))

#include "stm32f4xx.h"
#include "math_algorithm.h"
#include "basic_type.h"
#include "math.h"

typedef struct
{
	float x1; //
	float x2;
	float x3;
	float x;
	float r;
	float h;
	float T;
	float aim;
} TD;

class cPID
{
	public :
	fp32 fpDes;//控制变量目标值
	fp32 fpFB;//控制变量反馈值

	fp32 fpKp;//比例系数Kp
	fp32 fpKi;//积分系数Ki
	fp32 fpPreKi;  //上一时刻的ki
	fp32 fpKd;//微分系数Kd

	fp32 fpE;//本次偏差
	fp32 fpPreE;//上次偏差
	fp32 fpSumE;//总偏差
	fp32 fpSumEMax;//累计误差最大值
	fp32 fpEpMax;//比例项输出最大值
	fp32 fpEiMax;//积分项输出最大值
	fp32 fpEdMax;//微分项输出最大值
	fp32 fpEMin;//积分误差小
	fp32 fpEMax;//积分误差大

	fp32 fpUp;//比例输出
	fp32 fpUi;//积分输出
	fp32 fpUd;//微分输出
	fp32 fpU;//本次PID运算结果
	fp32 fpUMax;//PID运算后输出最大值及做遇限削弱时的上限值
	fp32 fpTs;//PID控制周期，单位：s
	cPID(){}
	cPID(fp32 Kp, fp32 Ki, fp32 Kd, fp32 UpMax, fp32 EiMax, fp32 SumEMax, fp32 UdMax, fp32 EMin, fp32 EMax,fp32 ts)
	{
		this->fpDes = 0;
		this->fpFB = 0;
		
		this->fpKp = Kp;
		this->fpKi = Ki;
		this->fpKd = Kd;
		
		this->fpUp = 0;
		this->fpUi = 0;
		this->fpUd = 0;
		
		this->fpE  = 0;
		this->fpPreE = 0;
		this->fpSumE = 0;
		
		this->fpU = 0;
	  
		this->fpSumEMax = SumEMax;
		this->fpUMax = UpMax;
		this->fpEpMax = UpMax;
		this->fpEiMax = EiMax;
		this->fpEdMax = UdMax;
		this->fpEMin = EMin;
		this->fpEMax = EMax;
		this->fpTs   = ts;
	}// initialize PID
	void CalPID(void);
	void CalISeparatedPID(void);
	void CalIResistedPID(void);
	void CalIWeakenPID(void);
	void CalFilterPID(void);
	void CalComprehensivePID(void);
	
};

class cFeedForward
{
public:
	float ffU;

    float Kf_sin; 
    float Kf_cos;
    float FFangle;

    float ffU_Max;

    cFeedForward(){}
    cFeedForward(float _Kf1, float _Kf2, float _FFangle, float _ffU_Max)
	{
		this->Kf_sin = _Kf1;
		this->Kf_cos = _Kf2;
		this->FFangle = _FFangle;
		this->ffU_Max = _ffU_Max;
	}
    
    void CalGFeedforward(float angle);
};

void TD_Function(TD *ptd);
void Clip_TD_Function(TD *pstTd, fp32 lim_x2);


#endif
