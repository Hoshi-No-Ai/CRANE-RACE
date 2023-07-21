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
	fp32 fpDes;//���Ʊ���Ŀ��ֵ
	fp32 fpFB;//���Ʊ�������ֵ

	fp32 fpKp;//����ϵ��Kp
	fp32 fpKi;//����ϵ��Ki
	fp32 fpPreKi;  //��һʱ�̵�ki
	fp32 fpKd;//΢��ϵ��Kd

	fp32 fpE;//����ƫ��
	fp32 fpPreE;//�ϴ�ƫ��
	fp32 fpSumE;//��ƫ��
	fp32 fpSumEMax;//�ۼ�������ֵ
	fp32 fpEpMax;//������������ֵ
	fp32 fpEiMax;//������������ֵ
	fp32 fpEdMax;//΢����������ֵ
	fp32 fpEMin;//�������С
	fp32 fpEMax;//��������

	fp32 fpUp;//�������
	fp32 fpUi;//�������
	fp32 fpUd;//΢�����
	fp32 fpU;//����PID������
	fp32 fpUMax;//PID�����������ֵ������������ʱ������ֵ
	fp32 fpTs;//PID�������ڣ���λ��s
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
